import {FACES,moveForFace,parseSequence} from './controls.js';
const $=id=>document.getElementById(id);
const client=globalThis.crypto?.randomUUID?.()||`cube-${Date.now()}-${Math.random()}`;
let seq=0,connected=false,reverse=false,double=false,busy=false,polling=false,queue=[];
let toastTimer;
const phaseNames={GRASP:'建立抓持',READY:'等待动作',RELAX:'手腕退让',TURN:'转层中',SETTLE:'端点稳定',REGRASP:'重新抓持',RELEASED:'已释放'};

function toast(message){$('toast').textContent=message;$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').hidden=true,3200);}
function setConnection(value){connected=value;$('connection').classList.toggle('offline',!value);$('connection').querySelector('span').textContent=value?'实时仿真':'连接中断';if(value)$('loading').hidden=true;}
function send(payload){
  if(!connected){toast('请先连接本机 MuJoCo 服务');return;}
  if(queue.length>40){toast('网络指令过多，请稍后重试');return;}
  queue.push(payload);pump();
}
async function pump(){
  if(busy)return;busy=true;
  while(queue.length){
    const controller=new AbortController(),timeout=setTimeout(()=>controller.abort(),2500);
    try{
      const response=await fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({...queue.shift(),client,seq:++seq}),signal:controller.signal});
      const result=await response.json();if(!response.ok||result.error)throw new Error(result.error||'控制连接失败');
    }catch(error){queue=[];toast(error.name==='AbortError'?'连接超时，请确认启动窗口仍在运行':error.message);}
    finally{clearTimeout(timeout);}
  }
  busy=false;
}
function faceMove(face,inverse=reverse){
  const move=moveForFace(face,inverse,double);send({action:'turn',move});
  const button=document.querySelector(`[data-face="${face}"]`);button.classList.add('pressed');setTimeout(()=>button.classList.remove('pressed'),160);
}
function paintDirection(){
  $('clockwise').classList.toggle('selected',!reverse);$('clockwise').setAttribute('aria-pressed',String(!reverse));
  $('counterclockwise').classList.toggle('selected',reverse);$('counterclockwise').setAttribute('aria-pressed',String(reverse));
  $('double').classList.toggle('selected',double);$('double').setAttribute('aria-pressed',String(double));
}
$('clockwise').addEventListener('click',()=>{reverse=false;paintDirection();});
$('counterclockwise').addEventListener('click',()=>{reverse=true;paintDirection();});
$('double').addEventListener('click',()=>{double=!double;paintDirection();});
document.querySelectorAll('[data-face]').forEach(button=>{if(button.tagName==='BUTTON')button.addEventListener('click',()=>faceMove(button.dataset.face));});
document.querySelectorAll('[data-action]').forEach(button=>button.addEventListener('click',()=>{
  const action=button.dataset.action;if(action==='reset')queue=[];send({action});
}));
document.querySelectorAll('[data-camera]').forEach(button=>button.addEventListener('click',()=>send({action:'camera',mode:button.dataset.camera})));
$('sequence-form').addEventListener('submit',event=>{
  event.preventDefault();try{const moves=parseSequence($('sequence').value);send({action:'sequence',moves:moves.join(' ')});}catch(error){toast(error.message);}
});
window.addEventListener('keydown',event=>{
  if(event.repeat||event.ctrlKey||event.metaKey||event.altKey||/INPUT|TEXTAREA|SELECT/.test(event.target?.tagName)||event.target?.isContentEditable)return;
  const face=event.code.startsWith('Key')?event.code.slice(3):'';
  if(FACES.includes(face)){event.preventDefault();faceMove(face,event.shiftKey||reverse);}
  else if(event.code==='Space'){event.preventDefault();send({action:'pause'});}
  else if(event.code==='KeyS'){event.preventDefault();send({action:'scramble'});}
  else if(event.code==='KeyH'){event.preventDefault();send({action:'restore'});}
  else if(event.code==='KeyZ'){event.preventDefault();send({action:'undo'});}
  else if(event.code==='Escape'){event.preventDefault();queue=[];send({action:'clear'});}
});
$('fullscreen').addEventListener('click',async()=>{
  try{if(document.fullscreenElement)await document.exitFullscreen();else if(document.documentElement.requestFullscreen)await document.documentElement.requestFullscreen();else toast('浏览器不支持全屏');}catch{toast('浏览器未允许全屏');}
});
const stickers={};
for(const face of FACES){
  const panel=document.createElement('div');panel.className='net-face';panel.dataset.face=face;panel.title=face+' 面';
  stickers[face]=[];for(let i=0;i<9;i++){const cell=document.createElement('span');cell.className='sticker';panel.append(cell);stickers[face].push(cell);}
  $('cube-net').append(panel);
}
function update(state){
  $('phase').textContent=state.paused?'已暂停':phaseNames[state.phase]||state.phase;
  $('message').textContent=state.error||state.message;
  $('grasp').textContent=state.grasped?'已抓持':state.phase==='RELEASED'?'已释放':'建立中';
  $('contact-label').textContent=`${state.contacts} 个指尖接触`;
  $('completed').textContent=String(state.completed).padStart(2,'0');
  $('solved').textContent=state.current?'转层中':state.solved?'已还原':'已打乱';
  $('solved').style.color=state.solved?'var(--mint)':'#e6c582';
  $('solved-tag').textContent=state.current?'TURNING':state.solved?'SOLVED':'MIXED';$('solved-tag').classList.toggle('mixed',!state.solved);
  $('pause').textContent=state.paused?'▷ 继续':'Ⅱ 暂停';
  $('turn-overlay').hidden=!state.current;$('current-move').textContent=state.current||'';$('current-angle').textContent=state.turn_angle.toFixed(0)+'°';
  $('fps').textContent=state.render_fps.toFixed(0)+' FPS';
  const seconds=Math.floor(state.time);$('time').textContent=String(Math.floor(seconds/60)).padStart(2,'0')+':'+String(seconds%60).padStart(2,'0');
  $('queue-count').textContent=state.queue.length+' 待执行';
  const display=[...(state.current?[state.current]:[]),...state.queue];
  $('queue').replaceChildren();
  if(!display.length){const empty=document.createElement('span');empty.className='empty-queue';empty.textContent=state.solved?'选择一个面，开始下一组动作。':'队列已完成，可继续转层或逆序还原。';$('queue').append(empty);}
  else display.slice(0,32).forEach((move,index)=>{const chip=document.createElement('span');chip.className='move-chip'+(state.current&&index===0?' active':'');chip.textContent=move;$('queue').append(chip);});
  for(const face of FACES)state.faces[face].forEach((color,index)=>stickers[face][index].style.backgroundColor=color);
  document.querySelectorAll('[data-camera]').forEach(button=>{const active=button.dataset.camera===state.camera;button.classList.toggle('selected',active);button.setAttribute('aria-pressed',String(active));});
}
async function poll(){
  if(polling)return;polling=true;
  const controller=new AbortController(),timeout=setTimeout(()=>controller.abort(),2500);
  try{const response=await fetch('/api/state',{cache:'no-store',signal:controller.signal});if(!response.ok)throw new Error();const state=await response.json();setConnection(true);update(state);}
  catch{setConnection(false);}finally{clearTimeout(timeout);polling=false;}
}
$('stream').addEventListener('error',()=>{setConnection(false);setTimeout(()=>$('stream').src='/stream.mjpg?retry='+Date.now(),1800);});
poll();setInterval(poll,180);
