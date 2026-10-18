import {InputState, MOVEMENT, ACTIONS} from './input.js';

const $ = id => document.getElementById(id);
const input = new InputState();
const client = globalThis.crypto?.randomUUID?.() || `g1-${Date.now()}-${Math.random()}`;
let seq=0, connected=false, polling=false, busy=false, queue=[], lastGoals=0;
let toastTimer, celebrateTimer;
const keyButtons=[...document.querySelectorAll('[data-code]')];

function toast(message){
  $('toast').textContent=message; $('toast').hidden=false;
  clearTimeout(toastTimer); toastTimer=setTimeout(()=>$('toast').hidden=true,3200);
}
function setConnection(value){
  connected=value;
  $('connection').classList.toggle('offline',!value);
  $('connection').querySelector('span').textContent=value?'实时仿真':'连接中断';
  if(value)$('loading').hidden=true;
}
function paintKeys(){
  keyButtons.forEach(button=>{
    const held=input.pressed(button.dataset.code);
    button.classList.toggle('pressed',held);
    button.setAttribute('aria-pressed',String(held));
  });
}
function send(payload){
  // Coalesce only consecutive movement updates; preserve action ordering.
  if(payload.action==='keys' && queue.at(-1)?.action==='keys')queue[queue.length-1]=payload;
  else queue.push(payload);
  if(queue.length>24){ queue=[]; input.clear(); paintKeys(); queue.push({action:'stop'}); }
  pump();
}
async function pump(){
  if(busy)return;
  busy=true;
  while(queue.length){
    const payload={...queue.shift(),client,seq:++seq};
    const controller=new AbortController();
    const timeout=setTimeout(()=>controller.abort(),1800);
    try{
      const response=await fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload),signal:controller.signal});
      const result=await response.json();
      if(!response.ok || result.error)throw new Error(result.error||'控制连接中断');
    }catch(error){
      queue=[]; input.clear(); paintKeys(); setConnection(false);
      toast(error.name==='AbortError'?'连接超时，已停止发送走位指令':error.message);
    }finally{ clearTimeout(timeout); }
  }
  busy=false;
}
function sendKeys(){ if(connected)send({action:'keys',keys:input.actions()}); }
function releaseAll(beacon=false){
  input.clear();paintKeys();queue=[];
  if(beacon && navigator.sendBeacon){
    const body=JSON.stringify({action:'stop',client,seq:++seq});
    navigator.sendBeacon('/api/control',new Blob([body],{type:'application/json'}));
  }else if(connected)send({action:'stop'});
}
function act(action){
  if(!connected){toast('请先连接本机训练场');return;}
  if(['stop','reset','pause','demo'].includes(action)){input.clear();paintKeys();queue=[];}
  send({action});
  if(action==='demo')toast('已复位，开始自动走位与射门');
  if(action==='kick'){
    const button=document.querySelector('.kick-button');
    button.classList.add('firing');setTimeout(()=>button.classList.remove('firing'),180);
  }
}
function typing(event){ return /INPUT|TEXTAREA|SELECT/.test(event.target?.tagName)||event.target?.isContentEditable; }
window.addEventListener('keydown',event=>{
  if(typing(event)||event.ctrlKey||event.metaKey||event.altKey)return;
  if(MOVEMENT[event.code]){
    event.preventDefault();
    if(!connected)return;
    input.press(`key:${event.code}`,event.code);paintKeys();sendKeys();
  }else if(ACTIONS[event.code]){
    event.preventDefault();if(!event.repeat)act(ACTIONS[event.code]);
  }
});
window.addEventListener('keyup',event=>{
  if(MOVEMENT[event.code]){
    event.preventDefault();input.release(`key:${event.code}`);paintKeys();sendKeys();
  }
});
keyButtons.forEach(button=>{
  button.addEventListener('contextmenu',event=>event.preventDefault());
  button.addEventListener('pointerdown',event=>{
    event.preventDefault();if(!connected)return;
    button.setPointerCapture(event.pointerId);
    input.press(`pointer:${event.pointerId}`,button.dataset.code);paintKeys();sendKeys();
  });
  const release=event=>{input.release(`pointer:${event.pointerId}`);paintKeys();sendKeys();};
  button.addEventListener('pointerup',release);
  button.addEventListener('pointercancel',release);
  button.addEventListener('lostpointercapture',release);
  // Accessible keyboard activation: hold physical keys normally; Enter taps a key.
  button.addEventListener('keydown',event=>{
    if(event.code==='Enter'&&!event.repeat){
      event.preventDefault();input.press(`accessible:${button.dataset.code}`,button.dataset.code);paintKeys();sendKeys();
    }
  });
  button.addEventListener('keyup',event=>{
    if(event.code==='Enter'){input.release(`accessible:${button.dataset.code}`);paintKeys();sendKeys();}
  });
});
document.querySelectorAll('[data-action]').forEach(button=>button.addEventListener('click',()=>act(button.dataset.action)));
document.querySelectorAll('[data-camera]').forEach(button=>button.addEventListener('click',()=>{
  if(connected)send({action:'camera',mode:button.dataset.camera});
}));
$('fullscreen').addEventListener('click',async()=>{
  try{
    if(document.fullscreenElement)await document.exitFullscreen();
    else if(document.documentElement.requestFullscreen)await document.documentElement.requestFullscreen();
    else toast('当前浏览器不支持全屏');
  }catch{toast('当前浏览器未允许全屏');}
});
window.addEventListener('blur',()=>releaseAll(true));
window.addEventListener('pagehide',()=>releaseAll(true));
document.addEventListener('visibilitychange',()=>{if(document.hidden)releaseAll(true);});
setInterval(()=>{if(!document.hidden)sendKeys();},120);

function update(state){
  $('goals').textContent=String(state.goals).padStart(2,'0');
  $('shots').textContent=String(state.shots).padStart(2,'0');
  $('speed').textContent=state.speed.toFixed(2);
  $('ball-speed').textContent=state.ball_speed.toFixed(2);
  $('phase').textContent=state.paused?'已暂停':state.phase;
  $('event').textContent=state.error||state.event;
  $('fps').textContent=`${state.render_fps.toFixed(0)} FPS`;
  const seconds=Math.floor(state.time);
  $('time').textContent=`${String(Math.floor(seconds/60)).padStart(2,'0')}:${String(seconds%60).padStart(2,'0')}`;
  $('pause').querySelector('b').textContent=state.paused?'继续':'暂停';
  $('pause').querySelector('span').textContent=state.paused?'▷':'Ⅱ';
  $('kick-status').classList.toggle('ready',state.ready_to_kick);
  $('kick-status').querySelector('span').textContent=state.ready_to_kick?'足球已在脚前，按空格射门':'按 B 将足球放回脚前';
  document.querySelectorAll('[data-camera]').forEach(button=>{
    const active=button.dataset.camera===state.camera;
    button.classList.toggle('selected',active);button.setAttribute('aria-pressed',String(active));
  });
  if(state.goals>lastGoals){
    $('celebration').hidden=false;
    clearTimeout(celebrateTimer);celebrateTimer=setTimeout(()=>$('celebration').hidden=true,2100);
  }
  lastGoals=state.goals;
}
async function poll(){
  if(polling)return;polling=true;
  const controller=new AbortController();
  const timeout=setTimeout(()=>controller.abort(),2000);
  try{
    const response=await fetch('/api/state',{cache:'no-store',signal:controller.signal});
    if(!response.ok)throw new Error('State unavailable');
    const state=await response.json();setConnection(true);update(state);
  }catch{
    setConnection(false);input.clear();paintKeys();
  }finally{clearTimeout(timeout);polling=false;}
}
$('stream').addEventListener('error',()=>{
  setConnection(false);
  setTimeout(()=>$('stream').src='/stream.mjpg?retry='+Date.now(),1800);
});
poll();setInterval(poll,200);
