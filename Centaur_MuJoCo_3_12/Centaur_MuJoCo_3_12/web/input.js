export function normalizeKey(value){
  const aliases={KeyW:'w',KeyS:'s',KeyA:'a',KeyD:'d',KeyQ:'q',KeyE:'e',ArrowUp:'w',ArrowDown:'s',ArrowLeft:'a',ArrowRight:'d',Space:'space',' ':'space'};
  return aliases[value] || (typeof value==='string'&&['w','s','a','d','q','e'].includes(value.toLowerCase())?value.toLowerCase():null);
}
export function velocityForKeys(keys){
  let x=Number(keys.has('w'))-Number(keys.has('s'));
  let y=Number(keys.has('a'))-Number(keys.has('d'));
  const length=Math.max(1,Math.hypot(x,y));
  return {vx:0.12*x/length,vy:0.08*y/length,yaw:0.30*(Number(keys.has('q'))-Number(keys.has('e')))};
}
