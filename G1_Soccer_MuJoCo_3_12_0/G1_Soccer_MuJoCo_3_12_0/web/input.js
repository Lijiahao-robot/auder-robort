export const MOVEMENT = Object.freeze({
  KeyW:'forward', ArrowUp:'forward', KeyS:'back', ArrowDown:'back',
  KeyA:'left', KeyD:'right', KeyQ:'turn_left', ArrowLeft:'turn_left',
  KeyE:'turn_right', ArrowRight:'turn_right', ShiftLeft:'boost', ShiftRight:'boost'
});
export const ACTIONS = Object.freeze({Space:'kick', KeyK:'kick', KeyP:'pause', KeyR:'reset', KeyB:'ball', KeyX:'stop'});

// Physical keys and pointer contacts have separate identities, so releasing one
// input never releases another input with the same logical direction.
export class InputState {
  constructor(){ this.sources = new Map(); }
  press(source, code){ if(MOVEMENT[code])this.sources.set(source,code); }
  release(source){ this.sources.delete(source); }
  clear(){ this.sources.clear(); }
  actions(){ return [...new Set([...this.sources.values()].map(code=>MOVEMENT[code]))].sort(); }
  pressed(code){ return this.actions().includes(MOVEMENT[code]); }
}
