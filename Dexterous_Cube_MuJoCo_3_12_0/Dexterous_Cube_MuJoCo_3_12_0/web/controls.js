export const FACES=Object.freeze(['U','R','F','D','L','B']);
export function moveForFace(face,inverse=false,double=false){
  if(!FACES.includes(face))throw new Error('请选择 U/R/F/D/L/B 中的一个面');
  return face+(double?'2':inverse?"'":'');
}
export function parseSequence(value){
  if(typeof value!=='string'||!value.trim())throw new Error('请输入动作序列');
  const moves=value.trim().split(/\s+/);
  if(moves.length>40)throw new Error('一次最多执行 40 步');
  if(moves.some(move=>! /^[URFDLB](?:'|2)?$/.test(move)))throw new Error("动作格式为 R U R' U'，使用英文单引号");
  return moves;
}
