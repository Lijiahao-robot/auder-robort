import assert from 'node:assert/strict';
import {moveForFace,parseSequence,FACES} from '../web/controls.js';
for(const face of FACES){assert.equal(moveForFace(face),face);assert.equal(moveForFace(face,true),face+"'");assert.equal(moveForFace(face,false,true),face+'2');}
assert.deepEqual(parseSequence("R U R' U'"),['R','U',"R'","U'"]);
assert.deepEqual(parseSequence('  F2\nD\tL\' '),['F2','D',"L'"]);
assert.throws(()=>parseSequence('R’'));assert.throws(()=>parseSequence('R3'));assert.throws(()=>parseSequence(''));
assert.throws(()=>parseSequence(Array(41).fill('R').join(' ')));assert.throws(()=>moveForFace('X'));
console.log('Controls passed: 6 faces, inverses, half turns, sequences, whitespace and invalid input rejection.');
