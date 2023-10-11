import { Material, MathUtils, Mesh } from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';

const objLoader = new OBJLoader();

export const deg = MathUtils.degToRad;

/**
 * Load a model in .OBJ format from a String and apply the specified material.
 * 
 * @param {String} text 
 * @param {Material} material 
 * @returns 
 */
export function loadObj (text, material) {
  const object = objLoader.parse(text);

  forEachMesh(object, (mesh) => {
    mesh.material = material;
  });

  return object;
}

/**
 * Call a function for each mesh of the specified OBJ.
 * 
 * @param {Object} object 
 * @param {Function} fn 
 */
export function forEachMesh (object, fn) {
  object.traverse(function(child) {
    if (child instanceof Mesh) {
      fn(child);
    }
  });
}

/**
 * https://www.arduino.cc/reference/en/language/functions/math/constrain/
 */
export function constrain (x, a, b) {
  return Math.max(a, Math.min(x, b));
}

/**
 * https://www.arduino.cc/reference/en/language/functions/math/map/
 */
export function map (x, in_min, in_max, out_min, out_max) {
  return Math.floor((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}