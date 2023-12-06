import { MeshPhongMaterial, MeshStandardMaterial } from 'three';
import { 
  loadObj, recomputeNormals
} from '../../util.js';
import baseGeometry from './geometry/base.js';
import receiverGeometry from './geometry/receiver.js';
import railsGeometry from './geometry/rails.js'

/**
 * 3D Model of the SSR1
 */
export default class SSR1Model {
  #orientation = -Math.PI / 2; // Rotation around the x-axis.

  load () {
    const ayvaBlueMaterial = new MeshPhongMaterial({ color: 0x3f5173 });
    const darkMaterial = new MeshPhongMaterial({ color: 0x1E1E1E });
    const railMaterial = new MeshStandardMaterial({
      color: 0xC0C0C0,
      metalness: 1,
      roughness: 0.5, 
    });

    const base = loadObj(baseGeometry, darkMaterial);
    const receiver = loadObj(receiverGeometry, ayvaBlueMaterial);
    const rails = loadObj(railsGeometry, railMaterial, recomputeNormals);

    this.objects = {
      base,
      receiver,
      rails,
    };

    return { objects: this.objects, orientation: this.#orientation };
  }

  preRender (axes, scale) {
    const position = -scale['L0'] * axes['L0'] * 120; // 120mm stroke
    this.objects.receiver.position.y = position;
  }
}
