import { Vector3, MeshPhongMaterial } from 'three';
import { deg, loadObj } from '../../util.js';
import mainArmGeometry from './geometry/arm.js';
import baseGeometry from './geometry/base.js';
import caseGeometry from './geometry/case.js';
import lidGeometry from './geometry/lid.js';
import pitcherArmGeometry from './geometry/pitcher-arm.js';
import pitcherGeometry from './geometry/pitcher.js';
import receiverGeometry from './geometry/receiver.js';
import supportArmGeometry from './geometry/support-arm.js';

/**
 * 3D Model of the OSR2
 */
export default class OSR2Model {
  #armAxisLength = 139.5;
  #armZOffset = 18.22;
  #armYOffset = -5;
  #supportAxisLength = 140;
  #orientation = -Math.PI / 2; // Rotation around the x-axis.


  load () {
    const receiverMaterial = new MeshPhongMaterial({ color: 0x3f5173 });
    const baseMaterial = new MeshPhongMaterial({ color: 0x1E1E1E });

    const modCase = loadObj(caseGeometry, baseMaterial); // ?
    const base = loadObj(baseGeometry, baseMaterial);
    const receiver = loadObj(receiverGeometry, receiverMaterial); // ?
    const rightArm = loadObj(mainArmGeometry, baseMaterial); // ?
    const leftArm = loadObj(mainArmGeometry, baseMaterial); // ?
    const supportArm = loadObj(supportArmGeometry, baseMaterial); // ?
    const lid = loadObj(lidGeometry, receiverMaterial);
    const pitcher = loadObj(pitcherGeometry, baseMaterial);
    const pitcherArm = loadObj(pitcherArmGeometry, baseMaterial);

    pitcherArm.position.set(73.973, 6.3454, 18.857);

    this.objects = {
      base,
      receiver,
      leftArm,
      rightArm,
      supportArm,
      lid,
      pitcher,
      pitcherArm,
      modCase,
    };

    return { objects: this.objects, orientation: this.#orientation };
  }

  preRender (axes, scale) {
    const L0_SCALE = scale['L0'];
    const R0_SCALE = scale['R0'];
    const R1_SCALE = scale['R1'];
    const R2_SCALE = scale['R2'];

    const receiverAxisLength = this.#armAxisLength + this.#armZOffset;
    const theta = axes['L0'] * deg(90 * L0_SCALE) - deg(45 * L0_SCALE);
    
    const y = -receiverAxisLength * Math.sin(theta);
    const z = receiverAxisLength * Math.cos(theta);

    const roll = deg(30 * R1_SCALE) - axes['R1'] * deg(60 * R1_SCALE);
    const pitch = axes['R2'] * deg(60 * R2_SCALE) - deg(30 * R2_SCALE);
    this.objects.receiver.position.set(0, y + this.#armYOffset, z);
    this.objects.receiver.rotation.set(pitch, 0, roll);

    this.objects.modCase.position.set(0, y + this.#armYOffset, z);
    this.objects.modCase.rotation.set(deg(90) + pitch, roll, axes['R0'] * deg(240 * R0_SCALE) - deg(120  * R0_SCALE));
  }

  postRender (axes) {
    // We need to do this calculation AFTER the render because the 
    // arm orientation depends on the new world coordinates of the receiver.
    this.#calculateArm(axes, this.objects.rightArm, this.#armAxisLength, -67, this.#armYOffset, 18.22, -65, 0, 0);
    this.#calculateArm(axes, this.objects.leftArm, this.#armAxisLength, 67, this.#armYOffset, 18.22, 63, 0, 0);
    this.#calculateArm(axes, this.objects.supportArm, this.#supportAxisLength, 66.385, -33.243 + this.#armYOffset, 18.558, 64, -40, 0);
  }

  #calculateArm(axes, arm, armLength, ax, ay, az, rx, ry, rz) {
    const receiverPosition = this.#receiverToWorldCoordinates(rx, ry, rz);
    const armPosition = new Vector3(ax, ay, az)
      .sub(receiverPosition)
      .normalize()
      .multiplyScalar(armLength)
      .add(receiverPosition);

    const x = receiverPosition.x - armPosition.x;
    const y = receiverPosition.y - armPosition.y;
    const z = receiverPosition.z - armPosition.z;

    const pitch = z !== 0 ? -Math.atan(y / z) : 0;
    const yaw = z !== 0 ? Math.atan(x / z) : 0;
    const roll = axes['R1'] * deg(60) - deg(30);

    arm.position.set(armPosition.x, armPosition.y, armPosition.z);
    arm.rotation.set(pitch, yaw, roll);
  }

  #receiverToWorldCoordinates (x, y, z) {
    const angle = -this.#orientation;
    const world = this.objects.receiver.localToWorld(new Vector3(x, y, z));

    return new Vector3(
      world.x,
      world.y * Math.cos(angle) - world.z * Math.sin(angle),
      world.y * Math.sin(angle) + world.z * Math.cos(angle),
    );
  }
}