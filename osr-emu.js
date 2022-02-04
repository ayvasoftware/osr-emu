import { OBJLoader } from 'https://cdn.jsdelivr.net/npm/three@0.127.0/examples/jsm/loaders/OBJLoader.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.127.0/examples/jsm/controls/OrbitControls.js';
import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.127.0/build/three.module.js';
import Axis from './axis.js';
import mainArmGeometry from './models/arm.js';
import baseGeometry from './models/base.js';
import caseGeometry from './models/case.js';
import lidGeometry from './models/lid.js';
import pitcherArmGeometry from './models/pitcher-arm.js';
import pitcherGeometry from './models/pitcher.js';
import receiverGeometry from './models/receiver.js';
import supportArmGeometry from './models/support-arm.js';

const cameraPosition = { x: 245.03116537126925, y: 427.3026288938325, z: 190.74318476308787 };
const cameraTarget = { x: -36.81235747311321, y: 6.186822929643268, z: 15.874049352801714 };

const deg = THREE.MathUtils.degToRad;
const receiverMaterial = new THREE.MeshLambertMaterial({ color: 0x3f5173 });
const baseMaterial = new THREE.MeshLambertMaterial({ color: 0x1E1E1E });

const COMMAND_REGEX = /^(L0|L1|L2|R0|R1|R2)([0-9]+)$/;
const COMMAND_EXTENSION_REGEX = /^(L0|L1|L2|R0|R1|R2)([0-9]+)(I|S)([0-9]+)$/;
const Vector3 = THREE.Vector3;

export default class OSREmulator {
  #buffer = '';

  #axes = {
    'L0': new Axis('L0'), // Stroke
    'L1': new Axis('L1'), // Forward
    'L2': new Axis('L2'), // Left
    'R0': new Axis('R0'), // Twist
    'R1': new Axis('R1'), // Roll
    'R2': new Axis('R2'), // Pitch
  };

  #element;
  #orientation = -Math.PI / 2; // Rotation around the x-axis.
  #armAxisLength = 139.5;
  #armZOffset = 18.22;
  #armYOffset = -5;
  #supportAxisLength = 140;
  #objLoader = new OBJLoader();

  get axes () {
    const result = {};

    Object.keys(this.#axes).forEach(axis => {
      result[axis] = this.#axes[axis].getPosition() / 10000;
    });

    return result;
  }

  constructor (element) {
    if (element instanceof HTMLElement) {
      this.#element = element;
    } else if (typeof element === 'string' || element instanceof String) {
      this.#element = document.querySelector(element);

      if (!this.#element) {
        throw new Error(`Element not found: ${element}`);
      }
    } else {
      throw new Error(`Invalid element: ${element}`);
    }

    this.#initCanvas(this.#element);
  }

  write (input) {
    if (typeof input !== 'string') {
      return;
    }

    for (let byte of input) {
      this.#buffer += byte;

      if (byte === '\n') {
        this.#executeCommand(this.#buffer);
        this.#buffer = '';
      }
    }
  }

  #executeCommand (buffer) {
    const commands = buffer.trim().split(/\s/).map(c => c.trim());
    const parseValue = value => Number(value.substring(0, 4).padEnd(4, '0'));

    for (const command of commands) {
      if (COMMAND_REGEX.test(command)) {
        const match = COMMAND_REGEX.exec(command);
        const axis = match[1];
        const value = match[2];

        this.#axes[axis].set(parseValue(value));
      } else if (COMMAND_EXTENSION_REGEX.test(command)) {
        const match = COMMAND_EXTENSION_REGEX.exec(command);
        const axis = match[1];
        const value = match[2];
        const ext = match[3];
        const extValue = match[4];

        this.#axes[axis].set(parseValue(value), ext, Number(extValue));
      } else {
        console.warn(`Cannot execute command '${command}'. Only updates on linear or rotation channels 0-2 are supported.`);
      }
    }
  }

  #initCanvas (element) {
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(50, this.#computeAspectRatio(), 0.1, 1000);
    const renderer = new THREE.WebGLRenderer();
    camera.position.set(cameraPosition.x, cameraPosition.y, cameraPosition.z);
    camera.up.set(0, 0, 1);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.maxDistance = 700;
    controls.target.set(cameraTarget.x, cameraTarget.y, cameraTarget.z);
    controls.update();

    this.camera = camera;
    this.renderer = renderer;
    this.scene = scene;
    this.controls = controls;

    this.#setupLighting(scene);
    this.#loadAllModels(scene);

    element.appendChild(renderer.domElement);

    this.#buffer = '';

    this.#resize();
    this.#animate();

    const onResize = this.#resize.bind(this);
    new ResizeObserver(onResize).observe(this.#element);
    window.addEventListener('resize', onResize);
  }

  #render () {
    this.renderer.render(this.scene, this.camera);
  }

  #animate () {
    requestAnimationFrame(this.#animate.bind(this));
    this.controls.update();
    this.#updatePositions();
    this.#render();
  }

  #resize () {
    const viewport = this.#element.getBoundingClientRect();
    this.camera.aspect = this.#computeAspectRatio();
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(viewport.width, viewport.height);
  }

  #computeAspectRatio () {
    const viewport = this.#element.getBoundingClientRect();
    return viewport.width / viewport.height;
  }

  #setupLighting (scene) {
    const sunlight = new THREE.DirectionalLight(0xFFFFFF, 1);
    sunlight.position.set(-1, 1, 1);
    scene.add(new THREE.AmbientLight(0xAAAAAA));
    scene.add(sunlight);
  }

  #loadObj (text, material) {
    const object = this.#objLoader.parse(text);

    object.traverse(function(child) {
      if (child instanceof THREE.Mesh) {
        child.material = material;
      }
    });

    return object;
  }

  #loadAllModels (scene) {
    const modCase = this.#loadObj(caseGeometry, baseMaterial);
    const base = this.#loadObj(baseGeometry, baseMaterial);
    const receiver = this.#loadObj(receiverGeometry, receiverMaterial);
    const rightArm = this.#loadObj(mainArmGeometry, baseMaterial);
    const leftArm = this.#loadObj(mainArmGeometry, baseMaterial);
    const supportArm = this.#loadObj(supportArmGeometry, baseMaterial);
    const lid = this.#loadObj(lidGeometry, receiverMaterial);
    const pitcher = this.#loadObj(pitcherGeometry, baseMaterial);
    const pitcherArm = this.#loadObj(pitcherArmGeometry, baseMaterial);

    pitcherArm.position.set(73.973, 6.3454, 18.857);

    const osrGroup = new THREE.Group();

    this.meshes = {
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

    osrGroup.add(receiver);
    osrGroup.add(leftArm);
    osrGroup.add(rightArm);
    osrGroup.add(base);
    osrGroup.add(lid);
    osrGroup.add(supportArm);
    osrGroup.add(pitcher);
    osrGroup.add(pitcherArm);
    osrGroup.add(modCase);
    osrGroup.rotation.set(this.#orientation, 0, 0);
    scene.add(osrGroup);
  }

  #updatePositions () {
    this.#calculateReceiverPosition();
    this.#render(); // We need the world coordinates of the receiver to update because the arm orientation depends on them.
    this.#calculateArmPositions();
  }

  #calculateReceiverPosition () {
    const receiverAxisLength = this.#armAxisLength + this.#armZOffset;
    const theta = this.axes['L0'] * deg(90) - deg(45);
    
    const y = -receiverAxisLength * Math.sin(theta);
    const z = receiverAxisLength * Math.cos(theta);

    const roll = this.axes['R1'] * deg(60) - deg(30);
    const pitch = this.axes['R2'] * deg(60) - deg(30);
    this.meshes.receiver.position.set(0, y + this.#armYOffset, z);
    this.meshes.receiver.rotation.set(pitch, 0, roll);

    this.meshes.modCase.position.set(0, y + this.#armYOffset, z);
    this.meshes.modCase.rotation.set(deg(90) + pitch, roll, this.axes['R0'] * deg(240) - deg(120));
  }

  #calculateArmPositions () {
    this.#calculateArm(this.meshes.rightArm, this.#armAxisLength, -67, this.#armYOffset, 18.22, -65, 0, 0);
    this.#calculateArm(this.meshes.leftArm, this.#armAxisLength, 67, this.#armYOffset, 18.22, 63, 0, 0);
    this.#calculateArm(this.meshes.supportArm, this.#supportAxisLength, 66.385, -33.243 + this.#armYOffset, 18.558, 64, -40, 0);
  }

  #calculateArm(arm, armLength, ax, ay, az, rx, ry, rz) {
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
    const roll = this.axes['R1'] * deg(60) - deg(30);

    arm.position.set(armPosition.x, armPosition.y, armPosition.z);
    arm.rotation.set(pitch, yaw, roll);
  }

  #receiverToWorldCoordinates (x, y, z) {
    const angle = -this.#orientation;
    const world = this.meshes.receiver.localToWorld(new Vector3(x, y, z));

    return new Vector3(
      world.x,
      world.y * Math.cos(angle) - world.z * Math.sin(angle),
      world.y * Math.sin(angle) + world.z * Math.cos(angle),
    );
  }
}
