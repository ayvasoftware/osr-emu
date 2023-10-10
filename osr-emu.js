
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import * as THREE from 'three';
import Axis from './lib/axis.js';
import OSR2Model from './lib/models/osr2/osr2.js';

const cameraPosition = { x: 245.03116537126925, y: 427.3026288938325, z: 190.74318476308787 };
const cameraTarget = { x: -36.81235747311321, y: 6.186822929643268, z: 15.874049352801714 };

const COMMAND_REGEX = /^(L0|L1|L2|R0|R1|R2)([0-9]+)$/;
const COMMAND_EXTENSION_REGEX = /^(L0|L1|L2|R0|R1|R2)([0-9]+)(I|S)([0-9]+)$/;

class OSREmulator {
  #buffer = '';

  #axisEmulator = {
    'L0': new Axis('L0'), // Stroke
    'L1': new Axis('L1'), // Forward
    'L2': new Axis('L2'), // Left
    'R0': new Axis('R0'), // Twist
    'R1': new Axis('R1'), // Roll
    'R2': new Axis('R2'), // Pitch
  };

  #scale = {
    'L0': 1, // Stroke
    'L1': 1, // Forward
    'L2': 1, // Left
    'R0': 1, // Twist
    'R1': 1, // Roll
    'R2': 1, // Pitch
  };

  #element;
  #osrModel;

  #resizeObserver;
  #boundResizeListener;
  #animationFrameRequestId;

  /**
   * Get a map of axis name to current decimal value (between zero and one).
   */
  get axes () {
    const result = {};

    Object.keys(this.#axisEmulator).forEach(axis => {
      result[axis] = this.#axisEmulator[axis].getPosition() / 10000;
    });

    return result;
  }

  constructor (element, options) {
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

    if (options && options.scale) {
      this.#scale = { ...this.#scale, ...options.scale };
    }

    this.#initCanvas();
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

  destroy () {
    this.#resizeObserver.unobserve(this.#element);
    window.removeEventListener('resize', this.#boundResizeListener);
    window.cancelAnimationFrame(this.#animationFrameRequestId);
    this.#element.innerHTML = '';
    this.renderer.dispose();
    this.renderer.forceContextLoss();
  }

  #executeCommand (buffer) {
    const commands = buffer.trim().split(/\s/).map(c => c.trim());
    const parseValue = value => Number(value.substring(0, 4).padEnd(4, '0'));

    for (const command of commands) {
      if (COMMAND_REGEX.test(command)) {
        const match = COMMAND_REGEX.exec(command);
        const axis = match[1];
        const value = match[2];

        this.#axisEmulator[axis].set(parseValue(value));
      } else if (COMMAND_EXTENSION_REGEX.test(command)) {
        const match = COMMAND_EXTENSION_REGEX.exec(command);
        const axis = match[1];
        const value = match[2];
        const ext = match[3];
        const extValue = match[4];

        this.#axisEmulator[axis].set(parseValue(value), ext, Number(extValue));
      }
    }
  }

  #initCanvas () {
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
    this.#loadModel(scene);

    this.#element.innerHTML = '';
    this.#element.appendChild(renderer.domElement);

    this.#buffer = '';

    this.#resize();
    this.#animate();

    this.#boundResizeListener = this.#resize.bind(this);
    this.#resizeObserver = new ResizeObserver(this.#boundResizeListener);
    this.#resizeObserver.observe(this.#element);
    window.addEventListener('resize', this.#boundResizeListener);
  }

  #render () {
    this.renderer.render(this.scene, this.camera);
  }

  #animate () {
    this.#animationFrameRequestId = requestAnimationFrame(this.#animate.bind(this));
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

  #loadModel (scene) {
    this.#osrModel = new OSR2Model();
    
    const osrGroup = new THREE.Group();

    const { meshes, orientation } = this.#osrModel.load();

    for (const mesh of Object.values(meshes)) {
      osrGroup.add(mesh);
    }

    osrGroup.rotation.set(orientation, 0, 0);
    scene.add(osrGroup);
  }

  #updatePositions () {
    this.#osrModel.preRender(this.axes, this.#scale);
    this.#render();
    this.#osrModel.postRender(this.axes, this.#scale);
  }
}

// Allow importing as named or default.
// TODO: Remove default export once updated elsewhere.
export default OSREmulator;
export { OSREmulator };
