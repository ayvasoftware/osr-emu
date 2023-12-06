
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import * as THREE from 'three';
import Axis from './lib/axis.js';
import { forEachMesh } from './lib/util.js';
import OSR2Model from './lib/models/osr2/osr2.js';
import SR6Model from './lib/models/sr6/sr6.js';
import SSR1Model from './lib/models/ssr1/ssr1.js';

const modelInfoByType = {
  'SR6': {
    constructor: SR6Model,
    cameraPosition: new THREE.Vector3(262.1988183293872, 487.3523930568582, 194.12937273009945),
    controlTarget: new THREE.Vector3(-62.330470689438, 28.868408052774832, -17.219160432091634),
  },

  'OSR2': {
    constructor: OSR2Model,
    cameraPosition: new THREE.Vector3(245.03116537126925, 427.3026288938325, 190.74318476308787),
    controlTarget: new THREE.Vector3(-36.81235747311321, 6.186822929643268, 15.874049352801714),
  },

  'SSR1': {
    constructor: SSR1Model,
    cameraPosition: new THREE.Vector3(223.80161934006662, 284.7018829005695, 262.21634318512554),
    controlTarget: new THREE.Vector3(13.385718044675617, -66.86882881039317, 111.35424951515378),
  },
}

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
  #modelType;
  #sceneHelpers;
  #objectHelpers;

  #resizeObserver;
  #boundResizeListener;
  #animationFrameRequestId;

  get osrModel () {
    return this.#osrModel;
  }

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

  get objectHelpers () {
    return this.#objectHelpers;
  }

  get sceneHelpers () {
    return this.#sceneHelpers;
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

    this.#scale = { ...this.#scale, ...(options?.scale || {}) };
    this.#sceneHelpers = options?.sceneHelpers ? [] : null;
    this.#objectHelpers = options?.objectHelpers ? [] : null;
    this.#modelType = (options?.model ?? 'OSR2').toUpperCase();

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

    const modelInfo = modelInfoByType[this.#modelType];

    camera.position.copy(modelInfo.cameraPosition);
    camera.up.set(0, 0, 1);

    const renderer = new THREE.WebGLRenderer();
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap; 

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.maxDistance = 700;
    controls.target.copy(modelInfo.controlTarget);
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
    const keyLight = new THREE.PointLight(0xFFFFFF);
    keyLight.intensity = 0.75;
    keyLight.position.set(200, 180, 50);
    keyLight.castShadow = true;

    const fillLight = new THREE.DirectionalLight(0xFFFFFF, 1);
    fillLight.intensity = 1;
    fillLight.position.set(-225, 225, 225);
    fillLight.castShadow = true;
    for (const side of ['left', 'right', 'bottom', 'top']) {
      fillLight.shadow.camera[side] *= 50;
    }

    const backLight = new THREE.PointLight(0xFFFFFF);
    backLight.intensity = 1;
    backLight.position.set(70, -100, 200);
    backLight.castShadow = true;

    const ambientLight = new THREE.AmbientLight(0xFFFFFF, 0.625);

    this.keyLight = keyLight;
    this.fillLight = fillLight;
    this.backLight = backLight;
    this.ambientLight = ambientLight;
    
    if (this.#modelType === 'SSR1') {
      // Slight adjustents to lighting for SSR1...
      this.keyLight.position.z = 220;
      this.backLight.position.y = -200;
    }

    scene.add(keyLight);
    scene.add(fillLight);
    scene.add(backLight);
    scene.add(ambientLight);

    if (this.#sceneHelpers) {
      this.#sceneHelpers = this.#sceneHelpers.concat([
        new THREE.DirectionalLightHelper(fillLight, 100),
        new THREE.CameraHelper(fillLight.shadow.camera),
        new THREE.PointLightHelper( keyLight, 5 ),
        new THREE.PointLightHelper( backLight, 5 ),
        new THREE.AxesHelper( 500 )
      ]);

      this.#sceneHelpers.forEach(h => scene.add(h));
    }
  }

  #loadModel (scene) {
    const Model = modelInfoByType[this.#modelType].constructor;

    this.#osrModel = new Model();
    
    const osrGroup = new THREE.Group();

    const { objects, orientation } = this.#osrModel.load();

    for (const object of Object.values(objects)) {
      forEachMesh(object, (mesh) => {
        mesh.receiveShadow = true;
        mesh.castShadow = true;
      });

      osrGroup.add(object);

      if (this.#objectHelpers) {
        const helper = new THREE.AxesHelper(100);

        osrGroup.add(helper);
        
        this.#objectHelpers.push({
          object,
          helper,
        });
      }
    }

    osrGroup.rotation.set(orientation, 0, 0);
    scene.add(osrGroup);
  }

  #updatePositions () {
    this.#osrModel.preRender(this.axes, this.#scale);
    this.#render();

    // Update the position of object helpers if enabled.
    if (this.#objectHelpers) {
      for (const { object, helper } of this.#objectHelpers) {
        helper.position.set(object.position.x, object.position.y, object.position.z);
        helper.rotation.set(object.rotation.x, object.rotation.y, object.rotation.z);
      }
    }
  }
}

// Allow importing as named or default.
// TODO: Remove default export once updated elsewhere.
export default OSREmulator;
export { OSREmulator };
