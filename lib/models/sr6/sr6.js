import { Vector3, Quaternion, MeshPhongMaterial, MeshStandardMaterial } from 'three';
import { loadObj, loadObjs, mapDecimal, servoToRotation, constrain, rad, circleSphereIntersection, vectorDirection, recomputeNormals } from '../../util.js';
import baseGeometry from './geometry/base.js';
import lidGeometry from './geometry/lid.js';
import mainArmServoGeometry from './geometry/main-arm-servos.js';
import armGeometry from './geometry/arm.js';
import leftPitcherGeometry from './geometry/left-pitcher.js';
import rightPitcherGeometry from './geometry/right-pitcher.js';

/**
 * 3D Model of the SR6
 */
export default class SR6Model {
  #orientation = -Math.PI / 2; // Rotation around the x-axis.

  load () {
    const receiverMaterial = new MeshPhongMaterial({ color: 0x3f5173 });
    const baseMaterial = new MeshPhongMaterial({ color: 0x1E1E1E });
    const servoMaterial = new MeshPhongMaterial({ color: 0x131313 });

    const base = loadObj(baseGeometry, baseMaterial);
    const mainArmServos = loadObj(mainArmServoGeometry, servoMaterial);
    const lid = loadObj(lidGeometry, receiverMaterial);

    const [
      lowerRightArm,
      upperRightArm,
      lowerLeftArm,
      upperLeftArm,
    ] = loadObjs(4, armGeometry, receiverMaterial);

    const leftPitcher = loadObj(leftPitcherGeometry, receiverMaterial);
    const rightPitcher = loadObj(rightPitcherGeometry, receiverMaterial);

    const armZ = 59.5 - 14 - 3;
    upperRightArm.position.fromArray([-59.3, 0, armZ]);
    lowerRightArm.position.fromArray([-59.3, 30, armZ]);
    lowerRightArm.rotation.x = Math.PI;

    upperLeftArm.position.fromArray([59.3, 0, armZ]);
    upperLeftArm.rotation.y = Math.PI;

    lowerLeftArm.position.fromArray([59.3, 30, armZ]);
    lowerLeftArm.rotation.x = Math.PI;
    lowerLeftArm.rotation.y = Math.PI;

    leftPitcher.position.fromArray([14.318, -29.72, 49.325]);
    rightPitcher.position.fromArray([-14.318, -29.72, 49.325]);

    this.objects = {
      base,
      lid,
      mainArmServos,
      lowerRightArm,
      upperRightArm,
      lowerLeftArm,
      upperLeftArm,
      leftPitcher,
      rightPitcher,
    };

    return { objects: this.objects, orientation: this.#orientation };
  }

  preRender (axes, scale) {
    this.#forwardKinematics(axes, scale);
  }

  postRender (axes) {
    // No post render...
  }

  #forwardKinematics (axes, scale) {
    const { 
      lowerLeftServoAngle,
      upperLeftServoAngle,
      lowerRightServoAngle,
      upperRightServoAngle,
      leftPitchServoAngle,
      rightPitchServoAngle,
    } = this.#computeServoAngles(axes, scale);

    const { 
      lowerRightArm,
      upperRightArm,
      lowerLeftArm,
      upperLeftArm,
      leftPitcher,
      rightPitcher,
    } = this.objects;

    lowerRightArm.rotation.x = lowerRightServoAngle;
    upperRightArm.rotation.x = upperRightServoAngle;
    lowerLeftArm.rotation.x = lowerLeftServoAngle;
    upperLeftArm.rotation.x = upperLeftServoAngle;
    leftPitcher.rotation.x = leftPitchServoAngle;
    rightPitcher.rotation.x = rightPitchServoAngle;
  }

  /**
   * Compute the angles for the servos based on the algorithm from the actual SR6 Firmware.
   */
  #computeServoAngles (axes, scale) {
    const servoZero = 1515.105;
    const servoFrequency = 330;
    const servoInterval = 1000000 / servoFrequency;
    const msPerRad = 637;

    const setMainServo = (x, y) => {
      // Function to calculate the angle for the main arm servos
      // Inputs are target x,y coords of receiver pivot in 1/100 of a mm
      x /= 100; y /= 100;               // Convert to mm
      const gamma = Math.atan2(x, y);   // Angle of line from servo pivot to receiver pivot
      const csq = x*x + y*y;            // Square of distance between servo pivot and receiver pivot
      const c = Math.sqrt(csq);         // Distance between servo pivot and receiver pivot
      const beta = Math.acos((csq - 28125)/(100*c));  // Angle between c-line and servo arm
      return msPerRad*(gamma + beta - 3.14159);       // Servo signal output, from neutral
    };

    const setPitchServo = (x, y, z, pitch) => {
      // Function to calculate the angle for the pitcher arm servos
      // Inputs are target x,y,z coords of receiver upper pivot in 1/100 of a mm
      // Also pitch in 1/100 of a degree
      pitch *= 0.0001745; // Convert to radians
      x += 5500*Math.sin(0.2618 + pitch);
      y -= 5500*Math.cos(0.2618 + pitch);
      x /= 100; y /= 100; z /= 100;             // Convert to mm
      const bsq = 36250 - (75 + z)*(75 + z);    // Equivalent arm length
      const gamma = Math.atan2(x,y);            // Angle of line from servo pivot to receiver pivot
      const csq = x*x + y*y;                    // Square of distance between servo pivot and receiver pivot
      const c = Math.sqrt(csq);                 // Distance between servo pivot and receiver pivot
      const beta = Math.acos((csq + 5625 - bsq)/(150*c)); // Angle between c-line and servo arm
      return msPerRad*(gamma + beta - 3.14159);           // Servo signal output, from neutral
    }

    let roll,pitch,fwd,thrust,side;
    let out1,out2,out3,out4,out5,out6;
    
    roll = mapDecimal(axes['R1'], 0, 0.9999, -3000, 3000);
    pitch = mapDecimal(axes['R2'], 0, 0.9999, -2500, 2500);
    fwd = mapDecimal(axes['L1'], 0, 0.9999, -3000, 3000);
    thrust = mapDecimal(axes['L0'], 0, 0.9999, -6000, 6000);
    side = mapDecimal(axes['L2'], 0, 0.9999, -3000, 3000);

    // Main arms
    out1 = setMainServo(16248 - fwd, 1500 + thrust + roll); // Lower left servo
    out2 = setMainServo(16248 - fwd, 1500 - thrust - roll); // Upper left servo
    out5 = setMainServo(16248 - fwd, 1500 - thrust + roll); // Upper right servo
    out6 = setMainServo(16248 - fwd, 1500 + thrust - roll); // Lower right servo

    // Pitchers
    out3 = setPitchServo(16248 - fwd, 4500 - thrust,  side - 1.5*roll, -pitch);
    out4 = setPitchServo(16248 - fwd, 4500 - thrust, -side + 1.5*roll, -pitch);

    // Set Servos
    const lowerLeftServo = mapDecimal(servoZero - out1, 0, servoInterval, 0, 65535);
    const upperLeftServo = mapDecimal(servoZero + out2, 0, servoInterval, 0, 65535);
    const leftPitchServo = mapDecimal(constrain(servoZero - out3, servoZero - 600, servoZero + 1000), 0, servoInterval, 0, 65535);
    const rightPitchServo = mapDecimal(constrain(servoZero + out4, servoZero - 1000, servoZero + 600), 0, servoInterval, 0, 65535);
    const upperRightServo = mapDecimal(servoZero - out5, 0, servoInterval, 0, 65535);
    const lowerRightServo = mapDecimal(servoZero + out6, 0, servoInterval, 0, 65535);

    return {
      lowerLeftServoAngle: servoToRotation(lowerLeftServo) - Math.PI,
      upperLeftServoAngle: servoToRotation(upperLeftServo),
      lowerRightServoAngle: servoToRotation(lowerRightServo, -1) - Math.PI,
      upperRightServoAngle: servoToRotation(upperRightServo, -1),
      leftPitchServoAngle: servoToRotation(leftPitchServo, -1),
      rightPitchServoAngle: servoToRotation(rightPitchServo),
    }
  }
}
