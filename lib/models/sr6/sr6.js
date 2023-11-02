import { Vector3, Quaternion, MeshPhongMaterial, MeshStandardMaterial } from 'three';
import { 
  loadObj, loadObjs, mapDecimal, servoToRotation, constrain, 
  rad, circleCircleIntersection, circleSphereIntersection, vectorDirection, recomputeNormals 
} from '../../util.js';
import baseGeometry from './geometry/base.js';
import lidGeometry from './geometry/lid.js';
import mainArmServoGeometry from './geometry/main-arm-servos.js';
import armGeometry from './geometry/arm.js';
import leftPitcherGeometry from './geometry/left-pitcher.js';
import rightPitcherGeometry from './geometry/right-pitcher.js';
import receiverGeometry from './geometry/receiver.js';
import bearingArmGeometry from './geometry/bearing-arm.js';
import bearingGeometry from '../common/rod-end-bearing.js'
import receiver from './geometry/receiver.js';

/**
 * 3D Model of the SR6
 */
export default class SR6Model {
  #orientation = -Math.PI / 2; // Rotation around the x-axis.

  load () {
    const ayvaBlueMaterial = new MeshPhongMaterial({ color: 0x3f5173 });
    const darkMaterial = new MeshPhongMaterial({ color: 0x1E1E1E });
    const servoMaterial = new MeshPhongMaterial({ color: 0x131313 });
    const bearingMaterial = new MeshStandardMaterial({
      color: 0xC0C0C0,
      metalness: 1,
      roughness: 0.5, 
    });

    const base = loadObj(baseGeometry, darkMaterial);
    const mainArmServos = loadObj(mainArmServoGeometry, servoMaterial);
    const lid = loadObj(lidGeometry, ayvaBlueMaterial);
    const receiver = loadObj(receiverGeometry, ayvaBlueMaterial);

    const [
      lowerRightArm,
      upperRightArm,
      lowerLeftArm,
      upperLeftArm,
    ] = loadObjs(4, armGeometry, ayvaBlueMaterial);

    const [
      lowerRightLink,
      upperRightLink,
      lowerLeftLink,
      upperLeftLink,
    ] = loadObjs(4, bearingArmGeometry, darkMaterial);

    const [
      upperRightBackBearing,
      upperRightFrontBearing,
      lowerRightBackBearing,
      lowerRightFrontBearing,
      upperLeftBackBearing,
      upperLeftFrontBearing,
      lowerLeftBackBearing,
      lowerLeftFrontBearing,
    ] = loadObjs(8, bearingGeometry, bearingMaterial, recomputeNormals);

    const leftPitcher = loadObj(leftPitcherGeometry, ayvaBlueMaterial);
    const rightPitcher = loadObj(rightPitcherGeometry, ayvaBlueMaterial);

    const armZ = 59.5 - 14 - 3;
    const armX = 59.3;
    upperRightArm.position.fromArray([-armX, 0, armZ]);
    lowerRightArm.position.fromArray([-armX, 30, armZ]);
    lowerRightArm.rotation.x = Math.PI;

    upperLeftArm.position.fromArray([armX, 0, armZ]);
    upperLeftArm.rotation.y = Math.PI;

    lowerLeftArm.position.fromArray([armX, 30, armZ]);
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
      lowerRightLink,
      upperRightLink,
      lowerLeftLink,
      upperLeftLink,
      leftPitcher,
      rightPitcher,
      upperRightBackBearing,
      upperRightFrontBearing,
      lowerRightBackBearing,
      lowerRightFrontBearing,
      upperLeftBackBearing,
      upperLeftFrontBearing,
      lowerLeftBackBearing,
      lowerLeftFrontBearing,
      receiver,
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
      lowerRightLink,
      upperRightLink,
      lowerLeftLink,
      upperLeftLink,
      upperRightBackBearing,
      upperRightFrontBearing,
      lowerRightBackBearing,
      lowerRightFrontBearing,
      upperLeftBackBearing,
      upperLeftFrontBearing,
      lowerLeftBackBearing,
      lowerLeftFrontBearing,
      receiver,
    } = this.objects;

    lowerRightArm.rotation.x = lowerRightServoAngle;
    upperRightArm.rotation.x = upperRightServoAngle;
    lowerLeftArm.rotation.x = lowerLeftServoAngle;
    upperLeftArm.rotation.x = upperLeftServoAngle;
    leftPitcher.rotation.x = leftPitchServoAngle;
    rightPitcher.rotation.x = rightPitchServoAngle;

    upperRightLink.position.copy(this.#computeLinkPosition(upperRightArm.position, upperRightServoAngle, -1));
    lowerRightLink.position.copy(this.#computeLinkPosition(lowerRightArm.position, lowerRightServoAngle, -1));
    upperLeftLink.position.copy(this.#computeLinkPosition(upperLeftArm.position, upperLeftServoAngle));
    lowerLeftLink.position.copy(this.#computeLinkPosition(lowerLeftArm.position, lowerLeftServoAngle));

    upperRightBackBearing.position.copy(upperRightLink.position);
    lowerRightBackBearing.position.copy(lowerRightLink.position);
    upperLeftBackBearing.position.copy(upperLeftLink.position);
    lowerLeftBackBearing.position.copy(lowerLeftLink.position);

    const { 
      intersection: rightIntersectionPoint, 
      center: rightIntersectionCenter
     } =  this.#computeIntersectionPoint(upperRightLink.position, lowerRightLink.position);
     
    const {
      intersection: leftIntersectionPoint,
      center: leftIntersectionCenter
     } = this.#computeIntersectionPoint(upperLeftLink.position, lowerLeftLink.position);

    if (rightIntersectionPoint && leftIntersectionPoint) {
      const receiverWidth = 145.5;
      const actualWidth = rightIntersectionPoint.clone().sub(leftIntersectionPoint).length();

      if (actualWidth > receiverWidth) {
        const offset = (actualWidth - receiverWidth) / 2;

        const rightCopy = rightIntersectionPoint.clone();
        const leftCopy = leftIntersectionPoint.clone();

        rightIntersectionPoint.addScaledVector(leftCopy.clone().sub(rightCopy).normalize(), offset);
        leftIntersectionPoint.addScaledVector(rightCopy.clone().sub(leftCopy).normalize(), offset);
      }

      const upperLinkOffset = 6;
      const upperRightVector = rightIntersectionPoint.clone().sub(leftIntersectionPoint).normalize();
      const upperLeftVector = leftIntersectionPoint.clone().sub(rightIntersectionPoint).normalize();
      const rightIntersectionPointShifted = rightIntersectionPoint.clone().addScaledVector(upperRightVector, upperLinkOffset);
      const leftIntersectionPointShifted = leftIntersectionPoint.clone().addScaledVector(upperLeftVector, upperLinkOffset);

      const upperRightLookAt = this.#toWorldCoordinates(rightIntersectionPointShifted);
      const upperRightLookAtReversed = this.#toWorldCoordinates(
        upperRightBackBearing.position.clone().sub(rightIntersectionPointShifted).add(upperRightBackBearing.position)
      );

      const lowerRightLookAt = this.#toWorldCoordinates(rightIntersectionPoint);
      const lowerRightLookAtReversed = this.#toWorldCoordinates(
        lowerRightBackBearing.position.clone().sub(rightIntersectionPoint).add(lowerRightBackBearing.position)
      );

      const upperLeftLookAt = this.#toWorldCoordinates(leftIntersectionPointShifted);
      const upperLeftLookAtReversed = this.#toWorldCoordinates(
        upperLeftBackBearing.position.clone().sub(leftIntersectionPointShifted).add(upperLeftBackBearing.position)
      );

      const lowerLeftLookAt = this.#toWorldCoordinates(leftIntersectionPoint);
      const lowerLeftLookAtReversed = this.#toWorldCoordinates(
        lowerLeftBackBearing.position.clone().sub(leftIntersectionPoint).add(lowerLeftBackBearing.position)
      );

      const upVector = new Vector3(0, 0, 1);
      upperRightLink.up.copy(upVector)
      upperRightLink.lookAt(upperRightLookAt);
      upperRightBackBearing.up.copy(upVector)
      upperRightBackBearing.lookAt(upperRightLookAtReversed);

      lowerRightLink.up.fromArray([0, 0, 1]);
      lowerRightLink.lookAt(lowerRightLookAt);
      lowerRightBackBearing.up.copy(upVector)
      lowerRightBackBearing.lookAt(lowerRightLookAtReversed);

      upperLeftLink.up.fromArray([0, 0, 1]);
      upperLeftLink.lookAt(upperLeftLookAt);
      upperLeftBackBearing.up.copy(upVector)
      upperLeftBackBearing.lookAt(upperLeftLookAtReversed);

      lowerLeftLink.up.fromArray([0, 0, 1]);
      lowerLeftLink.lookAt(lowerLeftLookAt);
      lowerLeftBackBearing.up.copy(upVector)
      lowerLeftBackBearing.lookAt(lowerLeftLookAtReversed);

      upperRightFrontBearing.position.copy(rightIntersectionPointShifted);
      upperRightFrontBearing.up.copy(upVector);
      upperRightFrontBearing.lookAt(this.#toWorldCoordinates(rightIntersectionPointShifted.clone().sub(upperRightLink.position).add(rightIntersectionPointShifted)));

      lowerRightFrontBearing.position.copy(rightIntersectionPoint);
      lowerRightFrontBearing.up.copy(upVector);
      lowerRightFrontBearing.lookAt(this.#toWorldCoordinates(rightIntersectionPoint.clone().sub(lowerRightLink.position).add(rightIntersectionPoint)));

      upperLeftFrontBearing.position.copy(leftIntersectionPointShifted);
      upperLeftFrontBearing.up.copy(upVector);
      upperLeftFrontBearing.lookAt(this.#toWorldCoordinates(leftIntersectionPointShifted.clone().sub(upperLeftLink.position).add(leftIntersectionPointShifted)));

      lowerLeftFrontBearing.position.copy(leftIntersectionPoint);
      lowerLeftFrontBearing.up.copy(upVector);
      lowerLeftFrontBearing.lookAt(this.#toWorldCoordinates(leftIntersectionPoint.clone().sub(lowerLeftLink.position).add(leftIntersectionPoint)));

      const receiverDirection = new Vector3(-1, 0, 0);
      const receiverBearingAxis = rightIntersectionPoint.clone()
        .sub(leftIntersectionPoint).normalize();
      const rotationAxis = receiverDirection.clone().cross(receiverBearingAxis).normalize();
      const rotationAngle = Math.acos(receiverDirection.dot(receiverBearingAxis));
      const rollQuaternion = new Quaternion().setFromAxisAngle(rotationAxis, rotationAngle);

      receiver.position.copy(leftIntersectionPoint.clone().lerp(rightIntersectionPoint, 0.5));
      receiver.setRotationFromQuaternion(rollQuaternion);
    } else {
      console.warn('No valid intersection found for link arms!');
    }
  }
  
  #computeIntersectionPoint (upperLinkPosition, lowerLinkPosition) {
    const linkArmLength = 169;

    const intersection = circleCircleIntersection(
      upperLinkPosition, linkArmLength, lowerLinkPosition, linkArmLength
    );

    if (!intersection || intersection.radius === 0) {
      return null;
    }

    const tangentVector = upperLinkPosition.clone().sub(lowerLinkPosition).cross(new Vector3(1, 0, 0)).normalize();
    return { 
      intersection: intersection.center.add(tangentVector.clone().multiplyScalar(intersection.radius)),
      center: intersection.center
    }
  }

  #computeLinkPosition (armPosition, angle, scale = 1) {
    const leftAxis = new Vector3(1, 0, 0);
    return new Vector3(0, -50, 0).applyAxisAngle(leftAxis, angle).add(armPosition).addScaledVector(leftAxis, scale * 14.5);
  }

  #toWorldCoordinates (vector) {
    return vector.clone().applyAxisAngle(new Vector3(1, 0, 0), this.#orientation);
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
