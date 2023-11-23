import { Vector3, Quaternion, MeshPhongMaterial, MeshStandardMaterial } from 'three';
import { 
  loadObj, loadObjs, loadComplexObj, mapDecimal, servoToRotation, constrain, degToRad,
  circleCircleIntersection, circleSphereIntersection, vectorDirection, recomputeNormals,
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
import leftPitcherLinkGeometry from './geometry/pitcher-link-left.js';
import rightPitcherLinkGeometry from './geometry/pitcher-link-right.js';
import caseGeometry from '../common/case.js';

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
    const modCase = loadObj(caseGeometry, darkMaterial);

    const [
      upperLeftArm,
      upperRightArm,
      lowerLeftArm,
      lowerRightArm,
    ] = loadObjs(4, armGeometry, ayvaBlueMaterial);

    const [
      upperLeftLink,
      upperRightLink,
      lowerLeftLink,
      lowerRightLink,
    ] = loadObjs(4, bearingArmGeometry, darkMaterial);

    const [
      // TODO: Include these as part of the link model with different material (the same way the pitcher links are loaded).
      upperLeftFrontBearing,
      upperLeftBackBearing,
      upperRightFrontBearing,
      upperRightBackBearing,
      lowerRightFrontBearing,
      lowerRightBackBearing,
      lowerLeftFrontBearing,
      lowerLeftBackBearing,
    ] = loadObjs(8, bearingGeometry, bearingMaterial, recomputeNormals);

    const pitcherLinkModelConfig = {
      rodEndBearing: {
        processMesh: recomputeNormals,
        material: bearingMaterial,
      },
      pitcherLink: {
        material: darkMaterial,
      }
    };

    const leftPitcher = loadObj(leftPitcherGeometry, ayvaBlueMaterial);
    const leftPitcherLink = loadComplexObj(leftPitcherLinkGeometry, pitcherLinkModelConfig);
    const rightPitcher = loadObj(rightPitcherGeometry, ayvaBlueMaterial);
    const rightPitcherLink = loadComplexObj(rightPitcherLinkGeometry, pitcherLinkModelConfig);

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
      leftPitcherLink,
      rightPitcherLink,
      receiver,
      modCase,
    };

    this.#setBaseArmPositions();

    return { objects: this.objects, orientation: this.#orientation };
  }

  preRender (axes, scale) {
    this.#setBaseArmPositions();
    this.#performKinematics(axes, scale);
  }

  /**
   * Set the positions of all the arms (where they attach to the base model).
   */
  #setBaseArmPositions () {
    const armX = 58.5;
    const upperArmY = 0;
    const lowerArmY = 30;
    const armZ = 59.92 - 10; // Servo axle is about 10mm from the edge of the servo.
    
    const pitcherArmX = 14.318;
    const pitcherArmYZ = [-29.72, 49.325]

    const { 
      leftPitcher, rightPitcher,
      upperLeftArm, upperRightArm, 
      lowerLeftArm, lowerRightArm,  
    } = this.objects;

    upperLeftArm.position.fromArray([armX, upperArmY, armZ]);
    upperLeftArm.rotation.y = Math.PI;

    upperRightArm.position.fromArray([-armX, upperArmY, armZ]);

    lowerLeftArm.position.fromArray([armX, lowerArmY, armZ]);
    lowerLeftArm.rotation.x = Math.PI;
    lowerLeftArm.rotation.y = Math.PI;

    lowerRightArm.position.fromArray([-armX, lowerArmY, armZ]);
    lowerRightArm.rotation.x = Math.PI;

    leftPitcher.position.fromArray([pitcherArmX, ...pitcherArmYZ]);
    rightPitcher.position.fromArray([-pitcherArmX, ...pitcherArmYZ]);
  }

  /**
   * This method uses a combination of Forward Kinematics and Inverse Kinematics to position everything.
   * 
   * The main arms' position and rotation are calculated via Forward Kinematics. The servo angles are
   * generated using code ripped directly from the actual SR6 firmware. The position of the receiver is then 
   * calculated from the main arm positions.
   * 
   * From there, we do a direct rotation of the expected pitch on the receiver, and then do Inverse Kinematics
   * to calculate the rotation of the pitcher servos and position of the pitcher angle links.
   *  
   * @param {Object} axes - map of current values for each axis
   * @param {Object} scale - map of scale for each axis
   */
  #performKinematics (axes, scale) {
    const pitchRange = 50; // mm
    const upVector = new Vector3(0, 0, 1);
    const pitchAngle = scale['R2'] * axes['R2'] * degToRad(pitchRange) - degToRad(pitchRange/2);
    const pitchBearingAngle = pitchAngle - degToRad(14.763202965644615); //  Angle to the pitch holes on the receiver (from main arm holes)
    const receiverDirection = new Vector3(-1, 0, 0);
    const pitcherBearingVector = new Vector3(0, -75, 0);
    const angleLinkLength = 185; // mm
    const receiverWidth = 145.5; // mm
    const upperLinkOffset = 6; // mm
    const swayRange = 30; // mm
    const pitchQuaternion = new Quaternion()
      .setFromAxisAngle(new Vector3(1, 0, 0), pitchAngle);
    const pitchBearingQuaternion = new Quaternion()
      .setFromAxisAngle(new Vector3(1, 0, 0), pitchBearingAngle);
    const twistQuaternion = new Quaternion().
      setFromAxisAngle(new Vector3(0, -1, 0), scale['R0'] * axes['R0'] * degToRad(240) - degToRad(120));

    const { 
      lowerLeftServoAngle,
      upperLeftServoAngle,
      lowerRightServoAngle,
      upperRightServoAngle,
      leftPitchServoAngle,
      rightPitchServoAngle,
    } = this.#computeFirmwareServoAngles(axes, scale);

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
      leftPitcherLink,
      rightPitcherLink,
      receiver,
      modCase,
    } = this.objects;

    leftPitcher.rotation.x = leftPitchServoAngle;
    rightPitcher.rotation.x = rightPitchServoAngle;
    upperLeftArm.rotation.x = upperLeftServoAngle;
    upperRightArm.rotation.x = upperRightServoAngle;
    lowerLeftArm.rotation.x = lowerLeftServoAngle;
    lowerRightArm.rotation.x = lowerRightServoAngle;

    upperLeftLink.position.copy(this.#computeLinkPosition(upperLeftArm.position, upperLeftServoAngle));
    upperRightLink.position.copy(this.#computeLinkPosition(upperRightArm.position, upperRightServoAngle, -1));
    lowerLeftLink.position.copy(this.#computeLinkPosition(lowerLeftArm.position, lowerLeftServoAngle));
    lowerRightLink.position.copy(this.#computeLinkPosition(lowerRightArm.position, lowerRightServoAngle, -1));
    
    upperLeftBackBearing.position.copy(upperLeftLink.position);
    upperRightBackBearing.position.copy(upperRightLink.position);
    lowerLeftBackBearing.position.copy(lowerLeftLink.position);
    lowerRightBackBearing.position.copy(lowerRightLink.position);

    const {
      intersection: leftIntersectionPoint,
      center: leftIntersectionCenter,
      radius: leftIntersectionRadius,
     } = this.#computeMainLinkIntersectionPoint(upperLeftLink.position, lowerLeftLink.position);

    const { 
      intersection: rightIntersectionPoint, 
      center: rightIntersectionCenter,
      radius: rightIntersectionRadius,
     } =  this.#computeMainLinkIntersectionPoint(upperRightLink.position, lowerRightLink.position);

    if (leftIntersectionPoint && rightIntersectionPoint) {
      const actualWidth = rightIntersectionPoint.clone().sub(leftIntersectionPoint).length();

      if (actualWidth > receiverWidth) {
        // Keep the main arms attached to the receiver by applying an offset
        // when the distance between the attachment points is too large.
        const offset = (actualWidth - receiverWidth) / 2;

        const rightCopy = rightIntersectionPoint.clone();
        const leftCopy = leftIntersectionPoint.clone();

        rightIntersectionPoint.addScaledVector(leftCopy.clone().sub(rightCopy).normalize(), offset);
        leftIntersectionPoint.addScaledVector(rightCopy.clone().sub(leftCopy).normalize(), offset);
      }

      const swayArc = scale['L2'] * swayRange * ((axes['L2'] - 0.5) / 0.5);
      const rightArcAngle = swayArc / rightIntersectionRadius;
      const leftArcAngle = swayArc / leftIntersectionRadius;
      
      if (rightArcAngle || leftArcAngle) {
        // To apply sway, we rotate the main arms around the Y axis (relative to their center points)
        const rotationUpVector = new Vector3(0, 1, 0);
        rightIntersectionPoint.sub(rightIntersectionCenter).applyAxisAngle(rotationUpVector, rightArcAngle).add(rightIntersectionCenter);
        leftIntersectionPoint.sub(leftIntersectionCenter).applyAxisAngle(rotationUpVector, leftArcAngle).add(leftIntersectionCenter);
      }

      // The upper links of the main arms are attached on the outside, so we shift them over by an offset.
      const upperLeftVector = leftIntersectionPoint.clone().sub(rightIntersectionPoint).normalize();
      const upperRightVector = rightIntersectionPoint.clone().sub(leftIntersectionPoint).normalize();
      const leftIntersectionPointShifted = leftIntersectionPoint.clone().addScaledVector(upperLeftVector, upperLinkOffset);
      const rightIntersectionPointShifted = rightIntersectionPoint.clone().addScaledVector(upperRightVector, upperLinkOffset);

      const upperLeftLookAt = this.#toWorldCoordinates(leftIntersectionPointShifted);
      const upperLeftLookAtReversed = this.#toWorldCoordinates(
        upperLeftBackBearing.position.clone().sub(leftIntersectionPointShifted).add(upperLeftBackBearing.position)
      );

      const upperRightLookAt = this.#toWorldCoordinates(rightIntersectionPointShifted);
      const upperRightLookAtReversed = this.#toWorldCoordinates(
        upperRightBackBearing.position.clone().sub(rightIntersectionPointShifted).add(upperRightBackBearing.position)
      );

      const lowerLeftLookAt = this.#toWorldCoordinates(leftIntersectionPoint);
      const lowerLeftLookAtReversed = this.#toWorldCoordinates(
        lowerLeftBackBearing.position.clone().sub(leftIntersectionPoint).add(lowerLeftBackBearing.position)
      );

      const lowerRightLookAt = this.#toWorldCoordinates(rightIntersectionPoint);
      const lowerRightLookAtReversed = this.#toWorldCoordinates(
        lowerRightBackBearing.position.clone().sub(rightIntersectionPoint).add(lowerRightBackBearing.position)
      );

      // Position and aim the main link bearings in the right direction.
      // TODO: Some of this can go away once we merge rod end bearings into the link model.
      upperLeftLink.up.fromArray([0, 0, 1]);
      upperLeftLink.lookAt(upperLeftLookAt);
      upperLeftBackBearing.up.copy(upVector)
      upperLeftBackBearing.lookAt(upperLeftLookAtReversed);

      upperRightLink.up.copy(upVector)
      upperRightLink.lookAt(upperRightLookAt);
      upperRightBackBearing.up.copy(upVector)
      upperRightBackBearing.lookAt(upperRightLookAtReversed);

      lowerLeftLink.up.fromArray([0, 0, 1]);
      lowerLeftLink.lookAt(lowerLeftLookAt);
      lowerLeftBackBearing.up.copy(upVector)
      lowerLeftBackBearing.lookAt(lowerLeftLookAtReversed);

      lowerRightLink.up.fromArray([0, 0, 1]);
      lowerRightLink.lookAt(lowerRightLookAt);
      lowerRightBackBearing.up.copy(upVector)
      lowerRightBackBearing.lookAt(lowerRightLookAtReversed);

      upperLeftFrontBearing.position.copy(leftIntersectionPointShifted);
      upperLeftFrontBearing.up.copy(upVector);
      upperLeftFrontBearing.lookAt(this.#toWorldCoordinates(leftIntersectionPointShifted.clone().sub(upperLeftLink.position).add(leftIntersectionPointShifted)));

      upperRightFrontBearing.position.copy(rightIntersectionPointShifted);
      upperRightFrontBearing.up.copy(upVector);
      upperRightFrontBearing.lookAt(this.#toWorldCoordinates(rightIntersectionPointShifted.clone().sub(upperRightLink.position).add(rightIntersectionPointShifted)));

      lowerLeftFrontBearing.position.copy(leftIntersectionPoint);
      lowerLeftFrontBearing.up.copy(upVector);
      lowerLeftFrontBearing.lookAt(this.#toWorldCoordinates(leftIntersectionPoint.clone().sub(lowerLeftLink.position).add(leftIntersectionPoint)));

      lowerRightFrontBearing.position.copy(rightIntersectionPoint);
      lowerRightFrontBearing.up.copy(upVector);
      lowerRightFrontBearing.lookAt(this.#toWorldCoordinates(rightIntersectionPoint.clone().sub(lowerRightLink.position).add(rightIntersectionPoint)));
      
      const receiverMainBearingAxis = rightIntersectionPoint.clone()
        .sub(leftIntersectionPoint).normalize();

      const rotationAxis = receiverDirection.clone().cross(receiverMainBearingAxis).normalize();
      const rotationAngle = Math.acos(receiverDirection.dot(receiverMainBearingAxis));
      const rollQuaternion = new Quaternion().setFromAxisAngle(rotationAxis, rotationAngle);

      const leftPitcherEndBearingPosition = new Vector3(0, -55, 0)
        .applyQuaternion(
          rollQuaternion.clone().multiply(pitchBearingQuaternion)
        ).add(leftIntersectionPoint);

      // TODO: Split out inverse kinematics into reusable function for both left and right.
      const leftPitcherServoIntersectionPoints = circleSphereIntersection(
        leftPitcher.position, 
        75, // Distance between pitcher hole and arm hole on the receiver...
        new Vector3(-1, 0, 0), 
        leftPitcherEndBearingPosition, 
        angleLinkLength, // Length of pitcher link arm to the hole...
      );

      if (leftPitcherServoIntersectionPoints) {
        const position = this.#intersectionPointsToPosition(leftPitcherServoIntersectionPoints);

        leftPitcherLink.position.copy(position);

        // Correct left servo angle using inverse kinematics.
        const correctLeftPitcherVector = leftPitcherLink.position.clone().sub(leftPitcher.position);
        const currentLeftPitcherVector = pitcherBearingVector.clone().applyAxisAngle(receiverDirection, -leftPitchServoAngle);
        const correction = correctLeftPitcherVector.angleTo(currentLeftPitcherVector);

        const direction = vectorDirection(correctLeftPitcherVector, currentLeftPitcherVector, new Vector3(1, 0, 0));
        leftPitcher.rotation.x -= (correction * direction);

        leftPitcherLink.up.fromArray([0, 0, 1]);
        leftPitcherLink.lookAt(this.#toWorldCoordinates(leftPitcherEndBearingPosition));

        const rightPitcherEndBearingPosition = leftPitcherEndBearingPosition.clone()
          .add(rightIntersectionPoint.clone()
          .sub(leftIntersectionPoint));

          const rightPitcherServoIntersectionPoints = circleSphereIntersection(
            rightPitcher.position, 
            75, // Distance between pitcher hole and arm hole on the receiver...
            new Vector3(-1, 0, 0), 
            rightPitcherEndBearingPosition, 
            angleLinkLength, // Length of pitcher link arm to the hole...
          );

          if (rightPitcherServoIntersectionPoints) {
            const position = this.#intersectionPointsToPosition(rightPitcherServoIntersectionPoints);

            rightPitcherLink.position.copy(position);

            // Correct right servo angle.
            const correctRightPitcherVector = rightPitcherLink.position.clone().sub(rightPitcher.position);
            const currentRightPitcherVector = pitcherBearingVector.clone().applyAxisAngle(receiverDirection, -rightPitchServoAngle);

            const correction = correctRightPitcherVector.angleTo(currentRightPitcherVector);

            const direction = vectorDirection(correctRightPitcherVector, currentRightPitcherVector, new Vector3(1, 0, 0));
            rightPitcher.rotation.x -= (correction * direction);
          }

        rightPitcherLink.up.fromArray([0, 0, 1]);
        rightPitcherLink.lookAt(this.#toWorldCoordinates(rightPitcherEndBearingPosition));

        this._lastLeftPitcherEndBearingPosition = leftPitcherEndBearingPosition;
      } else {
        console.warn('Invalid model arrangement. No intersection found for positioning pitcher link!');
      }

      const receiverQuaternion = rollQuaternion.multiply(pitchQuaternion);

      receiver.position.copy(leftIntersectionPoint.clone().lerp(rightIntersectionPoint, 0.5));
      receiver.setRotationFromQuaternion(receiverQuaternion);

      modCase.position.copy(receiver.position);
      modCase.setRotationFromQuaternion(receiverQuaternion.clone().multiply(twistQuaternion));
    } else {
      console.warn('No valid intersection found for link arms!');
    }
  }
  
  #computeMainLinkIntersectionPoint (upperLinkPosition, lowerLinkPosition) {
    const linkArmLength = 175; // mm

    const intersection = circleCircleIntersection(
      upperLinkPosition, linkArmLength, lowerLinkPosition, linkArmLength
    );

    if (!intersection || intersection.radius === 0) {
      return null;
    }

    const tangentVector = upperLinkPosition.clone().sub(lowerLinkPosition).cross(new Vector3(1, 0, 0)).normalize();

    return { 
      intersection: intersection.center.clone().add(tangentVector.clone().multiplyScalar(intersection.radius)),
      center: intersection.center,
      radius: intersection.radius,
    };
  }

  #computeLinkPosition (armPosition, angle, scale = 1) {
    const leftAxis = new Vector3(1, 0, 0);
    return new Vector3(0, -50, 0).applyAxisAngle(leftAxis, angle).add(armPosition).addScaledVector(leftAxis, scale * 14.5);
  }

  #toWorldCoordinates (vector) {
    return vector.clone().applyAxisAngle(new Vector3(1, 0, 0), this.#orientation);
  }

  #intersectionPointsToPosition (points) {
    return points.length === 1 ? points[0] :
      points[0].y < points[1].y ? points[0] :
      points[1];
  }

  /**
   * Compute the angles for the servos based on the algorithm from the actual SR6 Firmware.
   */
  #computeFirmwareServoAngles (axes) {
    const pitchServoZero = 1580;
    const rightPitchServoZero = (1515.105 - pitchServoZero) + 1515.105;
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

      let betaCos = (csq - 28125)/(100*c)
      betaCos = betaCos < -1 ? -1 : betaCos > 1 ? 1 : betaCos;
      const beta = Math.acos(betaCos);  // Angle between c-line and servo arm
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

      let betaCos = (csq + 5625 - bsq)/(150*c);
      betaCos = betaCos < -1 ? -1 : betaCos > 1 ? 1 : betaCos;

      const beta = Math.acos(betaCos); // Angle between c-line and servo arm

      return msPerRad*(gamma + beta - 3.14159);           // Servo signal output, from neutral
    }

    let roll,pitch,fwd,thrust,side;
    let out1,out2,out3,out4,out5,out6;
    
    roll = mapDecimal(axes['R1'], 0, 0.9999, -3000, 3000);
    pitch = mapDecimal(axes['R2'], 0, 0.9999, -2500, 2500);
    fwd = mapDecimal(axes['L1'], 0, 0.9999, -3000, 3000); // 60 mm
    thrust = mapDecimal(axes['L0'], 0, 0.9999, -6000, 6000); // 120 mm stroke length
    side = mapDecimal(axes['L2'], 0, 0.9999, -3000, 3000); // 60 mm

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
    const leftPitchServo = mapDecimal(constrain(pitchServoZero - out3, pitchServoZero - 600, pitchServoZero + 1000), 0, servoInterval, 0, 65535);
    const rightPitchServo = mapDecimal(constrain(rightPitchServoZero + out4, rightPitchServoZero - 1000, rightPitchServoZero + 600), 0, servoInterval, 0, 65535);
    const upperRightServo = mapDecimal(servoZero - out5, 0, servoInterval, 0, 65535);
    const lowerRightServo = mapDecimal(servoZero + out6, 0, servoInterval, 0, 65535);

   const scale = 0.5; // 180 degrees rotation

    return {
      lowerLeftServoAngle: servoToRotation(lowerLeftServo, scale) - Math.PI,
      upperLeftServoAngle: servoToRotation(upperLeftServo, scale),
      lowerRightServoAngle: servoToRotation(lowerRightServo, -scale) - Math.PI,
      upperRightServoAngle: servoToRotation(upperRightServo, -scale),
      leftPitchServoAngle: servoToRotation(leftPitchServo, -scale),
      rightPitchServoAngle: servoToRotation(rightPitchServo, scale),
    }
  }
}
