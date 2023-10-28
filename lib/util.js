import { Material, MathUtils, Mesh, Vector3 } from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { BufferGeometryUtils } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

const objLoader = new OBJLoader();

export const deg = MathUtils.degToRad;
export const rad = MathUtils.radToDeg;

/**
 * Load a model in .OBJ format from a String and apply the specified material.
 * 
 * @param {String} text 
 * @param {Material} material 
 * @Param {Function} [processMesh] - optional function to run on each mesh in the object.
 * @returns 
 */
export function loadObj (text, material, processMesh) {
  const object = objLoader.parse(text);

  forEachMesh(object, (mesh) => {
    if (processMesh instanceof Function) {
      processMesh(mesh);
    }

    mesh.material = material;
  });

  return object;
}

export function loadObjs (count, text, material, processMesh) {
  const objects = [];

  for (let i = 0; i < count; i++) {
    objects.push(loadObj(text, material, processMesh));
  }

  return objects;
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

export function recomputeNormals (mesh) {
  mesh.geometry.deleteAttribute('normal');
  mesh.geometry = BufferGeometryUtils.mergeVertices(mesh.geometry, 0.000001);
  mesh.geometry.computeVertexNormals();
};

/**
 * Calculate the intersection point(s) of a circle and a sphere.
 * 
 * Uses the algorithm described here:
 * https://gamedev.stackexchange.com/questions/75756/sphere-sphere-intersection-and-circle-sphere-intersection
 * 
 * @param {Vector3} circleCenter 
 * @param {Number} circleRadius 
 * @param {Vector3} circlePlaneDirection 
 * @param {Vector3} sphereCenter 
 * @param {Number} sphereRadius 
 */
export function circleSphereIntersection (circleCenter, circleRadius, circlePlaneDirection, sphereCenter, sphereRadius) {
  const circlePlaneNormal = circlePlaneDirection.clone().normalize();

  // The plane of the circle cuts the sphere this distance from the sphere's center.
  const distanceToPlane = circlePlaneNormal.dot(circleCenter.clone().sub(sphereCenter));

  if (Math.abs(distanceToPlane) > sphereRadius) {
    // There is no intersection.
    return null;
  }

  // The center of the circle cut out of the sphere by the plane.
  const sphereCircleCenter = circlePlaneNormal.clone().multiplyScalar(distanceToPlane).add(sphereCenter);

  if (Math.abs(distanceToPlane) === sphereRadius) {
    // This is the sole intersection point with the plane.
    // If it lies on the circle its the intersection point.
    if (sphereCircleCenter.distanceTo(circleCenter) === circleRadius) {
      return [sphereCircleCenter];
    }
    
    // Does not intersect.
    return null;
  }

  const sphereCircleRadius = Math.sqrt(sphereRadius*sphereRadius - distanceToPlane*distanceToPlane);
  const intersectionCircle = circleCircleIntersection(sphereCircleCenter, sphereCircleRadius, circleCenter, circleRadius);

  if (!intersectionCircle) {
    return null;
  }

  const { center, radius } = intersectionCircle;

  if (radius === 0) {
    // Single point of intersection..
    return [center];
  }

  const tangentVector = sphereCircleCenter.clone().sub(circleCenter).cross(circlePlaneNormal).normalize();

  return [
    center.clone().sub(tangentVector.clone().multiplyScalar(radius)),
    center.clone().add(tangentVector.clone().multiplyScalar(radius)),
  ];
}

/**
 * Compute the intersection between two circles. Returns an object with { center, radius }.
 * If there is only one intersection point, the radius will be zero.
 * 
 * https://gamedev.stackexchange.com/questions/75756/sphere-sphere-intersection-and-circle-sphere-intersection
 * 
 * @param {Vector3} c1 
 * @param {Number} r1 
 * @param {Vector3} c2 
 * @param {Number} r2 
 * @returns object with { center, radius }
 */
export function circleCircleIntersection (c1, r1, c2, r2) {
  const difference = c2.clone().sub(c1);
  const distance = Math.abs(difference.length());

  if (distance === 0 || r1 + r2 < distance || distance + Math.min(r1, r2) < Math.max(r1, r2)) {
    // Infinitely many intersections or none.
    return null;
  } else if (r1 + r2 === distance) {
    // Exactly one intersection
    return {
      center: c1.clone().add(difference).multiplyScalar(r1 / distance),
      radius: 0,
    }
  } else if (distance + Math.min(r1, r2) === Math.max(r1, r2)) {
    // Exactly one intersection, but one circle is inside the other.
    const largerCircle = r1 > r2 ? c1.clone() : c2.clone();
    const smallerCircle = r1 < r2 ? c1.clone() : c2.clone();
    const largerRadius = Math.max(r1, r2);

    return {
      center: largerCircle.add(smallerCircle.sub(largerCircle).multiplyScalar(largerRadius / distance)),
      radius: 0,
    }
  }

  const h = 0.5 + (r1*r1 - r2*r2)/(2 * distance*distance);
  const center = c1.clone().add(difference.multiplyScalar(h));
  const radius = Math.sqrt(r1 * r1 - h*h*distance*distance);

  return { center, radius };
}

/**
 * Returns the "direction" of a vector with respect to another using an up vector.
 * 
 * Inspired by https://forum.unity.com/threads/left-right-test-function.31420/
 * 
 * @param {Vector3} a 
 * @param {Vector3} b 
 * @param {Vector3} up 
 * @returns 
 */
export function vectorDirection (a, b, up) {
  const right = up.clone().cross(a);
  const dir = right.dot(b);
 
  return dir > 0 ? 1 : dir < 0 ? -1 : 0;
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

/**
 * Similar to Arduino's method but allows decimals.
 * https://www.arduino.cc/reference/en/language/functions/math/map/
 */
export function mapDecimal (value, inMin, inMax, outMin = 0, outMax = 1) {
  return ((value - inMin) * (outMax - outMin)) / (inMax - inMin) + outMin;
}

export function servoToRotation (servoValue, scale = 1) {
  // Map the servo value to an angle in radians.
  return mapDecimal(servoValue, 0, 65535, deg(-180 * scale), deg(180 * scale));
};
