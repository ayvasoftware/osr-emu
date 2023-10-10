import { map, constrain } from './util.js';

const MIN_SMOOTH_INTERVAL = 3;     // Minimum auto-smooth ramp interval for live commands (ms)
const MAX_SMOOTH_INTERVAL = 100;   // Maximum auto-smooth ramp interval for live commands (ms)

/**
 * A class that emulates the behavior of an axis as coded on the OSR2 Firmware (v3.3)
 */
export default class Axis {
  name = '';

  #startTime = 0;
  #startPosition = 5000;
  #targetTime = 0;
  #targetPosition = 5000;
  #minInterval = MAX_SMOOTH_INTERVAL;

  constructor (name) {
    this.name = name;
  }

  set (magnitude, ext, extMagnitude) {
    const currentTime = performance.now();
    magnitude = constrain(magnitude, 0, 9999);
    extMagnitude = constrain(extMagnitude, 0, 9999999);

    if (!extMagnitude || ( ext != 'S' && ext != 'I' ) ) {
      // Live command
      // Update auto-smooth regulator
      let lastInterval = currentTime - this.#startTime;
      if (lastInterval > this.#minInterval && this.#minInterval < MAX_SMOOTH_INTERVAL) { 
        this.#minInterval += 1; 
      } else if (lastInterval < this.#minInterval && this.#minInterval > MIN_SMOOTH_INTERVAL) {
        this.#minInterval -= 1;
      } 

      this.#startPosition = this.getPosition();
      this.#targetTime = currentTime + this.#minInterval;  
    } else if ( ext == 'S' ) {
      // Speed command
      const speed = extMagnitude / 100; // Interpret extMagntitude as units per 100 ms.
      this.#startPosition = this.getPosition();

      let distance = Math.abs(magnitude - this.#startPosition);
      let duration = Math.floor(distance / speed);
      this.#targetTime = currentTime + duration;
    } else if ( ext == 'I' ) {
      // Interval command
      const duration = extMagnitude; // Interpret extMagnitude as the duration of the move in ms.
      this.#startPosition = this.getPosition();
      this.#targetTime = currentTime + duration;
    }

    this.#startTime = currentTime;
    this.#targetPosition = magnitude;
  }

  getPosition () {
    let position; // 0 - 9999
    const currentTime = performance.now();

    if (currentTime > this.#targetTime) {
      position = this.#targetPosition;
    } else if (currentTime > this.#startTime) { 
      position = map(currentTime, this.#startTime, this.#targetTime, this.#startPosition, this.#targetPosition);
    } else {
      position = this.#startPosition;
    }
    
    return constrain(position, 0, 9999);
  }
}
