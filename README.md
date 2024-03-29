# osr-emu
A simple, web-based graphical emulator for <a href="https://www.patreon.com/tempestvr" target="_blank">Open Source Multi Axis Stroker Robots</a>. TCode can be sent to an instance of the emulator as if it were an actual device, and the movements that would be generated are simulated on a 3D model of an OSR. The OSR2+, SR6, and SSR1 are supported.

The following axes are available on the OSR2+:  

**L0** (stroke/up)  
**R0** (twist)  
**R1** (roll)  
**R2** (pitch)

The SR6 model allows for an additional two axes:

**L1** (forward/back)  
**L2** (left/right)

The SSR1 model only has an **L0** axis.

## Quick Start

In a browser, the OSR Emulator can be imported as an <a href="https://developer.mozilla.org/en-US/docs/Web/JavaScript/Guide/Modules" target="_blank">ES6 module</a> using a CDN such as unpkg, and initialized into a target html element:

### OSR2 (default):
```html
<!DOCTYPE html>
<body>
  <div id="canvas"></div>

  <script type="module">
    import { OSREmulator } from 'https://unpkg.com/osr-emu';

    const osr = new OSREmulator('#canvas');

    // Write t-code to device to simulate movement.
    osr.write('L0000I500\n'); 
  </script>
</body>
```

### SR6:
```html
<!DOCTYPE html>
<body>
  <div id="canvas"></div>

  <script type="module">
    import { OSREmulator } from 'https://unpkg.com/osr-emu';

    const osr = new OSREmulator('#canvas', { 
      model: 'SR6' 
    });

    // Write t-code to device to simulate movement.
    osr.write('L0000I500 L1000I500\n'); 
  </script>
</body>
```

### SSR1:
```html
<!DOCTYPE html>
<body>
  <div id="canvas"></div>

  <script type="module">
    import { OSREmulator } from 'https://unpkg.com/osr-emu';

    const osr = new OSREmulator('#canvas', { 
      model: 'SSR1' 
    });

    // Write t-code to device to simulate movement.
    osr.write('L0000I500\n'); 
  </script>
</body>
```

### NPM
The OSR Emulator is also available as an NPM package.

```
npm install osr-emu
```
...
```javascript
import { OSREmulator } from 'osr-emu';
```
### Resizing

The emulator automatically sizes itself to the target element, and will resize itself when the target element's size changes.

### Cleanup

When you are finished with the emulator, you must destroy the instance in order to prevent memory leaks.

```
osr.destroy();
```

This disposes of the 3D renderer and cancels resize event listeners.

### Examples

Live, interval, and speed based commands are supported on all available axes.

See the following editable [codepen example](https://ayvasoftware.github.io/osr-emu/example.html).
