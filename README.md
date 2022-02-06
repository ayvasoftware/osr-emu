# osr-emu
OSR Emulator is a simple, web-based graphical emulator for <a href="https://www.patreon.com/tempestvr" target="_blank">Open Source Multi Axis Stroker Robots</a>. TCode can be sent to an instance of the emulator as if it were an actual device, and the movements that would be generated are simulated on a 3D model of an OSR.

The following axes are currently supported: L0 (stroke/up), R0 (twist), R1 (roll), R2 (pitch)

## Quick Start

In a browser, the OSR Emulator can be imported as an <a href="https://developer.mozilla.org/en-US/docs/Web/JavaScript/Guide/Modules" target="_blank">ES6 module</a> using a CDN such as unpkg, and initialized into a target html element:

```html
<!DOCTYPE html>
<body>
  <div id="canvas"></div>

  <script type="module">
    import OSREmulator from 'https://unpkg.com/osr-emu';

    const osr = new OSREmulator('#canvas');

    // Write t-code to device to simulate movement.
    osr.write('L0000I500\n'); 
  </script>
</body>
```

Live, interval, and speed based commands are supported on all available axes.

See the following editable [codepen example](https://ayvajs.github.io/osr-emu/example.html).
