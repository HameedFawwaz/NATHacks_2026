/* One GPU pass, <=20 fps, <=1024x720. No SVG turbulence or CPU pixel readback. */
(() => {
  "use strict";
  const stage = document.getElementById("map-stage");
  const source = document.getElementById("map");
  const canvas = document.getElementById("map-glitch");
  const reduced = window.matchMedia("(prefers-reduced-motion: reduce)");
  const scratch = document.createElement("canvas");
  const copy = scratch.getContext("2d", { alpha: false });
  let gl, program, texture, frame = null, last = 0, failed = false;
  let resolution, time, strength;

  const fragment = `
    precision highp float;
    uniform sampler2D image;
    uniform vec2 resolution;
    uniform float time;
    uniform float strength;
    varying vec2 uv;
    float hash(float n) { return fract(sin(n * 127.1) * 43758.5453); }
    void main() {
      float beat = floor(time * 2.0);
      float burst = step(.7, hash(beat + 19.0));
      float row = floor(uv.y * resolution.y / 2.0);
      float badLine = step(.96, hash(row + floor(time * 3.0) * 13.0));
      float jitter = (hash(row + beat * 17.0) - .5) * badLine * (8.0 + 16.0 * burst) * strength;
      vec2 p = uv + vec2(jitter / resolution.x, 0.0);
      float split = (2.5 + 5.0 * burst) * strength / resolution.x;
      vec3 color;
      color.r = texture2D(image, p + vec2(split, 0.0)).r;
      color.g = texture2D(image, p - vec2(split * .65, 0.0)).g;
      color.b = texture2D(image, p - vec2(split, 0.0)).b;
      float luminance = dot(color, vec3(.299, .587, .114));
      color = mix(vec3(luminance), color, 1.0 + .22 * strength);
      color = (color - .5) * (1.0 + .08 * strength) + .5;
      float scan = mod(floor(uv.y * resolution.y), 3.0);
      color *= 1.0 - step(2.0, scan) * .07 * strength;
      float grain = hash(dot(floor(uv * resolution), vec2(1.0, 37.0)) + floor(time * 5.0)) - .5;
      color += grain * .035 * strength;
      float edge = smoothstep(.3, .75, length((uv - .5) * vec2(1.15, 1.0)));
      color *= 1.0 - edge * .14 * strength;
      color += vec3(.045, 0.0, .018) * edge * strength;
      gl_FragColor = vec4(clamp(color, 0.0, 1.0), 1.0);
    }
  `;

  function init() {
    gl = canvas.getContext("webgl", { alpha: false, antialias: false, depth: false, stencil: false, powerPreference: "low-power" });
    if (!gl || !copy) throw new Error("GPU effects unavailable");
    function shader(type, source) {
      const shader = gl.createShader(type);
      gl.shaderSource(shader, source);
      gl.compileShader(shader);
      if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) throw new Error("Shader unavailable");
      return shader;
    }
    program = gl.createProgram();
    const vertex = shader(gl.VERTEX_SHADER, "attribute vec2 position; varying vec2 uv; void main(){uv=position*.5+.5;gl_Position=vec4(position,0.,1.);}");
    const pixel = shader(gl.FRAGMENT_SHADER, fragment);
    gl.attachShader(program, vertex);
    gl.attachShader(program, pixel);
    gl.linkProgram(program);
    gl.deleteShader(vertex);
    gl.deleteShader(pixel);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error("GPU effect unavailable");
    gl.useProgram(program);
    const vertices = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vertices);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 1, -1, -1, 1, 1, 1]), gl.STATIC_DRAW);
    const position = gl.getAttribLocation(program, "position");
    gl.enableVertexAttribArray(position);
    gl.vertexAttribPointer(position, 2, gl.FLOAT, false, 0, 0);
    texture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, true);
    gl.uniform1i(gl.getUniformLocation(program, "image"), 0);
    resolution = gl.getUniformLocation(program, "resolution");
    time = gl.getUniformLocation(program, "time");
    strength = gl.getUniformLocation(program, "strength");
  }

  function stop() {
    if (frame !== null) window.cancelAnimationFrame(frame);
    frame = null;
    canvas.hidden = true;
    if (stage.classList.contains("gpu-glitch-active")) stage.classList.remove("gpu-glitch-active");
  }

  function draw(now) {
    if (now - last >= 50) {
      last = now;
      try {
        const box = source.getBoundingClientRect();
        const ratio = Math.min(1, 1024 / box.width, 720 / box.height);
        const width = Math.max(1, Math.round(box.width * ratio));
        const height = Math.max(1, Math.round(box.height * ratio));
        if (canvas.width !== width || canvas.height !== height) {
          canvas.width = scratch.width = width;
          canvas.height = scratch.height = height;
          gl.viewport(0, 0, width, height);
        }
        copy.drawImage(source, 0, 0, width, height);
        gl.bindTexture(gl.TEXTURE_2D, texture);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, scratch);
        gl.uniform2f(resolution, width, height);
        gl.uniform1f(time, now / 1000);
        gl.uniform1f(strength, document.body.classList.contains("is-critical") ? 1.0 : .28);
        gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
        canvas.hidden = false;
        if (!stage.classList.contains("gpu-glitch-active")) stage.classList.add("gpu-glitch-active");
      } catch {
        // Unsupported GPU/CORS/context: retain the real map and cheap CSS warnings.
        failed = true;
        stop();
        return;
      }
    }
    frame = window.requestAnimationFrame(draw);
  }

  function sync() {
    const active = !failed && !reduced.matches && !document.hidden && !stage.classList.contains("fx-disabled") &&
      (document.body.classList.contains("is-critical") || document.body.classList.contains("is-caution"));
    if (!active) { stop(); return; }
    if (frame !== null) return;
    try {
      if (!gl) init();
      last = 0;
      frame = window.requestAnimationFrame(draw);
    } catch { failed = true; stop(); }
  }
  new MutationObserver(sync).observe(document.body, { attributes: true, attributeFilter: ["class"] });
  new MutationObserver(sync).observe(stage, { attributes: true, attributeFilter: ["class"] });
  reduced.addEventListener("change", sync);
  document.addEventListener("visibilitychange", sync);
  window.addEventListener("pagehide", stop);
  window.addEventListener("pageshow", sync);
  canvas.addEventListener("webglcontextlost", () => { failed = true; stop(); });
  sync();
})();
