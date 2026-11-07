// Optional presentation tests: node --test web/mission.test.cjs (no packages needed).
const { test } = require("node:test");
const assert = require("node:assert/strict");
const { readFileSync } = require("node:fs");
const vm = require("node:vm");
const path = require("node:path");

const source = name => readFileSync(path.join(__dirname, name), "utf8");

function mapHarness() {
  const images = [], draws = [];
  const status = { textContent: "" };
  const buttons = Object.fromEntries(["map-satellite", "map-street"].map(id => [id, {
    setAttribute() {}, addEventListener(name, fn) { this[name] = fn; }
  }]));
  const window = {};
  let now = 1000;
  vm.runInNewContext(source("map-tiles.js"), {
    window, document: { getElementById: id => id === "map-status" ? status : buttons[id] || null }, Math,
    Date: { now: () => now },
    Image: class { constructor() { images.push(this); } }
  });
  const ctx = { save() {}, restore() {}, drawImage(...args) { draws.push(args); } };
  const draw = () => window.TrustIssuesMap.draw(ctx, 800, 600, -35.363262, 149.165237, 0, 0, 1);
  return { images, draws, status, ctx, draw, buttons, map: window.TrustIssuesMap, advance: () => { now += 61000; } };
}

test("basemap requests only visible tiles and reuses them across frames", () => {
  const h = mapHarness();
  h.draw();
  const count = h.images.length;
  assert.ok(count > 0 && count < 40);
  assert.ok(h.images.every(i => i.src.startsWith("https://server.arcgisonline.com/")));
  for (let i = 0; i < 10; i++) h.draw();
  assert.equal(h.images.length, count);
  h.images.forEach(i => i.onload());
  h.draw();
  assert.equal(h.draws.length, count);
  assert.match(h.status.textContent, /Satellite map · drag/);
  // The tile containing home must cover the same screen centre as model.ToLocal.
  assert.ok(h.draws.some(([, x, y, w, height]) => x <= 400 && x + w >= 400 && y <= 300 && y + height >= 300));
  assert.ok(h.draws.every(([, , , w, height]) => w > 0 && height > 0));
});

test("switching basemaps uses distinct cached tiles and updates the style", () => {
  const h = mapHarness();
  h.draw();
  const count = h.images.length;
  h.buttons["map-street"].click();
  h.draw();
  assert.equal(h.images.length, count * 2);
  assert.ok(h.images.slice(count).every(i => i.src.startsWith("https://tile.openstreetmap.org/")));
  h.buttons["map-satellite"].click();
  h.draw();
  assert.equal(h.images.length, count * 2, "previous basemap should use cached tiles");
});

test("offline tiles leave the grid intact and failures are not retried every frame", () => {
  const h = mapHarness();
  h.draw();
  const count = h.images.length;
  h.images.forEach(i => i.onerror());
  h.draw();
  assert.match(h.status.textContent, /Offline grid/);
  assert.equal(h.draws.length, 0);
  assert.equal(h.images.length, count);
  h.advance();
  h.draw();
  assert.equal(h.images.length, count * 2);
});

test("basemap handles polar limits and wraps request longitudes", () => {
  const h = mapHarness();
  h.buttons["map-street"].click();
  h.map.draw(h.ctx, 800, 600, 89, 0, 0, 0, 1);
  assert.equal(h.images.length, 0);
  assert.match(h.status.textContent, /latitude/);
  h.map.draw(h.ctx, 800, 600, 0, 180, 0, 0, .1);
  for (const image of h.images) {
    const [, z, x, y] = image.src.match(/\/(\d+)\/(\d+)\/(\d+)\.png$/);
    assert.ok(+x >= 0 && +x < 2 ** +z && +y >= 0 && +y < 2 ** +z);
  }
});

function alertHarness({ available = true, blocked = false, missingAsset = false } = {}) {
  const listeners = new Map(), sources = [];
  let observer;
  function element() {
    const classes = new Set();
    return {
      textContent: "", hidden: true, attrs: {},
      classList: {
        contains: name => classes.has(name),
        toggle(name, value) { if (value) classes.add(name); else classes.delete(name); }
      },
      setAttribute(name, value) { this.attrs[name] = value; },
      addEventListener(name, fn) { this[name] = fn; }
    };
  }
  const elements = Object.fromEntries(["summary", "summary-title", "critical-banner", "critical-detail", "alarm-toggle"].map(id => [id, element()]));
  const body = element();
  const document = {
    body, hidden: false, getElementById: id => elements[id],
    addEventListener: (name, fn) => listeners.set(name, fn)
  };
  const param = () => ({ setValueAtTime() {}, linearRampToValueAtTime() {} });
  class AudioContext {
    state = blocked ? "suspended" : "running";
    currentTime = 0;
    destination = {};
    addEventListener() {}
    async resume() { if (blocked) throw Error("Autoplay blocked"); this.state = "running"; }
    async decodeAudioData() { return { duration: 21 }; }
    createBufferSource() {
      const buffer = { connect() {}, disconnect() {}, start() { this.started = true; }, stop() { this.stopped = true; } };
      sources.push(buffer);
      return buffer;
    }
    createGain() { return { gain: param(), connect() {}, disconnect() {} }; }
  }
  const window = {
    AudioContext: available ? AudioContext : undefined,
    addEventListener() {}
  };
  vm.runInNewContext(source("mission.js"), {
    window, document,
    fetch: async url => {
      assert.equal(url, "audio/critical-siren.mp3");
      return { ok: !missingAsset, arrayBuffer: async () => new ArrayBuffer(1) };
    },
    MutationObserver: class { constructor(fn) { observer = fn; } observe() {} }
  });
  return {
    body, document, elements, sources,
    playing: () => sources.filter(s => s.started && !s.stopped).length,
    gesture: async () => { listeners.get("pointerdown")({ target: { closest: () => false } }); await new Promise(setImmediate); },
    toggle: async () => { elements["alarm-toggle"].click(); await new Promise(setImmediate); },
    visibility: hidden => { document.hidden = hidden; listeners.get("visibilitychange")(); },
    assess(critical, message = "Critical: GNSS") {
      elements.summary.classList.toggle("level-critical", critical);
      elements["summary-title"].textContent = message;
      observer();
    }
  };
}

test("critical visuals do not require audio permission; recorded siren waits for a gesture", async () => {
  const h = alertHarness();
  h.assess(true);
  assert.equal(h.body.classList.contains("is-critical"), true);
  assert.equal(h.elements["critical-banner"].hidden, false);
  assert.equal(h.elements["critical-detail"].textContent, "Critical: GNSS");
  assert.equal(h.sources.length, 0);
  await h.gesture();
  assert.equal(h.playing(), 1);
  assert.equal(h.sources[0].loop, true);
  h.assess(true, "Critical: Link");
  assert.equal(h.elements["critical-detail"].textContent, "Critical: Link");
  assert.equal(h.sources.length, 1, "telemetry updates must not restart the alarm");
});

test("mute stops the recorded siren without hiding visual alerts", async () => {
  const h = alertHarness();
  await h.gesture();
  h.assess(true);
  await h.toggle();
  assert.equal(h.playing(), 0);
  assert.equal(h.elements["alarm-toggle"].textContent, "Alarm muted");
  assert.ok(h.sources.every(s => s.stopped));
  assert.equal(h.elements["critical-banner"].hidden, false);
  await h.gesture();
  assert.equal(h.playing(), 0, "an unrelated click must not unmute");
  await h.toggle();
  assert.equal(h.playing(), 1);
});

test("recovery, reset or no-data summary clears critical presentation and sound", async () => {
  const h = alertHarness();
  await h.gesture();
  h.assess(true);
  h.assess(false, "No contact with the drone");
  assert.equal(h.playing(), 0);
  assert.equal(h.elements["critical-banner"].hidden, true);
  assert.equal(h.body.classList.contains("is-critical"), false);
});

test("hidden tabs silence alarms; visible critical tabs resume", async () => {
  const h = alertHarness();
  await h.gesture();
  h.assess(true);
  h.visibility(true);
  assert.equal(h.playing(), 0);
  h.visibility(false);
  assert.equal(h.playing(), 1);
});

test("missing audio, missing asset and rejected autoplay never disable critical visuals", async () => {
  for (const options of [{ available: false }, { blocked: true }, { missingAsset: true }]) {
    const h = alertHarness(options);
    await h.gesture();
    h.assess(true);
    assert.equal(h.elements["critical-banner"].hidden, false);
    assert.equal(h.playing(), 0);
  }
});

test("caution enables mild visual effects without starting the critical siren", async () => {
  const h = alertHarness();
  await h.gesture();
  h.elements.summary.classList.toggle("level-caution", true);
  h.assess(false, "Caution: GNSS");
  assert.equal(h.body.classList.contains("is-caution"), true);
  assert.equal(h.body.classList.contains("is-critical"), false);
  assert.equal(h.playing(), 0);
  h.elements.summary.classList.toggle("level-caution", false);
  h.assess(false, "All systems healthy");
  assert.equal(h.body.classList.contains("is-caution"), false);
});

test("recorded siren is bundled with attribution; heavy full-map filters stay removed", () => {
  const audio = readFileSync(path.join(__dirname, "audio/critical-siren.mp3"));
  assert.ok(audio.length > 10000);
  assert.match(source("audio/README.md"), /BOMB SIREN/);
  assert.match(source("audio/README.md"), /creativecommons.org\/licenses\/by\/3.0/);
  assert.doesNotMatch(source("index.html"), /feTurbulence|feDisplacementMap/);
  assert.doesNotMatch(source("mission.js"), /createOscillator/);
});

function cockpitHarness() {
  const observers = new Map();
  const elements = Object.fromEntries(["i-heading", "heading-gauge", "gauge-arrow", "gauge-value", "source-badge", "deck-data-source", "tab-fly", "tab-plan"].map(id => [id, {
    textContent: "", style: {}, attrs: {}, active: false,
    setAttribute(name, value) { this.attrs[name] = value; },
    classList: { contains: () => id === "tab-fly" }
  }]));
  vm.runInNewContext(source("cockpit.js"), {
    document: { getElementById: id => elements[id] },
    MutationObserver: class {
      constructor(fn) { this.fn = fn; }
      observe(node) { observers.set(node, this.fn); }
    }
  });
  return { elements, update(id, text) { elements[id].textContent = text; observers.get(elements[id])(); } };
}

test("heading gauge mirrors telemetry and rotates through north by the short path", () => {
  const h = cockpitHarness();
  h.update("i-heading", "359°");
  assert.equal(h.elements["gauge-arrow"].style.transform, "rotate(359deg)");
  h.update("i-heading", "001°");
  assert.equal(h.elements["gauge-arrow"].style.transform, "rotate(361deg)");
  assert.equal(h.elements["gauge-value"].textContent, "001°");
  h.update("i-heading", "--");
  assert.equal(h.elements["gauge-arrow"].hidden, true);
  assert.equal(h.elements["heading-gauge"].attrs["aria-label"], "Vehicle heading unavailable");
});

test("instrument dock preserves simulation and offline source labels", () => {
  const h = cockpitHarness();
  h.update("source-badge", "Simulated data");
  assert.equal(h.elements["deck-data-source"].textContent, "SIMULATION");
  h.update("source-badge", "Bridge offline");
  assert.equal(h.elements["deck-data-source"].textContent, "BRIDGE OFFLINE");
  assert.equal(h.elements["tab-fly"].attrs["aria-selected"], "true");
  assert.equal(h.elements["tab-plan"].attrs["aria-selected"], "false");
});
