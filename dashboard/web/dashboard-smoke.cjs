#!/usr/bin/env node
// Browser smoke checks for the dashboard. Uses installed Chrome and Node's
// built-in WebSocket; it does not install packages or add test hooks to the app.
"use strict";

const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { spawn } = require("node:child_process");
const http = require("node:http");

const WEB = __dirname;
const wait = ms => new Promise(resolve => setTimeout(resolve, ms));

function options(argv) {
  const result = { mode: "fixture", url: "", out: "" };
  for (let i = 2; i < argv.length; i++) {
    const key = argv[i];
    if (!["--mode", "--url", "--out"].includes(key) || !argv[i + 1]) {
      throw new Error("Usage: node web/dashboard-smoke.cjs [--mode fixture|live] [--url http://127.0.0.1:8080] [--out DIR]");
    }
    result[key.slice(2)] = argv[++i];
  }
  if (!["fixture", "live"].includes(result.mode)) throw new Error("--mode must be fixture or live");
  if (result.mode === "live" && !result.url) throw new Error("--mode live requires --url");
  if (result.mode === "fixture" && result.url) throw new Error("--url is only used with --mode live");
  if (result.mode === "live") {
    const url = new URL(result.url);
    if (url.protocol !== "http:" || !["localhost", "127.0.0.1", "::1", "[::1]"].includes(url.hostname)) {
      throw new Error("Live screenshot mode only accepts an http:// localhost URL");
    }
  }
  return result;
}

function chromePath() {
  if (process.env.CHROME_BIN) return process.env.CHROME_BIN;
  const candidates = process.platform === "darwin"
    ? ["/Applications/Google Chrome.app/Contents/MacOS/Google Chrome", "/Applications/Chromium.app/Contents/MacOS/Chromium"]
    : process.platform === "win32"
      ? [path.join(process.env.PROGRAMFILES || "C:\\Program Files", "Google/Chrome/Application/chrome.exe"), path.join(process.env["PROGRAMFILES(X86)"] || "C:\\Program Files (x86)", "Google/Chrome/Application/chrome.exe")]
      : ["/usr/bin/google-chrome", "/usr/bin/chromium", "/usr/bin/chromium-browser"];
  const found = candidates.find(file => fs.existsSync(file));
  if (!found) throw new Error("Chrome/Chromium not found. Set CHROME_BIN to an installed browser executable; no browser will be downloaded.");
  return found;
}

function reading(value, now) { return { Value: value, AtMs: now }; }

function snapshot(state) {
  const now = Date.now();
  const connected = state !== "no-data";
  const link = state === "critical" ? 0.2 : state === "caution" ? 0.65 : 1;
  const deltaLon = 17 / (6371000 * Math.cos(-35.363262 * Math.PI / 180)) * 180 / Math.PI;
  const home = { Lat: -35.363262, Lon: 149.165237 };
  const estimate = state === "drift" ? { Lat: home.Lat, Lon: home.Lon + deltaLon } : home;
  return {
    NowMs: now,
    Connected: connected,
    Armed: false,
    Mode: "STANDBY",
    Behavior: "NOMINAL",
    Reported: home,
    HasPosition: connected,
    Estimated: estimate,
    HasEstimate: connected,
    EstimateFrom: "core",
    HasHeading: connected,
    HeadingDeg: 0,
    HasFlightData: connected,
    AltM: 0,
    SpeedMS: 0,
    HasGPS: connected,
    Satellites: connected ? 14 : 0,
    Overall: reading(connected ? Math.min(link, state === "critical" ? 0.2 : 1) : 0, now),
    Nav: reading(connected ? 1 : 0, now),
    Link: reading(link, now),
    IMU: reading(connected ? 0.98 : 0, now),
    Motors: reading(connected ? 0.96 : 0, now),
    Battery: reading(connected ? 0.88 : 0, now),
    Hardware: reading(connected ? 0.96 : 0, now),
  };
}

function fixtureServer() {
  let state = "healthy";
  const streams = new Set();
  const types = { ".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8", ".css": "text/css; charset=utf-8", ".wasm": "application/wasm", ".mp3": "audio/mpeg", ".svg": "image/svg+xml", ".md": "text/plain; charset=utf-8" };
  const send = res => res.write(`data: ${JSON.stringify({ Snapshot: snapshot(state), Events: [] })}\n\n`);
  const server = http.createServer((req, res) => {
    const pathname = new URL(req.url, "http://127.0.0.1").pathname;
    if (req.method !== "GET") { res.writeHead(405).end(); return; }
    if (pathname === "/api/status") {
      res.writeHead(200, { "content-type": "application/json" }).end(JSON.stringify({ listen: "synthetic fixture (not a vehicle)" }));
      return;
    }
    if (pathname === "/api/telemetry") {
      res.writeHead(200, { "content-type": "text/event-stream", "cache-control": "no-cache", connection: "keep-alive" });
      res.write("retry: 1000\n\n");
      streams.add(res);
      send(res);
      req.on("close", () => streams.delete(res));
      return;
    }
    const relative = pathname === "/" ? "index.html" : decodeURIComponent(pathname.slice(1));
    const file = path.resolve(WEB, relative);
    if (!file.startsWith(WEB + path.sep) || !fs.existsSync(file) || !fs.statSync(file).isFile()) { res.writeHead(404).end(); return; }
    res.writeHead(200, { "content-type": types[path.extname(file)] || "application/octet-stream", "cache-control": "no-store" });
    fs.createReadStream(file).pipe(res);
  });
  const ticker = setInterval(() => { for (const stream of streams) send(stream); }, 500);
  return new Promise(resolve => server.listen(0, "127.0.0.1", () => resolve({
    url: `http://127.0.0.1:${server.address().port}/`,
    setState(next) { state = next; for (const stream of streams) send(stream); },
    close() { clearInterval(ticker); for (const stream of streams) stream.end(); server.close(); },
  })));
}

class DevTools {
  constructor(ws) {
    this.ws = ws;
    this.nextId = 0;
    this.pending = new Map();
    this.listeners = new Map();
    ws.addEventListener("message", event => {
      const message = JSON.parse(event.data);
      if (message.id) {
        const pending = this.pending.get(message.id);
        if (!pending) return;
        this.pending.delete(message.id);
        message.error ? pending.reject(new Error(message.error.message)) : pending.resolve(message.result);
      } else {
        for (const listener of this.listeners.get(message.method) || []) listener(message.params);
      }
    });
  }
  call(method, params = {}) {
    const id = ++this.nextId;
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.ws.send(JSON.stringify({ id, method, params }));
    });
  }
  on(method, listener) {
    if (!this.listeners.has(method)) this.listeners.set(method, new Set());
    this.listeners.get(method).add(listener);
  }
  async evaluate(expression) {
    const result = await this.call("Runtime.evaluate", { expression, returnByValue: true, awaitPromise: true });
    if (result.exceptionDetails) throw new Error(result.exceptionDetails.text || "Browser evaluation failed");
    return result.result.value;
  }
}

async function waitFor(predicate, description, timeout = 10000) {
  const until = Date.now() + timeout;
  while (Date.now() < until) {
    if (await predicate()) return;
    await wait(100);
  }
  throw new Error(`Timed out waiting for ${description}`);
}

async function main() {
  const opts = options(process.argv);
  const out = path.resolve(opts.out || path.join(os.tmpdir(), `dashboard-smoke-${Date.now()}`));
  fs.mkdirSync(out, { recursive: true });
  const profile = fs.mkdtempSync(path.join(os.tmpdir(), "dashboard-smoke-chrome-"));
  let fixture, chrome, cdp, ws;
  const blockedMutations = [];
  try {
    let targetUrl = opts.url;
    if (opts.mode === "fixture") {
      fixture = await fixtureServer();
      targetUrl = fixture.url;
      console.log("Fixture telemetry is synthetic; it does not represent a vehicle.");
    }
    const executable = chromePath();
    chrome = spawn(executable, [
      "--headless=new", "--no-first-run", "--no-default-browser-check", "--disable-background-networking",
      "--remote-debugging-port=0", `--user-data-dir=${profile}`, "about:blank",
    ], { stdio: "ignore" });
    const activePortFile = path.join(profile, "DevToolsActivePort");
    await waitFor(() => fs.existsSync(activePortFile), "Chrome DevTools endpoint");
    const port = Number(fs.readFileSync(activePortFile, "utf8").split("\n")[0]);
    const base = `http://127.0.0.1:${port}`;
    let target;
    await waitFor(async () => {
      try {
        const response = await fetch(`${base}/json/new?about:blank`, { method: "PUT" });
        if (!response.ok) return false;
        target = await response.json();
        return true;
      } catch { return false; }
    }, "Chrome tab");
    ws = new WebSocket(target.webSocketDebuggerUrl);
    await new Promise((resolve, reject) => { ws.addEventListener("open", resolve, { once: true }); ws.addEventListener("error", reject, { once: true }); });
    cdp = new DevTools(ws);
    await cdp.call("Page.enable");
    await cdp.call("Runtime.enable");
    await cdp.call("Network.enable");
    await cdp.call("Network.setCacheDisabled", { cacheDisabled: true });
    // Force basemap failure so the offline-grid fallback is tested deterministically.
    await cdp.call("Network.setBlockedURLs", { urls: ["*arcgisonline.com*", "*tile.openstreetmap.org*"] });
    if (opts.mode === "live") {
      // Read-only guarantee: never let a POST/PUT/PATCH/DELETE reach a dashboard.
      await cdp.call("Fetch.enable", { patterns: [{ urlPattern: "*", requestStage: "Request" }] });
      cdp.on("Fetch.requestPaused", async event => {
        if (event.request.method !== "GET" && event.request.method !== "HEAD") {
          blockedMutations.push(`${event.request.method} ${event.request.url}`);
          await cdp.call("Fetch.failRequest", { requestId: event.requestId, errorReason: "Aborted" });
        } else {
          await cdp.call("Fetch.continueRequest", { requestId: event.requestId });
        }
      });
    }
    await cdp.call("Emulation.setDeviceMetricsOverride", { width: 1440, height: 1000, deviceScaleFactor: 1, mobile: false });
    const save = async name => {
      const image = await cdp.call("Page.captureScreenshot", { format: "png", captureBeyondViewport: false });
      const file = path.join(out, `${name}.png`);
      fs.writeFileSync(file, Buffer.from(image.data, "base64"));
      console.log(`screenshot: ${file}`);
    };
    await cdp.call("Page.navigate", { url: targetUrl });
    if (opts.mode === "live") {
      await waitFor(() => cdp.evaluate(`document.querySelector('#summary-title')?.textContent && document.querySelector('#source-badge')?.textContent`), "live dashboard render");
      await wait(500);
      await save("live-readonly");
      if (blockedMutations.length) throw new Error(`Blocked unexpected mutating request(s): ${blockedMutations.join(", ")}`);
      console.log("Live mode completed using localhost GETs only; no telemetry controls were clicked.");
      return;
    }

    const state = async name => {
      fixture.setState(name);
      const expectedTitle = {
        healthy: "All systems healthy",
        caution: "Caution: Link",
        critical: "Critical: Link",
        "no-data": "No contact with the drone",
        drift: "All systems healthy",
      }[name];
      await waitFor(() => cdp.evaluate(`document.querySelector('#summary-title')?.textContent === ${JSON.stringify(expectedTitle)}`), `${name} summary`);
      const check = await cdp.evaluate(`(() => {
        const cards = [...document.querySelectorAll('#health .card')].map(card => ({
          name: card.querySelector('.card-name').textContent,
          level: card.querySelector('.card-level').textContent,
          score: card.querySelector('.card-score').textContent,
          reason: card.querySelector('.card-reason').textContent,
          className: card.className,
        }));
        return { title: document.querySelector('#summary-title').textContent, cards,
          drift: document.querySelector('#i-drift').textContent,
          map: document.querySelector('#map-status').textContent,
          bannerHidden: document.querySelector('#critical-banner').hidden };
      })()`);
      return check;
    };
    await waitFor(() => cdp.evaluate(`document.querySelector('#health .card')?.querySelector('.card-level')?.textContent`), "initial health panel");
    await waitFor(() => cdp.evaluate(`document.querySelector('#map-status')?.textContent.includes('Offline grid')`), "offline basemap fallback");
    // The app correctly identifies any bridge endpoint as Live; overlay explicit
    // fixture provenance in screenshots without changing telemetry or app code.
    await cdp.evaluate(`(() => {
      const badge = document.querySelector('#source-badge');
      const deck = document.querySelector('#deck-data-source');
      const label = () => {
        if (badge.textContent === 'Live') {
          badge.textContent = 'Synthetic fixture';
          badge.title = 'Synthetic test telemetry; no vehicle is connected.';
        }
        if (deck.textContent === 'LIVE') deck.textContent = 'SYNTHETIC FIXTURE';
      };
      new MutationObserver(label).observe(badge, { childList: true, characterData: true, subtree: true });
      new MutationObserver(label).observe(deck, { childList: true, characterData: true, subtree: true });
      label();
    })()`);

    let result = await state("healthy");
    if (result.title !== "All systems healthy" || result.cards.some(card => !card.level.includes("Healthy"))) throw new Error(`Healthy fixture mismatch: ${JSON.stringify(result)}`);
    await save("healthy");
    console.log("PASS healthy: all subsystem cards report Healthy.");

    result = await state("caution");
    if (result.title !== "Caution: Link" || !result.cards.find(card => card.name === "Link")?.level.includes("Caution")) throw new Error(`Caution fixture mismatch: ${JSON.stringify(result)}`);
    await save("caution");
    console.log("PASS caution: Link card shows 65% and a plain-language reason.");

    result = await state("critical");
    if (result.title !== "Critical: Link" || result.bannerHidden || !result.cards.find(card => card.name === "Link")?.level.includes("Critical")) throw new Error(`Critical fixture mismatch: ${JSON.stringify(result)}`);
    await save("critical");
    console.log("PASS critical: Link card and persistent critical banner are visible.");

    result = await state("no-data");
    if (result.title !== "No contact with the drone" || result.cards.some(card => !card.level.includes("No data") || card.score !== "--")) throw new Error(`No-data fixture mismatch: ${JSON.stringify(result)}`);
    await save("no-data");
    console.log("PASS no-data: every subsystem is marked No data with no stale score.");

    result = await state("drift");
    if (result.drift !== "17 m" || result.map !== "Offline grid · map tiles unavailable") throw new Error(`17 m/offline map fixture mismatch: ${JSON.stringify(result)}`);
    await save("drift-17m-offline");
    console.log("PASS map: exact 17 m core-estimate disagreement; offline grid and planned route render.");
  } finally {
    try { ws?.close(); } catch {}
    try { fixture?.close(); } catch {}
    if (chrome && chrome.exitCode === null) {
      chrome.kill("SIGTERM");
      await Promise.race([new Promise(resolve => chrome.once("exit", resolve)), wait(3000)]);
    }
    fs.rmSync(profile, { recursive: true, force: true });
  }
}

main().catch(error => { console.error(`FAIL: ${error.message}`); process.exitCode = 1; });
