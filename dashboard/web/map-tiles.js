/* Raster basemap for the existing local-metre canvas; no map framework or API key. */
(() => {
  "use strict";
  const radius = 6371000; // Match model.ToLocal, not Web Mercator's nominal radius.
  const radians = Math.PI / 180;
  const cache = new Map();
  const maxTiles = 160;
  const status = document.getElementById("map-status");
  let lastStatus = "";
  let style = "satellite";
  const satellite = document.getElementById("map-satellite");
  const street = document.getElementById("map-street");
  function selectStyle(next) {
    style = next;
    satellite?.setAttribute("aria-pressed", String(style === "satellite"));
    street?.setAttribute("aria-pressed", String(style === "street"));
    const credit = document.getElementById("map-credit");
    if (credit) {
      credit.textContent = style === "satellite" ? "Esri, Vantor, Earthstar Geographics, GIS User Community" : "© OpenStreetMap contributors";
      credit.href = style === "satellite" ? "https://goto.arcgisonline.com/maps/World_Imagery" : "https://www.openstreetmap.org/copyright";
    }
    const note = document.getElementById("map-imagery-note");
    if (note) note.textContent = style === "satellite" ? "SATELLITE BASEMAP / NOT LIVE VIDEO" : "STREET BASEMAP / NORTH UP";
  }
  satellite?.addEventListener("click", () => selectStyle("satellite"));
  street?.addEventListener("click", () => selectStyle("street"));

  function setStatus(text) {
    if (text !== lastStatus) {
      status.textContent = text;
      lastStatus = text;
    }
  }

  function tile(z, x, y) {
    const key = `${style}/${z}/${x}/${y}`;
    let entry = cache.get(key);
    if (entry && (!entry.failedAt || Date.now() - entry.failedAt < 60000)) {
      // Bounded LRU: repeated animation frames must not issue more requests.
      cache.delete(key);
      cache.set(key, entry);
      return entry;
    }
    const image = new Image();
    // Both providers allow CORS; keep the canvas readable for local GPU effects.
    image.crossOrigin = "anonymous";
    entry = { image, ready: false, failedAt: 0 };
    image.referrerPolicy = "strict-origin-when-cross-origin";
    image.onload = () => { entry.ready = true; };
    image.onerror = () => { entry.failedAt = Date.now(); };
    cache.set(key, entry);
    image.src = style === "satellite"
      ? `https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/${z}/${y}/${x}`
      : `https://tile.openstreetmap.org/${z}/${x}/${y}.png`;
    while (cache.size > maxTiles) cache.delete(cache.keys().next().value);
    return entry;
  }

  const tileY = (lat, n) => (1 - Math.asinh(Math.tan(Math.max(-85.05112878, Math.min(85.05112878, lat)) * radians)) / Math.PI) / 2 * n;
  const latitude = (y, n) => Math.atan(Math.sinh(Math.PI * (1 - 2 * y / n))) / radians;

  window.TrustIssuesMap = {
    draw(ctx, width, height, lat, lon, centerEast, centerNorth, scale) {
      if (!(width > 0 && height > 0 && scale > 0) || Math.abs(lat) > 85) {
        setStatus("Offline grid · basemap unavailable at this latitude");
        return;
      }
      const metresPerDegree = radius * radians;
      const longitudeScale = metresPerDegree * Math.cos(lat * radians);
      const zoom = Math.max(0, Math.min(19, Math.round(Math.log2(2 * Math.PI * radius * Math.cos(lat * radians) * scale / 256))));
      const n = 2 ** zoom;
      const west = lon + (centerEast - width / 2 / scale) / longitudeScale;
      const east = lon + (centerEast + width / 2 / scale) / longitudeScale;
      const north = lat + (centerNorth + height / 2 / scale) / metresPerDegree;
      const south = lat + (centerNorth - height / 2 / scale) / metresPerDegree;
      const firstX = Math.floor((west + 180) / 360 * n);
      const lastX = Math.floor((east + 180) / 360 * n);
      const firstY = Math.max(0, Math.floor(tileY(north, n)));
      const lastY = Math.min(n - 1, Math.floor(tileY(south, n)));
      let loaded = 0, failed = 0, total = 0;
      ctx.save();
      // Keep street labels legible in the dark console without a paid tile style.
      ctx.filter = style === "street" ? "invert(1) hue-rotate(180deg) saturate(.45) brightness(.85)" : "brightness(.8) saturate(.85)";
      // No prefetching: only request the visible map, retaining a small local cache.
      for (let x = firstX; x <= lastX; x++) {
        for (let y = firstY; y <= lastY; y++) {
          total++;
          const entry = tile(zoom, ((x % n) + n) % n, y);
          if (entry.failedAt) failed++;
          if (!entry.ready) continue;
          loaded++;
          const left = width / 2 + (((x / n * 360 - 180) - lon) * longitudeScale - centerEast) * scale;
          const right = width / 2 + ((((x + 1) / n * 360 - 180) - lon) * longitudeScale - centerEast) * scale;
          const top = height / 2 - ((latitude(y, n) - lat) * metresPerDegree - centerNorth) * scale;
          const bottom = height / 2 - ((latitude(y + 1, n) - lat) * metresPerDegree - centerNorth) * scale;
          ctx.drawImage(entry.image, left, top, right - left + .5, bottom - top + .5);
        }
      }
      ctx.restore();
      const label = style === "satellite" ? "Satellite map" : "Street map";
      setStatus(failed
        ? (loaded ? `${label} · missing tiles use offline grid` : "Offline grid · map tiles unavailable")
        : loaded === total && total > 0
          ? `${label} · drag to pan / scroll to zoom`
          : "Loading map · offline grid available");
    }
  };
})();
