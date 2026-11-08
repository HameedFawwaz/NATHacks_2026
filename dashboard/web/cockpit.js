/* Read-only instrument mirrors. No synthetic values or vehicle commands. */
(() => {
  "use strict";
  const heading = document.getElementById("i-heading");
  const gauge = document.getElementById("heading-gauge");
  const arrow = document.getElementById("gauge-arrow");
  const value = document.getElementById("gauge-value");
  const source = document.getElementById("source-badge");
  const effects = document.getElementById("map-effects");
  const stage = document.getElementById("map-stage");
  if (effects && stage) {
    const reducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)");
    let enabled = true;
    const renderEffects = () => {
      const active = enabled && !reducedMotion.matches;
      stage.classList.toggle("fx-disabled", !active);
      effects.setAttribute("aria-pressed", String(active));
      effects.disabled = reducedMotion.matches;
      effects.textContent = active ? "Glitch FX on" : "Glitch FX off";
      effects.title = reducedMotion.matches ? "Glitch effects disabled by reduced-motion preference" : active ? "Disable critical map glitch effects" : "Enable critical map glitch effects";
    };
    effects.addEventListener("click", () => { enabled = !enabled; renderEffects(); });
    reducedMotion.addEventListener("change", renderEffects);
    renderEffects();
  }
  let previous = "";
  let rotation = 0;

  function renderHeading() {
    const text = heading.textContent;
    if (text === previous) return;
    const degrees = Number.parseFloat(text);
    const old = Number.parseFloat(previous);
    previous = text;
    value.textContent = text || "--";
    gauge.setAttribute("aria-label", Number.isFinite(degrees) ? `Vehicle heading ${degrees} degrees` : "Vehicle heading unavailable");
    arrow.hidden = !Number.isFinite(degrees);
    if (Number.isFinite(degrees)) {
      // Take the short path across north instead of spinning through 359 degrees.
      rotation = Number.isFinite(old) ? rotation + ((degrees - old + 540) % 360 - 180) : degrees;
      arrow.style.transform = `rotate(${rotation}deg)`;
    }
  }
  function renderSource() {
    document.getElementById("deck-data-source").textContent = source.textContent === "Simulated data" ? "SIMULATION" : source.textContent.toUpperCase() || "WAITING";
  }
  new MutationObserver(renderHeading).observe(heading, { childList: true, characterData: true, subtree: true });
  new MutationObserver(renderSource).observe(source, { childList: true, characterData: true, subtree: true });
  for (const id of ["tab-fly", "tab-plan"]) {
    const tab = document.getElementById(id);
    const update = () => tab.setAttribute("aria-selected", String(tab.classList.contains("active")));
    new MutationObserver(update).observe(tab, { attributes: true, attributeFilter: ["class"] });
    update();
  }
  renderHeading();
  renderSource();
})();
