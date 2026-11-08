/* Presentation observes assessed states; thresholds and flight controls are unchanged. */
(() => {
  "use strict";
  const body = document.body;
  const summary = document.getElementById("summary");
  const banner = document.getElementById("critical-banner");
  const detail = document.getElementById("critical-detail");
  const toggle = document.getElementById("alarm-toggle");
  const AudioContext = window.AudioContext || window.webkitAudioContext;
  let audio = null, sample = null, loading = null, voice = null;
  let muted = false, critical = false, failed = false;

  function renderSoundControl() {
    const enabled = !muted && !failed && audio?.state === "running";
    toggle.textContent = !AudioContext ? "Audio unavailable" : failed ? "Retry alarm audio" : loading ? "Loading alarm…" : enabled ? "Alarm sound on" : muted ? "Alarm muted" : "Enable alarm";
    toggle.disabled = !AudioContext;
    toggle.setAttribute("aria-pressed", String(enabled));
    toggle.title = enabled ? "Mute the emergency siren (visual warnings stay on)" : "Enable the recorded emergency siren";
  }

  function stopSound() {
    if (!voice) return;
    const previous = voice;
    voice = null;
    previous.source.stop();
    previous.source.disconnect();
    previous.gain.disconnect();
  }

  function syncSound() {
    const shouldPlay = critical && !muted && !document.hidden && sample && audio?.state === "running";
    if (!shouldPlay) stopSound();
    else if (!voice) {
      const source = audio.createBufferSource();
      const gain = audio.createGain();
      source.buffer = sample;
      source.loop = true;
      // The mastered asset is already -20 LUFS. Do not turn this into a jump scare.
      gain.gain.setValueAtTime(0, audio.currentTime);
      gain.gain.linearRampToValueAtTime(.7, audio.currentTime + .08);
      source.connect(gain);
      gain.connect(audio.destination);
      voice = { source, gain };
      source.start();
    }
    renderSoundControl();
  }

  async function loadSample() {
    if (sample) return;
    if (!loading) {
      loading = (async () => {
        const response = await fetch("audio/critical-siren.mp3");
        if (!response.ok) throw new Error("Alarm asset unavailable");
        sample = await audio.decodeAudioData(await response.arrayBuffer());
      })();
      renderSoundControl();
    }
    try { await loading; }
    finally { loading = null; }
  }

  async function unlockSound() {
    if (!AudioContext || muted || document.hidden) return;
    failed = false;
    try {
      if (!audio) {
        audio = new AudioContext();
        audio.addEventListener("statechange", syncSound);
      }
      if (audio.state !== "running") await audio.resume();
      await loadSample();
      syncSound();
    } catch {
      failed = true;
      stopSound();
      renderSoundControl();
    }
  }

  toggle.addEventListener("click", () => {
    if (!failed && !muted && audio?.state === "running") {
      muted = true;
      syncSound();
    } else {
      muted = false;
      void unlockSound();
    }
  });
  // Unlock/decode after a gesture, but don't play anything until actual critical.
  function onGesture(event) {
    if (event.target.closest?.("#alarm-toggle") || muted || failed || loading || (sample && audio?.state === "running")) return;
    void unlockSound();
  }
  document.addEventListener("pointerdown", onGesture);
  document.addEventListener("keydown", onGesture);
  document.addEventListener("visibilitychange", syncSound);
  window.addEventListener("pagehide", stopSound);

  function syncAlert() {
    const next = summary.classList.contains("level-critical");
    body.classList.toggle("is-caution", summary.classList.contains("level-caution"));
    const message = document.getElementById("summary-title").textContent;
    if (next && detail.textContent !== message) detail.textContent = message;
    if (next !== critical) {
      critical = next;
      body.classList.toggle("is-critical", critical);
      banner.hidden = !critical;
      syncSound();
    }
  }
  new MutationObserver(syncAlert).observe(summary, {
    attributes: true, attributeFilter: ["class"], childList: true, characterData: true, subtree: true
  });
  renderSoundControl();
  syncAlert();
})();
