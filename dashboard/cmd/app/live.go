//go:build js && wasm

package main

import (
	"encoding/json"
	"errors"
	"syscall/js"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

// liveState is the connection to the bridge served alongside the page.
type liveState struct {
	listen         string // the UDP address the bridge receives MAVLink on
	streaming      bool   // the telemetry stream is up
	lastNowMs      int64
	lastMessage    float64 // performance.now() of the newest message
	uploaded       bool    // a plan has been sent to the drone from this page
	uploading      bool
	starting       bool // a start-flight request is in flight
	injecting      bool // a fault-injection request is in flight
	recovering     bool // an automatic-recovery setting request is in flight
	recoveryHTTP   bool // HTTP must finish before another setting request starts
	resetting      bool
	resetConfirm   bool
	recoveryTarget bool
	homeSynced     bool
}

// await blocks the calling goroutine on a promise, so it must not be called
// directly from a browser callback.
func await(promise js.Value) (js.Value, error) {
	result := make(chan js.Value, 1)
	failure := make(chan js.Value, 1)
	resolve := js.FuncOf(func(_ js.Value, args []js.Value) any { result <- args[0]; return nil })
	defer resolve.Release()
	reject := js.FuncOf(func(_ js.Value, args []js.Value) any { failure <- args[0]; return nil })
	defer reject.Release()
	promise.Call("then", resolve, reject)
	select {
	case v := <-result:
		return v, nil
	case v := <-failure:
		return js.Undefined(), errors.New(v.Call("toString").String())
	}
}

// detectBridge asks the page's server whether it runs a bridge. Only a 404, from a
// plain static server, means simulation. Any other failure stays in live mode and
// shows as offline, so a broken bridge can't pass for a simulated flight or vice versa.
func detectBridge() (listen string, ok bool) {
	resp, err := await(js.Global().Call("fetch", "api/status"))
	if err != nil {
		return "", true
	}
	if resp.Get("status").Int() == 404 {
		return "", false
	}
	if !resp.Get("ok").Bool() {
		return "", true
	}
	body, err := await(resp.Call("json"))
	if err != nil {
		return "", true
	}
	return body.Get("listen").String(), true
}

func (a *app) startStream() {
	source := js.Global().Get("EventSource").New("api/telemetry")
	on(source, "message", func(e js.Value) {
		var u model.Update
		if err := json.Unmarshal([]byte(e.Get("data").String()), &u); err != nil {
			return
		}
		a.live.streaming = true
		a.live.lastMessage = js.Global().Get("performance").Call("now").Float()
		if u.Snapshot.NowMs < a.live.lastNowMs {
			a.clearEvents() // the bridge restarted, so its event numbers did too
		}
		a.live.lastNowMs = u.Snapshot.NowMs
		a.showEvents(u.Events)
		a.setSnapshot(u.Snapshot)
		if a.live.recovering && !a.live.recoveryHTTP && a.recoveryFresh() && u.Snapshot.RecoveryEnabled == a.live.recoveryTarget {
			a.live.recovering = false
			a.renderRecovery()
			a.showRecoveryStatus("", false)
		}
	})
	// EventSource reconnects by itself. Until then, drop the last values rather
	// than keep showing them as if they were current.
	lost := func() {
		a.live.streaming = false
		a.setSnapshot(model.Snapshot{NowMs: 1})
	}
	on(source, "error", func(js.Value) { lost() })

	// A stalled connection doesn't always raise an error, so watch for silence too.
	watchdog := js.FuncOf(func(js.Value, []js.Value) any {
		silence := js.Global().Get("performance").Call("now").Float() - a.live.lastMessage
		if a.live.streaming && silence > model.StaleAfterMs {
			lost()
		}
		return nil
	})
	js.Global().Call("setInterval", watchdog, 1000)
	a.renderFly()
}

// uploadPlan sends the plan to the drone. It runs as a goroutine.
func (a *app) uploadPlan() {
	plan := a.plan.Clone()
	data, err := json.Marshal(plan)
	if err != nil {
		a.showPlanStatus(err.Error(), true)
		return
	}

	a.live.uploading = true
	a.showPlanStatus("Uploading…", false)
	a.renderPlanAction()
	defer func() {
		a.live.uploading = false
		a.renderPlanAction()
	}()

	headers := map[string]any{"Content-Type": "application/json"}
	resp, err := await(js.Global().Call("fetch", "api/mission", map[string]any{
		"method":  "POST",
		"headers": headers,
		"body":    string(data),
	}))
	if err != nil {
		a.showPlanStatus("Couldn't reach the bridge: "+err.Error(), true)
		return
	}
	body, err := await(resp.Call("json"))
	if err != nil {
		a.showPlanStatus("The bridge sent an unreadable reply.", true)
		return
	}
	if !resp.Get("ok").Bool() {
		a.showPlanStatus(body.Get("error").String(), true)
		return
	}

	a.live.uploaded = true
	a.flownPlan = plan
	a.showPlanStatus("Uploaded. Use Start flight in the Fly tab when ready.", false)
}

// startFlight arms the drone and starts the uploaded mission. It runs as a goroutine.
func (a *app) startFlight() {
	a.live.starting = true
	a.showLiveStatus("Starting…", false)
	a.renderFlyAction()
	defer func() {
		a.live.starting = false
		a.renderFlyAction()
	}()

	resp, err := await(js.Global().Call("fetch", "api/start", map[string]any{"method": "POST"}))
	if err != nil {
		a.showLiveStatus("Couldn't reach the bridge: "+err.Error(), true)
		return
	}
	body, err := await(resp.Call("json"))
	if err != nil {
		a.showLiveStatus("The bridge sent an unreadable reply.", true)
		return
	}
	if !resp.Get("ok").Bool() {
		a.showLiveStatus(body.Get("error").String(), true)
		return
	}

	a.showLiveStatus("Flight started.", false)
}

// injectFault asks the bridge to inject a fault into SITL. It runs as a goroutine.
func (a *app) injectFault(fault string) {
	a.live.injecting = true
	a.showFaultStatus("", false)
	a.renderFaults()
	defer func() {
		a.live.injecting = false
		a.renderFaults()
	}()

	data, _ := json.Marshal(map[string]string{"fault": fault})
	resp, err := await(js.Global().Call("fetch", "api/fault", map[string]any{
		"method":  "POST",
		"headers": map[string]any{"Content-Type": "application/json"},
		"body":    string(data),
	}))
	if err != nil {
		a.showFaultStatus("Couldn't reach the bridge: "+err.Error(), true)
		return
	}
	body, err := await(resp.Call("json"))
	if err != nil {
		a.showFaultStatus("The bridge sent an unreadable reply.", true)
		return
	}
	if !resp.Get("ok").Bool() {
		a.showFaultStatus(body.Get("error").String(), true)
		return
	}
	// The telemetry stream confirms it too, but not for up to 200 ms.
	a.snap.Fault = fault
}

// setRecovery sends a setting request but waits for telemetry to confirm the
// resulting state; a successful HTTP response is not treated as confirmation.
func (a *app) setRecovery(enabled bool) {
	a.live.recovering = true
	a.live.recoveryHTTP = true
	a.live.recoveryTarget = enabled
	a.showRecoveryStatus("Waiting for core confirmation…", false)
	a.renderRecovery()
	defer func() { a.live.recoveryHTTP = false }()

	data, _ := json.Marshal(map[string]bool{"enabled": enabled})
	resp, err := await(js.Global().Call("fetch", "api/recovery", map[string]any{
		"method":  "POST",
		"headers": map[string]any{"Content-Type": "application/json"},
		"body":    string(data),
	}))
	if err != nil {
		a.live.recovering = false
		a.showRecoveryStatus("Couldn't reach the bridge: "+err.Error(), true)
		a.renderRecovery()
		return
	}
	body, err := await(resp.Call("json"))
	if err != nil {
		a.live.recovering = false
		a.showRecoveryStatus("The bridge sent an unreadable reply.", true)
		a.renderRecovery()
		return
	}
	if !resp.Get("ok").Bool() {
		a.live.recovering = false
		a.showRecoveryStatus(body.Get("error").String(), true)
		a.renderRecovery()
		return
	}
	// A confirming SSE event can arrive before the HTTP response. It must not
	// leave the control stuck waiting, nor allow a second concurrent request.
	if a.recoveryFresh() && a.snap.RecoveryEnabled == enabled {
		a.live.recovering = false
		a.showRecoveryStatus("", false)
		a.renderRecovery()
	} else {
		a.showRecoveryStatus("Request accepted; waiting for telemetry confirmation.", false)
	}
}

func (a *app) resetState() {
	a.live.resetting, a.live.resetConfirm = true, false
	a.showRecoveryStatus("Clearing simulated threats and requesting ground rebaseline…", false)
	a.renderRecovery()
	a.renderFlyAction()
	defer func() { a.live.resetting = false; a.renderRecovery(); a.renderFlyAction() }()
	resp, err := await(js.Global().Call("fetch", "api/reset", map[string]any{
		"method": "POST", "headers": map[string]any{"Content-Type": "application/json"},
		"body": `{"confirm":true}`,
	}))
	if err != nil {
		a.showRecoveryStatus("Couldn't reach the bridge: "+err.Error(), true)
		return
	}
	body, err := await(resp.Call("json"))
	if err != nil {
		a.showRecoveryStatus("The bridge sent an unreadable reset reply.", true)
		return
	}
	if !resp.Get("ok").Bool() {
		a.showRecoveryStatus(body.Get("error").String(), true)
		return
	}
	a.showRecoveryStatus("Ground baseline revalidated. Trust will recover only as new sensor checks pass. Mission preserved.", false)
}
