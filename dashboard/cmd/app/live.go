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
	listen     string // the UDP address the bridge receives MAVLink on
	streaming  bool   // the telemetry stream is up
	lastNowMs  int64
	uploaded   bool // a plan has been sent to the drone from this page
	uploading  bool
	homeSynced bool
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

// detectBridge asks the page's server whether it runs a bridge. A plain static
// server answers 404, which means simulation.
func detectBridge() (listen string, ok bool) {
	resp, err := await(js.Global().Call("fetch", "api/status"))
	if err != nil || !resp.Get("ok").Bool() {
		return "", false
	}
	body, err := await(resp.Call("json"))
	if err != nil {
		return "", false
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
		if u.Snapshot.NowMs < a.live.lastNowMs {
			a.clearEvents() // the bridge restarted, so its event numbers did too
		}
		a.live.lastNowMs = u.Snapshot.NowMs
		a.showEvents(u.Events)
		a.setSnapshot(u.Snapshot)
	})
	// EventSource reconnects by itself. Until then, drop the last values rather
	// than keep showing them as if they were current.
	on(source, "error", func(js.Value) {
		a.live.streaming = false
		a.setSnapshot(model.Snapshot{NowMs: 1})
	})
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
	a.showPlanStatus("Uploaded. Arm and switch to AUTO from your ground control to fly it.", false)
}
