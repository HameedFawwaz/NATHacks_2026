//go:build js && wasm

// Command app is the dashboard's browser side, compiled to WebAssembly.
//
// Every callback runs on the browser's event loop, so state needs no locking.
package main

import (
	"encoding/json"
	"syscall/js"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/sim"
)

const (
	planStorageKey = "dashboard.plan"
	tickMs         = 100
	minTrailStepM  = 1.0
	maxTrail       = 5000
)

type tab int

const (
	tabFly tab = iota
	tabPlan
)

type app struct {
	tab  tab
	plan model.Plan // the plan being edited; the flight flies its own copy

	sim    *sim.Sim
	snap   model.Snapshot
	events int // how many sim events the log already shows

	reportedTrail  []model.LatLon
	estimatedTrail []model.LatLon

	mapView *mapView
	health  *healthPanel
}

func main() {
	a := &app{plan: loadPlan()}
	a.sim = sim.New(a.plan)
	a.snap = a.sim.Snapshot()
	a.mapView = newMapView(a)
	a.health = newHealthPanel()

	a.bindShell()
	a.bindFly()
	a.bindPlan()
	a.setTab(tabFly)
	a.mapView.fit()

	onTick := js.FuncOf(func(js.Value, []js.Value) any {
		a.tick()
		return nil
	})
	js.Global().Call("setInterval", onTick, tickMs)
	a.tick()
	a.mapView.startDrawing()

	select {}
}

func (a *app) tick() {
	a.sim.Step(float64(tickMs) / 1000)
	a.snap = a.sim.Snapshot()
	if a.sim.Flying() {
		a.reportedTrail = appendTrail(a.reportedTrail, a.snap.Reported)
		a.estimatedTrail = appendTrail(a.estimatedTrail, a.snap.Estimated)
	}
	a.renderFly()
}

func appendTrail(trail []model.LatLon, p model.LatLon) []model.LatLon {
	if n := len(trail); n > 0 && model.DistanceM(trail[n-1], p) < minTrailStepM {
		return trail
	}
	if len(trail) >= maxTrail {
		trail = trail[1:]
	}
	return append(trail, p)
}

func (a *app) bindShell() {
	onClick(byID("tab-fly"), func(js.Value) { a.setTab(tabFly) })
	onClick(byID("tab-plan"), func(js.Value) { a.setTab(tabPlan) })
	onClick(byID("map-fit"), func(js.Value) { a.mapView.fit() })
}

func (a *app) setTab(t tab) {
	a.tab = t
	fly := t == tabFly
	setClass(byID("tab-fly"), "active", fly)
	setClass(byID("tab-plan"), "active", !fly)
	byID("side-fly").Set("hidden", !fly)
	byID("legend").Set("hidden", !fly)
	byID("instruments").Set("hidden", !fly)
	byID("side-plan").Set("hidden", fly)
	byID("plan-hint").Set("hidden", fly)
	if fly {
		// An idle drone picks up whatever was edited in the meantime.
		if !a.sim.Flying() {
			a.newFlight()
		}
	} else {
		a.renderPlan()
	}
	a.mapView.resize()
	a.mapView.fit()
}

// newFlight replaces the simulated flight with a fresh one on the current plan.
func (a *app) newFlight() {
	old := a.sim.Events()
	a.sim = sim.New(a.plan)
	a.snap = a.sim.Snapshot()
	a.reportedTrail, a.estimatedTrail = nil, nil
	if len(old) > 0 {
		a.events = 0
		clearChildren(byID("events"))
	}
	a.renderFly()
}

func loadPlan() model.Plan {
	stored := js.Global().Get("localStorage").Call("getItem", planStorageKey)
	if stored.IsNull() {
		return model.DefaultPlan()
	}
	var p model.Plan
	if err := json.Unmarshal([]byte(stored.String()), &p); err != nil || p.Home == (model.LatLon{}) {
		return model.DefaultPlan()
	}
	return p
}

func (a *app) savePlan() {
	data, err := json.Marshal(a.plan)
	if err != nil {
		return
	}
	js.Global().Get("localStorage").Call("setItem", planStorageKey, string(data))
}
