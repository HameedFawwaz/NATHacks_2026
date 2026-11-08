//go:build js && wasm

// Command app is the dashboard's browser side, compiled to WebAssembly. It shows
// live telemetry when the page is served with a bridge (`serve -mavlink`) and a
// simulated flight otherwise.
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
	maxEventRows   = 200
)

type tab int

const (
	tabFly tab = iota
	tabPlan
)

type app struct {
	tab       tab
	plan      model.Plan // the plan being edited
	planSaved bool       // the plan came from storage, rather than being the default
	flownPlan model.Plan // the route shown on the Fly tab; edits don't touch it
	snap      model.Snapshot

	live       *liveState // nil when simulating
	sim        *sim.Sim   // nil when live
	shownEvent int        // Seq of the newest event in the log

	reportedTrail  []model.LatLon
	estimatedTrail []model.LatLon
	wasArmed       bool

	mapView *mapView
	health  *healthPanel
}

func main() {
	a := &app{}
	a.plan, a.planSaved = loadPlan()
	a.flownPlan = a.plan.Clone()
	a.mapView = newMapView(a)
	a.health = newHealthPanel()

	listen, live := detectBridge()
	if live {
		a.live = &liveState{listen: listen}
		a.snap = model.Snapshot{NowMs: 1}
	} else {
		a.sim = sim.New(a.flownPlan)
		a.snap = a.sim.Snapshot()
	}

	a.bindShell()
	a.bindFly()
	a.bindPlan()
	a.setTab(tabFly)
	a.mapView.fit()

	if live {
		a.startStream()
	} else {
		onTick := js.FuncOf(func(js.Value, []js.Value) any {
			a.simTick()
			return nil
		})
		js.Global().Call("setInterval", onTick, tickMs)
		a.simTick()
	}
	a.mapView.startDrawing()

	select {}
}

func (a *app) simTick() {
	a.sim.Step(float64(tickMs) / 1000)
	a.showEvents(a.sim.Events())
	a.setSnapshot(a.sim.Snapshot())
}

// setSnapshot takes in the vehicle's latest state from either source.
func (a *app) setSnapshot(s model.Snapshot) {
	if s.Armed && !a.wasArmed {
		a.reportedTrail, a.estimatedTrail = nil, nil
	}
	a.wasArmed = s.Armed
	a.snap = s
	if s.Armed && s.Connected {
		if s.HasPosition {
			a.reportedTrail = appendTrail(a.reportedTrail, s.Reported)
		}
		if s.HasEstimate {
			a.estimatedTrail = appendTrail(a.estimatedTrail, s.Estimated)
		}
	}
	a.syncHome(s)
	a.renderFly()
	a.renderPlanAction()
}

// syncHome moves the plan's home to where a live drone first reports being on the
// ground, so the route and the drone appear on the same map.
func (a *app) syncHome(s model.Snapshot) {
	if a.live == nil || a.live.homeSynced || !s.Connected || !s.HasPosition || s.Armed {
		return
	}
	a.live.homeSynced = true
	if a.planSaved {
		a.plan.Home = s.Reported // the waypoints stay put: they're real places
	} else {
		a.plan = model.DefaultPlanAt(s.Reported)
	}
	a.savePlan()
	a.syncFlownPlan()
	if a.tab == tabPlan {
		a.renderPlan()
	}
	a.mapView.fit()
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
		a.syncFlownPlan()
	} else {
		a.renderPlan()
	}
	a.mapView.resize()
	a.mapView.fit()
}

// syncFlownPlan shows edits on the Fly tab when nothing else claims that route:
// an idle simulated drone, or a live one that hasn't been given a plan here.
func (a *app) syncFlownPlan() {
	switch {
	case a.sim != nil && !a.sim.Flying():
		a.newFlight()
	case a.live != nil && !a.live.uploaded:
		a.flownPlan = a.plan.Clone()
	}
}

// newFlight replaces the simulated flight with a fresh one on the current plan.
func (a *app) newFlight() {
	a.flownPlan = a.plan.Clone()
	a.sim = sim.New(a.flownPlan)
	a.reportedTrail, a.estimatedTrail = nil, nil
	a.clearEvents()
	a.setSnapshot(a.sim.Snapshot())
}

func loadPlan() (plan model.Plan, saved bool) {
	stored := js.Global().Get("localStorage").Call("getItem", planStorageKey)
	if stored.IsNull() {
		return model.DefaultPlan(), false
	}
	var p model.Plan
	if err := json.Unmarshal([]byte(stored.String()), &p); err != nil || p.Home == (model.LatLon{}) {
		return model.DefaultPlan(), false
	}
	return p, true
}

func (a *app) savePlan() {
	a.planSaved = true
	data, err := json.Marshal(a.plan)
	if err != nil {
		return
	}
	js.Global().Get("localStorage").Call("setItem", planStorageKey, string(data))
}
