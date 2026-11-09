//go:build js && wasm

package main

import (
	"fmt"
	"math"
	"strconv"
	"strings"
	"syscall/js"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/sim"
)

func (a *app) bindFly() {
	if a.live != nil {
		byID("sim-controls").Set("hidden", true)
		byID("sim-demo").Set("hidden", true)
		byID("live-controls").Set("hidden", false)
		byID("live-demo").Set("hidden", false)
		onClick(byID("live-start"), func(js.Value) {
			if !a.live.starting {
				go a.startFlight()
			}
		})
		onClick(byID("live-faults"), func(e js.Value) {
			value := e.Get("target").Call("closest", "button[data-fault]")
			if value.IsNull() || a.live.injecting || !a.snap.Connected {
				return
			}
			go a.injectFault(value.Call("getAttribute", "data-fault").String())
		})
		return
	}
	onClick(byID("flight-start"), func(js.Value) {
		if !a.sim.Flying() {
			a.newFlight()
			a.sim.Start()
			a.simTick()
		}
	})
	onClick(byID("flight-reset"), func(js.Value) { a.newFlight() })
	onClick(byID("scenarios"), func(e js.Value) {
		value := e.Get("target").Call("getAttribute", "data-scenario")
		if value.IsNull() {
			return
		}
		if n, err := strconv.Atoi(value.String()); err == nil {
			a.sim.SetScenario(sim.Scenario(n))
			a.renderFly()
		}
	})
}

func (a *app) renderFly() {
	s := a.snap
	a.renderSource()

	armed := byID("vehicle-armed")
	switch {
	case !s.Connected:
		setText(armed, "No contact")
	case s.Armed:
		setText(armed, "Armed")
	default:
		setText(armed, "Disarmed")
	}
	setClass(armed, "armed", s.Connected && s.Armed)
	mode := s.Mode
	if !s.Connected {
		mode = "--"
	}
	setText(byID("vehicle-mode"), "Mode: "+mode)

	subsystems := model.Assess(s)
	a.health.render(subsystems)
	a.renderSummary(subsystems)
	a.renderInstruments()

	if a.live != nil {
		a.renderFlyAction()
		a.renderFaults()
	}

	if a.sim == nil {
		return
	}
	byID("flight-start").Set("disabled", a.sim.Flying() || len(a.plan.Waypoints) == 0)
	buttons := byID("scenarios").Call("querySelectorAll", "button")
	for i := 0; i < buttons.Length(); i++ {
		b := buttons.Index(i)
		setClass(b, "active", b.Call("getAttribute", "data-scenario").String() == strconv.Itoa(int(a.sim.Scenario())))
	}
}

// renderFlyAction enables the live "Start flight" button only when it can
// succeed, mirroring renderPlanAction's convention for "Upload to drone".
func (a *app) renderFlyAction() {
	button := byID("live-start")
	hint := ""
	switch {
	case !a.live.uploaded:
		hint = "Upload a plan first."
	case a.live.starting:
		hint = "Starting…"
	case !a.snap.Connected:
		hint = "No drone connected."
	case a.snap.Armed:
		hint = "Already armed."
	}
	button.Set("disabled", hint != "")
	button.Set("title", hint)
}

// renderFaults marks the fault the bridge says is injected, which is also right
// for a page opened after the fault was set from another one.
func (a *app) renderFaults() {
	active := a.snap.Fault
	if active == "" {
		active = "none" // nothing injected from here, so SITL is at its defaults
	}
	hint := ""
	switch {
	case a.live.injecting:
		hint = "Injecting…"
	case !a.snap.Connected:
		hint = "No drone connected."
	}
	buttons := byID("live-faults").Call("querySelectorAll", "button")
	for i := 0; i < buttons.Length(); i++ {
		b := buttons.Index(i)
		setClass(b, "active", a.snap.Connected && b.Call("getAttribute", "data-fault").String() == active)
		b.Set("disabled", hint != "")
	}
}

func (a *app) showFaultStatus(text string, isError bool) {
	status := byID("fault-status")
	setText(status, text)
	status.Set("hidden", text == "")
	setClass(status, "error", isError)
}

func (a *app) showLiveStatus(text string, isError bool) {
	status := byID("live-status")
	setText(status, text)
	status.Set("hidden", text == "")
	setClass(status, "error", isError)
}

// renderSource says where the data comes from. It stays visible so simulated
// data can't be mistaken for a real flight.
func (a *app) renderSource() {
	badge := byID("source-badge")
	var text, title string
	var ok bool
	switch {
	case a.live == nil:
		text = "Simulated data"
		title = "Telemetry is generated in the browser. No drone is connected."
	case !a.live.streaming:
		text = "Bridge offline"
		title = "The page can't reach its bridge."
	case !a.snap.Connected:
		text = "Live: waiting for drone"
		title = "Listening for MAVLink on UDP " + a.live.listen
	default:
		text, ok = "Live", true
		title = "Receiving MAVLink on UDP " + a.live.listen
	}
	setText(badge, text)
	badge.Set("title", title)
	setClass(badge, "live", ok)
}

func (a *app) renderInstruments() {
	s := a.snap
	show := func(id string, current bool, value string) {
		if !current {
			value = "--"
		}
		setText(byID(id), value)
	}
	show("i-alt", s.HasFlightData, fmt.Sprintf("%.0f m", s.AltM))
	show("i-speed", s.HasFlightData, fmt.Sprintf("%.1f m/s", s.SpeedMS))
	show("i-heading", s.HasHeading, fmt.Sprintf("%03.0f°", s.HeadingDeg))
	show("i-sats", s.HasGPS, strconv.Itoa(s.Satellites))
	driftEl := byID("i-drift")
	drift, showDrift := s.Drift()
	switch {
	case !s.HasPosition || !s.HasEstimate:
		setText(driftEl, "--")
	case showDrift:
		setText(driftEl, fmt.Sprintf("%.0f m", drift))
	default:
		setText(driftEl, "None")
	}
	setClass(driftEl, "alert", showDrift)
}

// renderSummary answers "is everything OK, and what is the drone doing about
// it?" before the operator has to read the individual cards.
func (a *app) renderSummary(subsystems []model.Subsystem) {
	worst := model.LevelHealthy
	var affected []string
	for _, rank := range []model.Level{model.LevelCritical, model.LevelCaution, model.LevelNoData} {
		for _, sub := range subsystems {
			if sub.Level == rank {
				affected = append(affected, sub.Name)
			}
		}
		if len(affected) > 0 {
			worst = rank
			break
		}
	}

	var title string
	switch worst {
	case model.LevelHealthy:
		title = "All systems healthy"
	case model.LevelNoData:
		title = "No data: " + strings.Join(affected, ", ")
	default:
		title = worst.String() + ": " + strings.Join(affected, ", ")
	}

	text := "On the ground, ready to fly."
	if a.snap.Armed {
		text = model.BehaviorText(a.snap.Behavior)
	}
	if !a.snap.Connected {
		worst = model.LevelNoData
		title = "No contact with the drone"
		text = "Everything shown is out of date."
		if a.live != nil {
			text = "Listening for telemetry on UDP " + a.live.listen + "."
			if !a.live.streaming {
				text = "The page can't reach its bridge."
			}
		}
	}

	summary := byID("summary")
	setClass(summary, "level-healthy", worst == model.LevelHealthy)
	setClass(summary, "level-caution", worst == model.LevelCaution)
	setClass(summary, "level-critical", worst == model.LevelCritical)
	setText(byID("summary-title"), title)
	setText(byID("summary-text"), text)
}

// showEvents adds any events not shown yet to the log, newest first.
func (a *app) showEvents(events []model.Event) {
	list := byID("events")
	for _, e := range events {
		if e.Seq <= a.shownEvent {
			continue
		}
		a.shownEvent = e.Seq
		li := newEl("li", "", "")
		seconds := e.AtMs / 1000
		li.Call("append", newEl("time", "", fmt.Sprintf("%02d:%02d", seconds/60, seconds%60)))
		li.Call("append", newEl("span", "", e.Text))
		list.Call("prepend", li)
		if list.Get("children").Length() > maxEventRows {
			list.Get("lastElementChild").Call("remove")
		}
	}
}

func (a *app) clearEvents() {
	a.shownEvent = 0
	clearChildren(byID("events"))
}

// healthPanel is one card per subsystem. Cards are built once and updated in
// place so the bars animate rather than flicker.
type healthPanel struct {
	cards []healthCard
}

type healthCard struct {
	root, name, level, score, fill, reason js.Value
}

func newHealthPanel() *healthPanel {
	return &healthPanel{}
}

func (p *healthPanel) render(subsystems []model.Subsystem) {
	for len(p.cards) < len(subsystems) {
		p.cards = append(p.cards, newHealthCard())
	}
	for i, card := range p.cards {
		// A card with no subsystem must not keep showing its last score.
		card.root.Set("hidden", i >= len(subsystems))
		if i < len(subsystems) {
			card.render(subsystems[i])
		}
	}
}

func newHealthCard() healthCard {
	c := healthCard{
		root:   newEl("div", "card", ""),
		name:   newEl("span", "card-name", ""),
		level:  newEl("span", "card-level", ""),
		score:  newEl("span", "card-score", ""),
		fill:   newEl("div", "card-fill", ""),
		reason: newEl("div", "card-reason", ""),
	}
	head := newEl("div", "card-head", "")
	head.Call("append", c.name, c.level, c.score)
	bar := newEl("div", "card-bar", "")
	bar.Call("append", c.fill)
	c.root.Call("append", head, bar, c.reason)
	byID("health").Call("append", c.root)
	return c
}

// levelIcons keep the states distinct without relying on color alone.
var levelIcons = map[model.Level]string{
	model.LevelHealthy:  "✓",
	model.LevelCaution:  "▲",
	model.LevelCritical: "✕",
	model.LevelNoData:   "?",
}

var levelClasses = map[model.Level]string{
	model.LevelHealthy:  "level-healthy",
	model.LevelCaution:  "level-caution",
	model.LevelCritical: "level-critical",
	model.LevelNoData:   "level-nodata",
}

func (c healthCard) render(sub model.Subsystem) {
	for level, class := range levelClasses {
		setClass(c.root, class, level == sub.Level)
	}
	setText(c.name, sub.Name)
	setText(c.level, levelIcons[sub.Level]+" "+sub.Level.String())
	setText(c.reason, sub.Reason)

	if sub.Level == model.LevelNoData {
		setText(c.score, "--")
		c.fill.Get("style").Set("width", "0")
		return
	}
	percent := math.Round(sub.Score * 100)
	setText(c.score, fmt.Sprintf("%.0f%%", percent))
	c.fill.Get("style").Set("width", fmt.Sprintf("%.0f%%", percent))
}
