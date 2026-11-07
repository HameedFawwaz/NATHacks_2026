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
	if !s.Connected {
		for _, id := range []string{"i-alt", "i-speed", "i-heading", "i-sats", "i-drift"} {
			setText(byID(id), "--")
			setClass(byID(id), "alert", false)
		}
		return
	}
	setText(byID("i-alt"), fmt.Sprintf("%.0f m", s.AltM))
	setText(byID("i-speed"), fmt.Sprintf("%.1f m/s", s.SpeedMS))
	setText(byID("i-heading"), fmt.Sprintf("%03.0f°", s.HeadingDeg))
	setText(byID("i-sats"), strconv.Itoa(s.Satellites))
	driftEl := byID("i-drift")
	drift, showDrift := s.Drift()
	switch {
	case !s.HasPosition || !s.HasEstimate:
		setText(driftEl, "n/a")
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
	for i, sub := range subsystems {
		p.cards[i].render(sub)
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
