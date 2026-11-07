//go:build js && wasm

package main

import (
	"fmt"
	"slices"
	"strconv"
	"syscall/js"
	"time"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

const (
	minAltM = 1.0
	maxAltM = 120.0 // the usual legal ceiling for small drones
)

func (a *app) bindPlan() {
	onClick(byID("plan-fly"), func(js.Value) {
		a.newFlight()
		a.setTab(tabFly)
	})
	onClick(byID("plan-clear"), func(js.Value) {
		a.plan.Waypoints = nil
		a.planChanged()
	})

	// The rows are rebuilt on every change, so these listeners live on the list.
	list := byID("waypoints")
	onClick(list, func(e js.Value) {
		index := e.Get("target").Call("getAttribute", "data-remove")
		if index.IsNull() {
			return
		}
		if i, err := strconv.Atoi(index.String()); err == nil && i < len(a.plan.Waypoints) {
			a.plan.Waypoints = slices.Delete(a.plan.Waypoints, i, i+1)
			a.planChanged()
		}
	})
	on(list, "change", func(e js.Value) {
		target := e.Get("target")
		index := target.Call("getAttribute", "data-alt")
		if index.IsNull() {
			return
		}
		i, err := strconv.Atoi(index.String())
		if err != nil || i >= len(a.plan.Waypoints) {
			return
		}
		// An unusable entry is reverted rather than kept, by the re-render below.
		if alt, err := strconv.ParseFloat(target.Get("value").String(), 64); err == nil {
			a.plan.Waypoints[i].AltM = min(max(alt, minAltM), maxAltM)
		}
		a.planChanged()
	})
}

func (a *app) addWaypoint(p model.LatLon) {
	a.plan.Waypoints = append(a.plan.Waypoints, model.Waypoint{LatLon: p, AltM: model.DefaultAltM})
	a.planChanged()
}

func (a *app) planChanged() {
	a.savePlan()
	a.renderPlan()
}

func (a *app) renderPlan() {
	wps := a.plan.Waypoints
	setText(byID("p-count"), strconv.Itoa(len(wps)))
	setText(byID("p-distance"), formatDistance(a.plan.LengthM()))
	setText(byID("p-time"), formatDuration(a.plan.EstimatedDuration()))
	byID("plan-fly").Set("disabled", len(wps) == 0)
	byID("plan-clear").Set("disabled", len(wps) == 0)
	byID("waypoints-empty").Set("hidden", len(wps) > 0)

	list := byID("waypoints")
	clearChildren(list)
	prev := a.plan.Home
	for i, w := range wps {
		li := newEl("li", "", "")
		li.Call("append", newEl("span", "wp-index", strconv.Itoa(i+1)))
		li.Call("append", newEl("span", "wp-leg", "+"+formatDistance(model.DistanceM(prev, w.LatLon))))

		alt := newEl("label", "wp-alt", "Alt")
		input := newEl("input", "", "")
		input.Set("type", "number")
		input.Set("min", minAltM)
		input.Set("max", maxAltM)
		input.Set("value", fmt.Sprintf("%.0f", w.AltM))
		input.Call("setAttribute", "data-alt", strconv.Itoa(i))
		alt.Call("append", input, "m")
		li.Call("append", alt)

		remove := newEl("button", "wp-remove", "×")
		remove.Call("setAttribute", "data-remove", strconv.Itoa(i))
		remove.Call("setAttribute", "aria-label", fmt.Sprintf("Remove waypoint %d", i+1))
		li.Call("append", remove)

		list.Call("append", li)
		prev = w.LatLon
	}
}

func formatDistance(m float64) string {
	if m >= 1000 {
		return fmt.Sprintf("%.2f km", m/1000)
	}
	return fmt.Sprintf("%.0f m", m)
}

func formatDuration(d time.Duration) string {
	total := int(d.Round(time.Second).Seconds())
	return fmt.Sprintf("%d:%02d", total/60, total%60)
}
