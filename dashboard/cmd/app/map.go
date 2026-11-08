//go:build js && wasm

package main

import (
	"fmt"
	"math"
	"strconv"
	"syscall/js"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

const (
	colorPlan      = "#23baff"
	colorReported  = "#ffffff"
	colorEstimated = "#3ecf8e"
	colorDrift     = "#ff5c5c"
	colorGrid      = "#1d2631"
	colorGridMajor = "#2c3743"
	colorText      = "#94a3b2"

	waypointRadiusPx = 11.0
	hitRadiusPx      = 16.0
	dragThresholdPx  = 4.0
	minScale         = 0.02 // pixels per metre
	maxScale         = 20.0
	fitMarginPx      = 90.0
)

// mapView draws onto a canvas with a flat projection and optional basemap tiles.
// Positions are metres east and north of the plan's home; the grid works offline.
type mapView struct {
	app    *app
	canvas js.Value
	ctx    js.Value

	width, height float64 // CSS pixels
	dpr           float64

	centerEast, centerNorth float64
	scale                   float64 // pixels per metre

	// The current pointer gesture.
	pressed        bool
	dragging       bool // moved past dragThresholdPx
	dragWaypoint   int  // -1 = panning the map
	pressX, pressY float64
	lastX, lastY   float64
}

func newMapView(a *app) *mapView {
	canvas := byID("map")
	m := &mapView{
		app:          a,
		canvas:       canvas,
		ctx:          canvas.Call("getContext", "2d"),
		scale:        1,
		dragWaypoint: -1,
	}
	on(canvas, "pointerdown", m.onPointerDown)
	on(canvas, "pointermove", m.onPointerMove)
	on(canvas, "pointerup", m.onPointerUp)
	on(canvas, "pointercancel", func(js.Value) { m.pressed = false })

	// preventDefault on wheel needs a non-passive listener, which on() can't ask for.
	canvas.Call("addEventListener", "wheel", js.FuncOf(func(_ js.Value, args []js.Value) any {
		m.onWheel(args[0])
		return nil
	}), map[string]any{"passive": false})

	on(js.Global().Get("window"), "resize", func(js.Value) { m.resize() })
	// Banners and responsive cockpit panels can resize the map without a window
	// resize. Keep pointer coordinates and the canvas backing size in sync.
	if observer := js.Global().Get("ResizeObserver"); !observer.IsUndefined() {
		observer.New(js.FuncOf(func(js.Value, []js.Value) any {
			m.resize()
			return nil
		})).Call("observe", canvas)
	}
	m.resize()
	return m
}

func (m *mapView) resize() {
	rect := m.canvas.Call("getBoundingClientRect")
	m.width, m.height = rect.Get("width").Float(), rect.Get("height").Float()
	m.dpr = js.Global().Get("devicePixelRatio").Float()
	m.canvas.Set("width", int(m.width*m.dpr))
	m.canvas.Set("height", int(m.height*m.dpr))
}

// activePlan is the plan being edited on the Plan tab, or the route on the Fly tab.
func (m *mapView) activePlan() model.Plan {
	if m.app.tab == tabPlan {
		return m.app.plan
	}
	return m.app.flownPlan
}

// origin is fixed to the home location so a view survives edits to the waypoints.
func (m *mapView) origin() model.LatLon { return m.app.plan.Home }

func (m *mapView) toScreen(p model.LatLon) (x, y float64) {
	east, north := model.ToLocal(m.origin(), p)
	return m.width/2 + (east-m.centerEast)*m.scale, m.height/2 - (north-m.centerNorth)*m.scale
}

func (m *mapView) fromScreen(x, y float64) model.LatLon {
	east := m.centerEast + (x-m.width/2)/m.scale
	north := m.centerNorth - (y-m.height/2)/m.scale
	return model.FromLocal(m.origin(), east, north)
}

// fit frames home and the whole route, and on the Fly tab where the drone has been.
func (m *mapView) fit() {
	plan := m.activePlan()
	minE, maxE, minN, maxN := -50.0, 50.0, -50.0, 50.0 // keeps a lone home point from over-zooming
	include := func(p model.LatLon) {
		e, n := model.ToLocal(m.origin(), p)
		minE, maxE = min(minE, e), max(maxE, e)
		minN, maxN = min(minN, n), max(maxN, n)
	}
	for _, w := range plan.Waypoints {
		include(w.LatLon)
	}
	if m.app.tab == tabFly {
		for _, p := range m.app.reportedTrail {
			include(p)
		}
		for _, p := range m.app.estimatedTrail {
			include(p)
		}
	}
	m.centerEast, m.centerNorth = (minE+maxE)/2, (minN+maxN)/2
	availW := max(m.width-2*fitMarginPx, 100)
	availH := max(m.height-2*fitMarginPx, 100)
	m.scale = clampScale(min(availW/(maxE-minE), availH/(maxN-minN)))
}

func clampScale(s float64) float64 { return max(minScale, min(maxScale, s)) }

func pointerPos(e js.Value) (x, y float64) {
	return e.Get("offsetX").Float(), e.Get("offsetY").Float()
}

func (m *mapView) waypointAt(x, y float64) int {
	if m.app.tab != tabPlan {
		return -1
	}
	wps := m.app.plan.Waypoints
	for i := len(wps) - 1; i >= 0; i-- { // the last drawn is on top
		wx, wy := m.toScreen(wps[i].LatLon)
		if math.Hypot(wx-x, wy-y) <= hitRadiusPx {
			return i
		}
	}
	return -1
}

func (m *mapView) onPointerDown(e js.Value) {
	x, y := pointerPos(e)
	m.pressed, m.dragging = true, false
	m.pressX, m.pressY, m.lastX, m.lastY = x, y, x, y
	m.dragWaypoint = m.waypointAt(x, y)
	m.canvas.Call("setPointerCapture", e.Get("pointerId"))
}

func (m *mapView) onPointerMove(e js.Value) {
	x, y := pointerPos(e)
	if !m.pressed {
		cursor := "grab"
		if m.app.tab == tabPlan {
			cursor = "crosshair"
			if m.waypointAt(x, y) >= 0 {
				cursor = "move"
			}
		}
		m.canvas.Get("style").Set("cursor", cursor)
		return
	}
	if !m.dragging && math.Hypot(x-m.pressX, y-m.pressY) < dragThresholdPx {
		return
	}
	m.dragging = true
	if m.dragWaypoint >= 0 {
		m.app.plan.Waypoints[m.dragWaypoint].LatLon = m.fromScreen(x, y)
	} else {
		m.centerEast -= (x - m.lastX) / m.scale
		m.centerNorth += (y - m.lastY) / m.scale
	}
	m.lastX, m.lastY = x, y
}

func (m *mapView) onPointerUp(e js.Value) {
	if !m.pressed {
		return
	}
	m.pressed = false
	switch {
	case m.dragging && m.dragWaypoint >= 0:
		m.app.planChanged()
	case !m.dragging && m.app.tab == tabPlan && e.Get("shiftKey").Bool():
		// Shift-click only removes an existing waypoint; empty space and home
		// are left alone. A Shift-drag still moves the point normally.
		if e.Get("button").Int() == 0 {
			m.app.removeWaypoint(m.dragWaypoint)
		}
	case !m.dragging && m.dragWaypoint < 0 && m.app.tab == tabPlan:
		x, y := pointerPos(e)
		m.app.addWaypoint(m.fromScreen(x, y))
	}
}

func (m *mapView) onWheel(e js.Value) {
	e.Call("preventDefault")
	x, y := pointerPos(e)
	before := m.fromScreen(x, y)
	m.scale = clampScale(m.scale * math.Exp(-e.Get("deltaY").Float()*0.0015))

	// Keep the point under the cursor in place.
	wantE, wantN := model.ToLocal(m.origin(), before)
	after := m.fromScreen(x, y)
	gotE, gotN := model.ToLocal(m.origin(), after)
	m.centerEast += wantE - gotE
	m.centerNorth += wantN - gotN
}

func (m *mapView) startDrawing() {
	var frame js.Func
	var lastFrame float64
	frame = js.FuncOf(func(_ js.Value, args []js.Value) any {
		// Telemetry arrives at 10 Hz. 30 fps keeps interaction smooth without
		// redrawing the entire map at 60/120 Hz on high-refresh displays.
		now := args[0].Float()
		const interval = 1000.0 / 30
		if now-lastFrame >= interval {
			lastFrame = now - math.Mod(now-lastFrame, interval)
			m.draw()
		}
		js.Global().Call("requestAnimationFrame", frame)
		return nil
	})
	js.Global().Call("requestAnimationFrame", frame)
}

func (m *mapView) draw() {
	c := m.ctx
	c.Call("setTransform", m.dpr, 0, 0, m.dpr, 0, 0)
	c.Set("fillStyle", "#0e1319")
	c.Call("fillRect", 0, 0, m.width, m.height)

	m.drawGrid()
	// The optional presentation layer uses the same local-metre projection. Keep
	// the grid underneath so missing/offline tiles don't affect flight planning.
	if tiles := js.Global().Get("TrustIssuesMap"); !tiles.IsUndefined() && !tiles.IsNull() {
		origin := m.origin()
		tiles.Call("draw", c, m.width, m.height, origin.Lat, origin.Lon, m.centerEast, m.centerNorth, m.scale)
	}
	m.drawRoute(m.activePlan())
	if m.app.tab == tabFly {
		m.drawFlight()
	}
	m.drawScaleBar()
}

// gridStep returns a round number of metres that is at least 60 pixels wide.
func (m *mapView) gridStep() float64 {
	target := 60 / m.scale
	magnitude := math.Pow(10, math.Floor(math.Log10(target)))
	for _, multiple := range []float64{1, 2, 5, 10} {
		if multiple*magnitude >= target {
			return multiple * magnitude
		}
	}
	return 10 * magnitude
}

func (m *mapView) drawGrid() {
	c := m.ctx
	step := m.gridStep()
	major := step * 5
	c.Set("lineWidth", 1)

	originX := m.width/2 - m.centerEast*m.scale
	originY := m.height/2 + m.centerNorth*m.scale

	line := func(position, end float64, vertical, isMajor bool) {
		c.Set("strokeStyle", colorGrid)
		if isMajor {
			c.Set("strokeStyle", colorGridMajor)
		}
		c.Call("beginPath")
		if vertical {
			c.Call("moveTo", position, 0)
			c.Call("lineTo", position, end)
		} else {
			c.Call("moveTo", 0, position)
			c.Call("lineTo", end, position)
		}
		c.Call("stroke")
	}

	isMajor := func(metres float64) bool {
		return math.Abs(math.Mod(metres, major)) < step/2
	}
	firstE := math.Floor((m.centerEast-m.width/2/m.scale)/step) * step
	for e := firstE; originX+e*m.scale <= m.width; e += step {
		line(originX+e*m.scale, m.height, true, isMajor(e))
	}
	firstN := math.Floor((m.centerNorth-m.height/2/m.scale)/step) * step
	for n := firstN; originY-n*m.scale >= 0; n += step { // y shrinks as north grows
		line(originY-n*m.scale, m.width, false, isMajor(n))
	}
}

func (m *mapView) polyline(points []model.LatLon) {
	if len(points) < 2 {
		return
	}
	c := m.ctx
	c.Call("beginPath")
	for i, p := range points {
		x, y := m.toScreen(p)
		if i == 0 {
			c.Call("moveTo", x, y)
		} else {
			c.Call("lineTo", x, y)
		}
	}
	c.Call("stroke")
}

func (m *mapView) drawRoute(plan model.Plan) {
	c := m.ctx
	route := []model.LatLon{plan.Home}
	for _, w := range plan.Waypoints {
		route = append(route, w.LatLon)
	}
	if len(plan.Waypoints) > 0 {
		route = append(route, plan.Home)
	}
	c.Set("strokeStyle", colorPlan)
	c.Set("lineWidth", 2)
	c.Call("setLineDash", []any{8, 6})
	m.polyline(route)
	c.Call("setLineDash", []any{})

	c.Set("textAlign", "center")
	c.Set("textBaseline", "middle")
	c.Set("font", "700 12px system-ui, sans-serif")

	x, y := m.toScreen(plan.Home)
	c.Set("fillStyle", "#0e1319")
	c.Set("strokeStyle", colorPlan)
	c.Set("lineWidth", 2)
	c.Call("beginPath")
	c.Call("rect", x-waypointRadiusPx, y-waypointRadiusPx, 2*waypointRadiusPx, 2*waypointRadiusPx)
	c.Call("fill")
	c.Call("stroke")
	c.Set("fillStyle", colorPlan)
	c.Call("fillText", "H", x, y+1)

	for i, w := range plan.Waypoints {
		x, y := m.toScreen(w.LatLon)
		c.Set("fillStyle", colorPlan)
		c.Call("beginPath")
		c.Call("arc", x, y, waypointRadiusPx, 0, 2*math.Pi)
		c.Call("fill")
		c.Set("fillStyle", "#04121f")
		c.Call("fillText", strconv.Itoa(i+1), x, y+1)
	}
}

func (m *mapView) drawFlight() {
	c := m.ctx
	s := m.app.snap

	// Dimmed once contact is lost: it's where the drone was, not where it is.
	c.Call("save")
	defer c.Call("restore")
	if !s.Connected {
		c.Set("globalAlpha", 0.35)
	}

	c.Set("lineWidth", 2)
	c.Set("strokeStyle", colorEstimated)
	m.polyline(m.app.estimatedTrail)
	c.Set("strokeStyle", colorReported)
	m.polyline(m.app.reportedTrail)

	if s.HasEstimate {
		ex, ey := m.toScreen(s.Estimated)
		if meters, show := s.Drift(); show {
			rx, ry := m.toScreen(s.Reported)
			m.drawDrift(rx, ry, ex, ey, meters)
		}
		// A diamond, so it stays distinct from the GPS arrow when they overlap.
		c.Set("fillStyle", colorEstimated)
		c.Call("beginPath")
		c.Call("moveTo", ex, ey-8)
		c.Call("lineTo", ex+8, ey)
		c.Call("lineTo", ex, ey+8)
		c.Call("lineTo", ex-8, ey)
		c.Call("closePath")
		c.Call("fill")
	}

	if !s.HasPosition {
		return
	}

	// The GPS position, as an arrow pointing along the heading, or a dot without one.
	rx, ry := m.toScreen(s.Reported)
	if !s.HasHeading {
		c.Set("fillStyle", colorReported)
		c.Call("beginPath")
		c.Call("arc", rx, ry, 7, 0, 2*math.Pi)
		c.Call("fill")
		return
	}
	heading := s.HeadingDeg * math.Pi / 180
	c.Call("save")
	c.Call("translate", rx, ry)
	c.Call("rotate", heading)
	c.Set("fillStyle", colorReported)
	c.Set("strokeStyle", "#0e1319")
	c.Set("lineWidth", 1.5)
	c.Call("beginPath")
	c.Call("moveTo", 0, -13)
	c.Call("lineTo", 9, 10)
	c.Call("lineTo", 0, 5)
	c.Call("lineTo", -9, 10)
	c.Call("closePath")
	c.Call("fill")
	c.Call("stroke")
	c.Call("restore")
}

func (m *mapView) drawDrift(rx, ry, ex, ey, meters float64) {
	c := m.ctx
	c.Set("strokeStyle", colorDrift)
	c.Set("lineWidth", 3)
	c.Call("beginPath")
	c.Call("moveTo", ex, ey)
	c.Call("lineTo", rx, ry)
	c.Call("stroke")

	label := fmt.Sprintf("%.0f m apart", meters)
	c.Set("font", "700 12px system-ui, sans-serif")
	width := c.Call("measureText", label).Get("width").Float()
	// Kept on screen: the GPS position can be spoofed far outside the view.
	x := max(width/2+8, min(m.width-width/2-8, (rx+ex)/2))
	y := max(60, min(m.height-20, (ry+ey)/2))
	c.Set("fillStyle", colorDrift)
	c.Call("beginPath")
	c.Call("roundRect", x-width/2-6, y-10, width+12, 20, 4)
	c.Call("fill")
	c.Set("fillStyle", "#1a0505")
	c.Set("textAlign", "center")
	c.Set("textBaseline", "middle")
	c.Call("fillText", label, x, y+1)
}

func (m *mapView) drawScaleBar() {
	c := m.ctx
	step := m.gridStep()
	barPx := step * m.scale
	x, y := 14.0, m.height-18

	c.Set("strokeStyle", colorText)
	c.Set("lineWidth", 2)
	c.Call("beginPath")
	c.Call("moveTo", x, y-4)
	c.Call("lineTo", x, y)
	c.Call("lineTo", x+barPx, y)
	c.Call("lineTo", x+barPx, y-4)
	c.Call("stroke")

	c.Set("fillStyle", colorText)
	c.Set("font", "12px system-ui, sans-serif")
	c.Set("textAlign", "left")
	c.Set("textBaseline", "bottom")
	c.Call("fillText", formatDistance(step), x, y-5)
}
