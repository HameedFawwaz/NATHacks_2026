package sim

import (
	"math"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

const (
	spoofTurnDegPerS = 30.0 // what a copter at cruise speed might plausibly do
	spoofMinLegM     = 60.0
	spoofMaxLegM     = 200.0
	spoofMaxTurnDeg  = 120.0 // sharpest corner between legs, so it never doubles back
	spoofFirstTurn   = 40.0  // the first leg peels away gently from the real heading
	spoofMinApartM   = 50.0  // leg endpoints stay this far from the real drone
	spoofAreaMarginM = 40.0  // and this close to the mission area, to stay believable and on the map
	spoofLegTries    = 30
	spoofReturnMS    = 3 * model.CruiseSpeedMS // how fast the fix rejoins the truth once cleared
)

// spoofTrack is the false position a spoofer feeds the GPS. It flies a route of
// its own, in random straight legs joined by limited-rate turns at cruise speed,
// so that on its own it looks like a believable flight rather than a copy of the
// real path shifted sideways.
type spoofTrack struct {
	active     bool
	pos        model.LatLon
	headingDeg float64
	target     model.LatLon
}

func (s *Sim) updateSpoof(dt float64) {
	t := &s.spoof
	if s.scenario != ScenarioGNSSSpoof || !s.Flying() {
		if t.active {
			t.moveToward(s.pos, spoofReturnMS, dt)
			if model.DistanceM(t.pos, s.pos) < spoofReturnMS*dt {
				t.active = false
			}
		}
		return
	}

	if !t.active {
		*t = spoofTrack{active: true, pos: s.pos, headingDeg: s.headingDeg}
		s.pickLeg(spoofFirstTurn)
	} else if model.DistanceM(t.pos, t.target) < arriveRadiusM+model.CruiseSpeedMS*dt {
		s.pickLeg(spoofMaxTurnDeg)
	}

	// Limited-rate turn toward the leg's end, then forward along the new heading.
	east, north := model.ToLocal(t.pos, t.target)
	wanted := math.Atan2(east, north) * 180 / math.Pi
	diff := math.Remainder(wanted-t.headingDeg, 360)
	maxTurn := spoofTurnDegPerS * dt
	t.headingDeg = math.Mod(t.headingDeg+math.Max(-maxTurn, math.Min(maxTurn, diff))+360, 360)
	sin, cos := math.Sincos(t.headingDeg * math.Pi / 180)
	step := model.CruiseSpeedMS * dt
	t.pos = model.FromLocal(t.pos, sin*step, cos*step)
}

// area is a box around the mission, in metres east and north of home.
type area struct{ minE, maxE, minN, maxN float64 }

func missionArea(plan model.Plan) area {
	a := area{-spoofAreaMarginM, spoofAreaMarginM, -spoofAreaMarginM, spoofAreaMarginM}
	for _, w := range plan.Waypoints {
		e, n := model.ToLocal(plan.Home, w.LatLon)
		a.minE, a.maxE = min(a.minE, e-spoofAreaMarginM), max(a.maxE, e+spoofAreaMarginM)
		a.minN, a.maxN = min(a.minN, n-spoofAreaMarginM), max(a.maxN, n+spoofAreaMarginM)
	}
	return a
}

// pickLeg chooses where the false route goes next.
func (s *Sim) pickLeg(maxTurnDeg float64) {
	t, rng, home := &s.spoof, s.rng, s.plan.Home
	box := missionArea(s.plan)
	inBox := func(p model.LatLon) bool {
		e, n := model.ToLocal(home, p)
		return e >= box.minE && e <= box.maxE && n >= box.minN && n <= box.maxN
	}

	for range spoofLegTries {
		turn := (rng.Float64()*2 - 1) * maxTurnDeg
		leg := spoofMinLegM + rng.Float64()*(spoofMaxLegM-spoofMinLegM)
		sin, cos := math.Sincos((t.headingDeg + turn) * math.Pi / 180)
		candidate := model.FromLocal(t.pos, sin*leg, cos*leg)
		if inBox(candidate) && model.DistanceM(candidate, s.pos) >= spoofMinApartM {
			t.target = candidate
			return
		}
	}

	// No forward leg fits, so turn back to anywhere in the area.
	for range spoofLegTries {
		t.target = model.FromLocal(home,
			box.minE+rng.Float64()*(box.maxE-box.minE),
			box.minN+rng.Float64()*(box.maxN-box.minN))
		if d := model.DistanceM(t.pos, t.target); d >= spoofMinLegM && model.DistanceM(t.target, s.pos) >= spoofMinApartM {
			return
		}
	}
}

func (t *spoofTrack) moveToward(target model.LatLon, speedMS, dt float64) {
	east, north := model.ToLocal(t.pos, target)
	dist := math.Hypot(east, north)
	if dist == 0 {
		return
	}
	step := math.Min(dist, speedMS*dt)
	t.pos = model.FromLocal(t.pos, east/dist*step, north/dist*step)
	t.headingDeg = math.Mod(math.Atan2(east, north)*180/math.Pi+360, 360)
}
