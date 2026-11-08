package sim

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

const dt = 0.1

// run steps the sim for up to the given seconds, stopping early once done
// returns true, and reports whether it did.
func run(s *Sim, seconds float64, done func() bool) bool {
	for i := 0; i < int(seconds/dt); i++ {
		s.Step(dt)
		if done() {
			return true
		}
	}
	return false
}

func behaviorIs(s *Sim, b string) func() bool {
	return func() bool { return s.Snapshot().Behavior == b }
}

func level(s *Sim, name string) model.Level {
	for _, sub := range model.Assess(s.Snapshot()) {
		if sub.Name == name {
			return sub.Level
		}
	}
	return model.LevelNoData
}

func startedSim(t *testing.T) *Sim {
	t.Helper()
	s := New(model.DefaultPlan())
	s.Start()
	if !run(s, 20, func() bool { return s.Snapshot().AltM >= model.DefaultAltM }) {
		t.Fatal("never reached cruise altitude")
	}
	return s
}

func TestNormalFlightCompletesAndLands(t *testing.T) {
	s := New(model.DefaultPlan())
	if snap := s.Snapshot(); snap.Armed {
		t.Fatal("should start disarmed")
	}
	s.Start()

	landed := run(s, 600, func() bool {
		snap := s.Snapshot()
		if _, show := snap.Drift(); show {
			t.Fatal("drift shown during a normal flight")
		}
		return !s.Flying()
	})
	if !landed {
		t.Fatal("flight never finished")
	}
	snap := s.Snapshot()
	if snap.AltM != 0 || snap.Armed {
		t.Fatalf("after landing: alt=%f armed=%v", snap.AltM, snap.Armed)
	}
	if d := model.DistanceM(snap.Reported, model.DefaultHome); d > 5 {
		t.Fatalf("landed %f m from home", d)
	}
}

func TestStartWithoutWaypointsDoesNothing(t *testing.T) {
	s := New(model.Plan{Home: model.DefaultHome})
	s.Start()
	if s.Flying() {
		t.Fatal("started with an empty plan")
	}
}

func TestGNSSSpoofShowsDriftAndFailsOver(t *testing.T) {
	s := startedSim(t)
	s.SetScenario(ScenarioGNSSSpoof)

	if !run(s, 30, func() bool { _, show := s.Snapshot().Drift(); return show }) {
		t.Fatal("drift never became visible")
	}
	if !run(s, 30, behaviorIs(s, model.BehaviorGNSSFailover)) {
		t.Fatal("never failed over")
	}
	if got := level(s, "GNSS"); got != model.LevelCritical {
		t.Fatalf("GNSS = %s, want critical", got)
	}

	s.SetScenario(ScenarioNone)
	if !run(s, 60, behaviorIs(s, model.BehaviorNominal)) {
		t.Fatal("never recovered after the spoof stopped")
	}
	if !run(s, 30, func() bool { _, show := s.Snapshot().Drift(); return !show }) {
		t.Fatal("drift never cleared")
	}
}

func TestLinkJamHoldsThenReturnsHome(t *testing.T) {
	s := startedSim(t)
	s.SetScenario(ScenarioLinkJam)

	if !run(s, 10, behaviorIs(s, model.BehaviorLinkHold)) {
		t.Fatal("never held")
	}
	held := s.Snapshot().Reported
	if !run(s, linkHoldS+1, behaviorIs(s, model.BehaviorLinkReturn)) {
		t.Fatal("never escalated to return")
	}
	if d := model.DistanceM(held, s.Snapshot().Reported); d > 2 {
		t.Fatalf("moved %f m while holding", d)
	}
	if !run(s, 300, func() bool { return !s.Flying() }) {
		t.Fatal("never landed")
	}
	if d := model.DistanceM(s.Snapshot().Reported, model.DefaultHome); d > 5 {
		t.Fatalf("landed %f m from home", d)
	}
}

func TestMotorFaultLandsAndStaysLatched(t *testing.T) {
	s := startedSim(t)
	s.SetScenario(ScenarioMotorFault)

	if !run(s, 20, behaviorIs(s, model.BehaviorSafeLand)) {
		t.Fatal("never entered safe land")
	}

	// As in core, safe land holds until disarmed even if the fault clears.
	s.SetScenario(ScenarioNone)
	stillLanding := true
	run(s, 60, func() bool {
		if s.Flying() && s.Snapshot().Behavior != model.BehaviorSafeLand {
			stillLanding = false
		}
		return !s.Flying()
	})
	if !stillLanding {
		t.Fatal("left safe land before disarming")
	}
	if s.Flying() {
		t.Fatal("never landed")
	}
}

func TestSpoofAndJamTogetherLand(t *testing.T) {
	s := startedSim(t)
	s.SetScenario(ScenarioGNSSSpoof)
	if !run(s, 30, behaviorIs(s, model.BehaviorGNSSFailover)) {
		t.Fatal("never failed over")
	}
	// Scenarios are exclusive, so nav trust starts recovering once the jam starts.
	// From near zero it stays untrusted for long enough to overlap with the jam.
	if !run(s, 30, func() bool { return s.navTrust < 0.05 }) {
		t.Fatal("nav trust never bottomed out")
	}
	s.SetScenario(ScenarioLinkJam)
	if !run(s, 30, behaviorIs(s, model.BehaviorSafeLand)) {
		t.Fatal("never entered safe land")
	}
}

func TestEventsKeepIncreasingSeqPastTheCap(t *testing.T) {
	s := New(model.DefaultPlan())
	for i := 0; i < maxEvents+10; i++ {
		s.logf("event %d", i)
	}
	events := s.Events()
	if len(events) != maxEvents || events[len(events)-1].Seq != maxEvents+10 {
		t.Fatalf("%d events, last %+v", len(events), events[len(events)-1])
	}
}

func seededSim(t *testing.T, seed uint64) *Sim {
	t.Helper()
	s := startedSim(t)
	s.rng = rand.New(rand.NewPCG(seed, 1))
	return s
}

// spoofedFlight runs a spoof for the given time and returns the reported track.
func spoofedFlight(t *testing.T, s *Sim, seconds float64) (reported, truth []model.LatLon) {
	t.Helper()
	s.SetScenario(ScenarioGNSSSpoof)
	for i := 0; i < int(seconds/dt) && s.Flying(); i++ {
		s.Step(dt)
		reported = append(reported, s.Snapshot().Reported)
		truth = append(truth, s.pos)
	}
	return reported, truth
}

func TestSpoofIsNotTheRealPathShifted(t *testing.T) {
	for seed := range uint64(20) {
		s := seededSim(t, seed)
		reported, truth := spoofedFlight(t, s, 60)

		// A shifted copy would keep the same offset throughout.
		var offsets [][2]float64
		for i := range reported {
			e, n := model.ToLocal(truth[i], reported[i])
			offsets = append(offsets, [2]float64{e, n})
		}
		spread := 0.0
		for _, o := range offsets {
			spread = math.Max(spread, math.Hypot(o[0]-offsets[len(offsets)-1][0], o[1]-offsets[len(offsets)-1][1]))
		}
		if spread < 50 {
			t.Errorf("seed %d: offset from the real path only varied by %.0f m", seed, spread)
		}
	}
}

func TestSpoofMovesLikeADrone(t *testing.T) {
	for seed := range uint64(20) {
		s := seededSim(t, seed)
		s.SetScenario(ScenarioGNSSSpoof)
		var prevPos model.LatLon
		prevHeading := math.NaN()
		apart := 0.0
		for i := 0; i < 900 && s.Flying(); i++ {
			s.Step(dt)
			snap := s.Snapshot()
			if !math.IsNaN(prevHeading) {
				if d := model.DistanceM(prevPos, snap.Reported); d > model.CruiseSpeedMS*dt*1.01 {
					t.Fatalf("seed %d: jumped %.2f m in one step", seed, d)
				}
				if d := math.Abs(math.Remainder(snap.HeadingDeg-prevHeading, 360)); d > spoofTurnDegPerS*dt*1.01 {
					t.Fatalf("seed %d: turned %.1f deg in one step", seed, d)
				}
			}
			prevPos, prevHeading = snap.Reported, snap.HeadingDeg
			apart = math.Max(apart, model.DistanceM(snap.Reported, s.pos))
		}
		e, n := model.ToLocal(s.plan.Home, s.Snapshot().Reported)
		box := missionArea(s.plan)
		const overshoot = 40 // room for a limited-rate turn at the edge
		if e < box.minE-overshoot || e > box.maxE+overshoot || n < box.minN-overshoot || n > box.maxN+overshoot {
			t.Errorf("seed %d: left the mission area at %.0f, %.0f", seed, e, n)
		}
		if apart < spoofMinApartM {
			t.Errorf("seed %d: only got %.0f m from the real position", seed, apart)
		}
	}
}

func TestSpoofOnATinyPlanStillMoves(t *testing.T) {
	plan := model.Plan{Home: model.DefaultHome, Waypoints: []model.Waypoint{
		{LatLon: model.FromLocal(model.DefaultHome, 30, 0), AltM: 5},
	}}
	s := New(plan)
	s.rng = rand.New(rand.NewPCG(7, 1))
	s.Start()
	s.SetScenario(ScenarioGNSSSpoof)
	for range 200 {
		s.Step(dt)
		if !s.Flying() {
			return
		}
		if snap := s.Snapshot(); math.IsNaN(snap.Reported.Lat) || math.IsNaN(snap.HeadingDeg) {
			t.Fatal("NaN position")
		}
	}
}

func TestSpoofRouteDiffersBetweenFlights(t *testing.T) {
	a, _ := spoofedFlight(t, seededSim(t, 1), 40)
	b, _ := spoofedFlight(t, seededSim(t, 2), 40)
	if d := model.DistanceM(a[len(a)-1], b[len(b)-1]); d < 10 {
		t.Fatalf("two seeds ended only %.1f m apart", d)
	}
}

func TestSpoofRejoinsTruthWhenCleared(t *testing.T) {
	s := seededSim(t, 3)
	spoofedFlight(t, s, 30)
	s.SetScenario(ScenarioNone)
	if !run(s, 60, func() bool { return !s.spoof.active }) {
		t.Fatal("never rejoined")
	}
	if s.Snapshot().Reported != s.pos {
		t.Fatal("still reporting a false position")
	}
}

func TestSpoofWaitsForFlight(t *testing.T) {
	s := New(model.DefaultPlan())
	s.SetScenario(ScenarioGNSSSpoof)
	run(s, 5, func() bool { return false })
	if s.spoof.active || s.Snapshot().Reported != s.pos {
		t.Fatal("spoofed a drone that hasn't taken off")
	}
}
