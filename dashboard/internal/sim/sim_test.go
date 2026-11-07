package sim

import (
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
	// Scenarios are exclusive, so the spoof's offset decays once the jam starts;
	// it still takes long enough for both scores to be low at the same time.
	s.spoofOffsetM = spoofMaxM
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
