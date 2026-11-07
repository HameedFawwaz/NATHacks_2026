package model

import (
	"math"
	"testing"
	"time"
)

func TestLocalRoundTrip(t *testing.T) {
	p := FromLocal(DefaultHome, 300, -120)
	east, north := ToLocal(DefaultHome, p)
	if math.Abs(east-300) > 0.001 || math.Abs(north+120) > 0.001 {
		t.Fatalf("round trip gave east=%f north=%f", east, north)
	}
	want := math.Hypot(300, 120)
	if got := DistanceM(DefaultHome, p); math.Abs(got-want) > 0.5 {
		t.Fatalf("DistanceM = %f, want about %f", got, want)
	}
}

func TestPlanLengthIncludesReturnHome(t *testing.T) {
	plan := Plan{Home: DefaultHome, Waypoints: []Waypoint{
		{LatLon: FromLocal(DefaultHome, 400, 0)},
	}}
	if got := plan.LengthM(); math.Abs(got-800) > 1 {
		t.Fatalf("LengthM = %f, want about 800", got)
	}
	if got := plan.EstimatedDuration(); got < 99*time.Second || got > 101*time.Second {
		t.Fatalf("EstimatedDuration = %s, want about 100s", got)
	}
	if got := (Plan{Home: DefaultHome}).LengthM(); got != 0 {
		t.Fatalf("empty plan LengthM = %f, want 0", got)
	}
}

func TestDriftOnlyShownAboveThreshold(t *testing.T) {
	s := Snapshot{Reported: DefaultHome, HasPosition: true, Estimated: FromLocal(DefaultHome, 5, 0), HasEstimate: true}
	if _, show := s.Drift(); show {
		t.Fatal("5 m of drift should not show")
	}
	s.Estimated = FromLocal(DefaultHome, 25, 0)
	if m, show := s.Drift(); !show || math.Abs(m-25) > 0.1 {
		t.Fatalf("Drift = %f, %v, want about 25, true", m, show)
	}
	s.HasEstimate = false
	if _, show := s.Drift(); show {
		t.Fatal("drift should not show without an estimate")
	}
	s.HasEstimate, s.HasPosition = true, false
	if _, show := s.Drift(); show {
		t.Fatal("drift should not show without a GPS position")
	}
}

func subsystemNamed(t *testing.T, s Snapshot, name string) Subsystem {
	t.Helper()
	for _, sub := range Assess(s) {
		if sub.Name == name {
			return sub
		}
	}
	t.Fatalf("no subsystem named %s", name)
	return Subsystem{}
}

func TestAssessLevels(t *testing.T) {
	const now = 10000
	cases := []struct {
		name  string
		set   func(*Snapshot, Reading)
		value float64
		want  Level
	}{
		{"GNSS", func(s *Snapshot, r Reading) { s.Nav = r }, 0.7, LevelHealthy},
		{"GNSS", func(s *Snapshot, r Reading) { s.Nav = r }, 0.69, LevelCaution},
		{"GNSS", func(s *Snapshot, r Reading) { s.Nav = r }, 0.4, LevelCaution},
		{"GNSS", func(s *Snapshot, r Reading) { s.Nav = r }, 0.39, LevelCritical},
		{"Battery", func(s *Snapshot, r Reading) { s.Battery = r }, 0.5, LevelHealthy},
		{"Battery", func(s *Snapshot, r Reading) { s.Battery = r }, 0.2, LevelCaution},
		{"Battery", func(s *Snapshot, r Reading) { s.Battery = r }, 0.1, LevelCritical},
	}
	for _, c := range cases {
		s := Snapshot{NowMs: now}
		c.set(&s, Reading{Value: c.value, AtMs: now})
		if got := subsystemNamed(t, s, c.name).Level; got != c.want {
			t.Errorf("%s at %.2f = %s, want %s", c.name, c.value, got, c.want)
		}
	}
}

func TestAssessNoData(t *testing.T) {
	s := Snapshot{NowMs: 10000}
	if got := subsystemNamed(t, s, "Link").Level; got != LevelNoData {
		t.Errorf("never received = %s, want no data", got)
	}

	s.Link = Reading{Value: 1, AtMs: 10000 - StaleAfterMs}
	if got := subsystemNamed(t, s, "Link").Level; got != LevelHealthy {
		t.Errorf("exactly at the stale limit = %s, want healthy", got)
	}

	// A good score that has stopped updating must not keep reading as healthy.
	s.Link = Reading{Value: 1, AtMs: 10000 - StaleAfterMs - 1}
	if got := subsystemNamed(t, s, "Link").Level; got != LevelNoData {
		t.Errorf("stale = %s, want no data", got)
	}
}

func TestHardwareRowOnlyOnceReported(t *testing.T) {
	s := Snapshot{NowMs: 1000}
	if got := len(Assess(s)); got != 5 {
		t.Fatalf("%d rows before TRUST_HW", got)
	}
	s.Hardware = Reading{Value: 0.1, AtMs: 1000}
	if got := subsystemNamed(t, s, "Hardware").Level; got != LevelCritical {
		t.Fatalf("Hardware = %s", got)
	}
}

func TestPlanCloneSharesNothing(t *testing.T) {
	p := DefaultPlan()
	c := p.Clone()
	c.Waypoints[0].AltM = 99
	if p.Waypoints[0].AltM == 99 {
		t.Fatal("clone shares waypoints with the original")
	}
}
