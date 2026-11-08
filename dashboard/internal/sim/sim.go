// Package sim generates fake telemetry so the interface can be used without a
// vehicle. Its trust scores and behaviors imitate core's so that each dashboard
// state can be shown; they say nothing about how the real detectors perform.
package sim

import (
	"fmt"
	"math"
	"math/rand/v2"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

type Scenario int

const (
	ScenarioNone Scenario = iota
	ScenarioGNSSSpoof
	ScenarioLinkJam
	ScenarioMotorFault
)

const (
	climbRateMS     = 3.0
	descentRateMS   = 2.0
	arriveRadiusM   = 3.0
	spoofTrustSpanM = 40.0 // distance from the truth at which nav trust reaches 0
	linkDecayPerS   = 0.25
	motorDecayPerS  = 0.08
	recoverPerS     = 0.1
	batteryPerS     = 0.0015
	linkHoldS       = 30.0 // core's link_hold_ms
	maxEvents       = 50
)

type phase int

const (
	phaseIdle phase = iota
	phaseMission
	phaseReturning
	phaseLanding
)

// Sim is a single simulated flight. Create it with New.
type Sim struct {
	plan     model.Plan
	scenario Scenario

	nowMs      int64
	phase      phase
	target     int
	pos        model.LatLon // where the drone really is
	altM       float64
	speedMS    float64
	headingDeg float64

	rng         *rand.Rand // chooses the spoofed route; differs every flight
	spoof       spoofTrack
	navTrust    float64
	linkTrust   float64
	motorHealth float64
	battery     float64

	behavior    string
	holdSeconds float64
	landLatched bool

	events   []model.Event
	eventSeq int
}

// New returns a drone sitting at the plan's home, ready to start.
func New(plan model.Plan) *Sim {
	return &Sim{
		plan:        plan,
		rng:         rand.New(rand.NewPCG(rand.Uint64(), rand.Uint64())),
		nowMs:       1,
		pos:         plan.Home,
		navTrust:    1,
		linkTrust:   1,
		motorHealth: 1,
		battery:     1,
		behavior:    model.BehaviorNominal,
	}
}

func (s *Sim) Plan() model.Plan      { return s.plan }
func (s *Sim) Scenario() Scenario    { return s.scenario }
func (s *Sim) Events() []model.Event { return s.events }

// Flying reports whether a flight is under way.
func (s *Sim) Flying() bool { return s.phase != phaseIdle }

// Start begins the flight. It does nothing without a waypoint or mid-flight.
func (s *Sim) Start() {
	if s.Flying() || len(s.plan.Waypoints) == 0 {
		return
	}
	s.phase = phaseMission
	s.target = 0
	s.landLatched = false
	s.logf("Takeoff. Starting the planned route.")
}

// SetScenario changes the injected problem. ScenarioNone clears it, after which
// the scores recover gradually.
func (s *Sim) SetScenario(c Scenario) {
	if c == s.scenario {
		return
	}
	s.scenario = c
	switch c {
	case ScenarioNone:
		s.logf("Demo: problem cleared.")
	case ScenarioGNSSSpoof:
		s.logf("Demo: GPS spoofing started.")
	case ScenarioLinkJam:
		s.logf("Demo: link jamming started.")
	case ScenarioMotorFault:
		s.logf("Demo: motor fault started.")
	}
}

// Step advances the simulation by dt seconds.
func (s *Sim) Step(dt float64) {
	s.nowMs += int64(dt * 1000)
	s.updateSpoof(dt)
	s.updateHealth(dt)
	if !s.Flying() {
		s.speedMS = 0
		return
	}
	s.updateBehavior(dt)
	s.move(dt)
	s.battery = math.Max(0, s.battery-batteryPerS*dt)
}

func (s *Sim) updateHealth(dt float64) {
	// Drops at once on evidence and recovers slowly, as core's nav trust does, so
	// the false route crossing the real one doesn't make it look trustworthy again.
	apart := 0.0
	if s.spoof.active {
		apart = model.DistanceM(s.spoof.pos, s.pos)
	}
	evidence := clamp01(1 - apart/spoofTrustSpanM)
	s.navTrust = math.Min(evidence, s.navTrust+recoverPerS*dt)

	if s.scenario == ScenarioLinkJam {
		s.linkTrust = clamp01(s.linkTrust - linkDecayPerS*dt)
	} else {
		s.linkTrust = clamp01(s.linkTrust + recoverPerS*dt)
	}

	if s.scenario == ScenarioMotorFault {
		s.motorHealth = clamp01(s.motorHealth - motorDecayPerS*dt)
	} else {
		s.motorHealth = clamp01(s.motorHealth + recoverPerS*dt)
	}
}

// updateBehavior follows the table at the top of core's autonomy.h.
func (s *Sim) updateBehavior(dt float64) {
	navBad := s.navTrust < model.TrustCritical
	linkBad := s.linkTrust < model.TrustCritical
	motorBad := s.motorHealth < model.TrustCritical

	if !linkBad {
		s.holdSeconds = 0
	}

	next := model.BehaviorNominal
	switch {
	case s.landLatched || motorBad || (navBad && linkBad):
		s.landLatched = true
		next = model.BehaviorSafeLand
	case linkBad:
		s.holdSeconds += dt
		next = model.BehaviorLinkHold
		if s.holdSeconds >= linkHoldS {
			next = model.BehaviorLinkReturn
		}
	case navBad:
		next = model.BehaviorGNSSFailover
	}

	if next != s.behavior {
		s.behavior = next
		s.logf("%s", model.BehaviorText(next))
	}
}

func (s *Sim) move(dt float64) {
	switch s.behavior {
	case model.BehaviorSafeLand:
		s.speedMS = 0
		s.descend(dt)
		return
	case model.BehaviorLinkHold:
		s.speedMS = 0
		return
	case model.BehaviorLinkReturn:
		if s.flyTo(s.plan.Home, dt) {
			s.speedMS = 0
			s.descend(dt)
		}
		return
	}

	switch s.phase {
	case phaseMission:
		wp := s.plan.Waypoints[s.target]
		s.climbTo(wp.AltM, dt)
		if s.flyTo(wp.LatLon, dt) {
			s.logf("Reached waypoint %d.", s.target+1)
			s.target++
			if s.target == len(s.plan.Waypoints) {
				s.phase = phaseReturning
				s.logf("Route finished. Returning home.")
			}
		}
	case phaseReturning:
		s.flyHome(dt)
	case phaseLanding:
		s.speedMS = 0
		s.descend(dt)
	}
}

func (s *Sim) flyHome(dt float64) {
	if s.flyTo(s.plan.Home, dt) {
		s.phase = phaseLanding
	}
}

func (s *Sim) climbTo(altM, dt float64) {
	if s.altM < altM {
		s.altM = math.Min(altM, s.altM+climbRateMS*dt)
	} else {
		s.altM = math.Max(altM, s.altM-descentRateMS*dt)
	}
}

func (s *Sim) descend(dt float64) {
	s.altM -= descentRateMS * dt
	if s.altM > 0 {
		return
	}
	s.altM = 0
	s.phase = phaseIdle
	s.behavior = model.BehaviorNominal
	s.landLatched = false
	s.logf("Landed and disarmed.")
}

// flyTo moves toward target and reports whether it has arrived.
func (s *Sim) flyTo(target model.LatLon, dt float64) bool {
	east, north := model.ToLocal(s.pos, target)
	dist := math.Hypot(east, north)
	if dist < arriveRadiusM {
		return true
	}
	step := math.Min(dist, model.CruiseSpeedMS*dt)
	s.pos = model.FromLocal(s.pos, east/dist*step, north/dist*step)
	s.speedMS = model.CruiseSpeedMS
	s.headingDeg = math.Mod(math.Atan2(east, north)*180/math.Pi+360, 360)
	return false
}

// Snapshot returns the current state as the dashboard would receive it.
func (s *Sim) Snapshot() model.Snapshot {
	now := s.nowMs
	reading := func(v float64) model.Reading { return model.Reading{Value: v, AtMs: now} }
	imu := 0.97 + 0.02*math.Sin(float64(now)/4000)

	// GPS reports the spoof's route while it runs, including its speed and course.
	// The independent estimate stays near the truth with a small wander.
	reported, speed, heading := s.pos, s.speedMS, s.headingDeg
	if s.spoof.active {
		reported, speed, heading = s.spoof.pos, model.CruiseSpeedMS, s.spoof.headingDeg
	}
	t := float64(now) / 1000
	return model.Snapshot{
		NowMs:         now,
		Connected:     true,
		HasPosition:   true,
		HasFlightData: true,
		HasHeading:    true,
		HasGPS:        true,
		Armed:         s.Flying(),
		Mode:          s.mode(),
		Behavior:      s.behavior,
		Reported:      reported,
		Estimated:     model.FromLocal(s.pos, 1.5*math.Sin(t/3), 1.5*math.Cos(t/5)),
		HasEstimate:   true,
		AltM:          s.altM,
		SpeedMS:       speed,
		HeadingDeg:    heading,
		Satellites:    14,
		Overall:       reading(min(s.navTrust, s.linkTrust, s.motorHealth)),
		Nav:           reading(s.navTrust),
		Link:          reading(s.linkTrust),
		IMU:           reading(imu),
		Motors:        reading(s.motorHealth),
		Battery:       reading(s.battery),
	}
}

func (s *Sim) mode() string {
	if !s.Flying() {
		return "STANDBY"
	}
	switch s.behavior {
	case model.BehaviorLinkHold:
		return "BRAKE"
	case model.BehaviorLinkReturn:
		return "RTL"
	case model.BehaviorSafeLand:
		return "LAND"
	}
	switch s.phase {
	case phaseReturning:
		return "RTL"
	case phaseLanding:
		return "LAND"
	}
	return "AUTO"
}

func clamp01(v float64) float64 { return math.Max(0, math.Min(1, v)) }

func (s *Sim) logf(format string, args ...any) {
	s.eventSeq++
	s.events = append(s.events, model.Event{Seq: s.eventSeq, AtMs: s.nowMs, Text: fmt.Sprintf(format, args...)})
	if len(s.events) > maxEvents {
		s.events = s.events[len(s.events)-maxEvents:]
	}
}
