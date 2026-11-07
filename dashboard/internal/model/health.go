package model

import "fmt"

// These mirror core_autonomy_default_config() in core: below TrustCritical the
// drone treats a subsystem as untrusted, and it needs TrustHealthy to recover.
const (
	TrustHealthy  = 0.7
	TrustCritical = 0.4
	StaleAfterMs  = 3000
)

// Below these fractions, battery charge is a warning or critical.
const (
	batteryCaution  = 0.30
	batteryCritical = 0.15
)

// DriftThresholdM is the reported-vs-estimated distance at which the map starts
// drawing a drift line.
const DriftThresholdM = 10.0

// Behaviors reported by the drone, named after core_autonomy_behavior_t.
const (
	BehaviorNominal      = "NOMINAL"
	BehaviorGNSSFailover = "GNSS_FAILOVER"
	BehaviorLinkHold     = "LINK_HOLD"
	BehaviorLinkReturn   = "LINK_RETURN"
	BehaviorSafeLand     = "SAFE_LAND"
)

// Reading is one sampled value in [0, 1]. AtMs == 0 means it was never received.
type Reading struct {
	Value float64
	AtMs  int64
}

// Snapshot is the vehicle's state at one moment.
type Snapshot struct {
	NowMs int64

	Connected bool // the vehicle is being heard
	Armed     bool
	Mode      string
	Behavior  string // empty until the drone has reported one

	// Each Has field is false until the data arrives and again once it goes stale,
	// even while heartbeats continue.
	Reported      LatLon // position according to GNSS
	HasPosition   bool
	Estimated     LatLon // independent position estimate
	HasEstimate   bool
	AltM          float64
	SpeedMS       float64
	HeadingDeg    float64
	HasFlightData bool // AltM, SpeedMS and HeadingDeg are current
	Satellites    int
	HasGPS        bool // Satellites is current

	Overall, Nav, Link, IMU, Motors, Battery Reading

	// Hardware is core's combined TRUST_HW score. It can trigger a landing, so it
	// has a row of its own once anything reports it.
	Hardware Reading
}

// Drift returns the distance between reported and estimated position, and
// whether it is large enough to show.
func (s Snapshot) Drift() (meters float64, show bool) {
	if !s.HasPosition || !s.HasEstimate {
		return 0, false
	}
	meters = DistanceM(s.Reported, s.Estimated)
	return meters, meters >= DriftThresholdM
}

// Event is one entry in the operator's event log. Seq increases with each event,
// so a consumer can tell which ones it has already shown.
type Event struct {
	Seq  int
	AtMs int64
	Text string
}

// Update is one message on the telemetry stream from the bridge to the browser.
type Update struct {
	Snapshot Snapshot
	Events   []Event `json:",omitempty"`
}

type Level int

const (
	LevelNoData Level = iota
	LevelHealthy
	LevelCaution
	LevelCritical
)

func (l Level) String() string {
	switch l {
	case LevelHealthy:
		return "Healthy"
	case LevelCaution:
		return "Caution"
	case LevelCritical:
		return "Critical"
	}
	return "No data"
}

// Subsystem is one row of the health panel.
type Subsystem struct {
	Name   string
	Score  float64 // meaningful unless Level is LevelNoData
	Level  Level
	Reason string
}

type subsystemText struct {
	name                       string
	healthy, caution, critical string
	neverReported              string // optional; why there may be no data
}

func assess(text subsystemText, r Reading, cautionBelow, criticalBelow float64, connected bool, nowMs int64) Subsystem {
	s := Subsystem{Name: text.name, Score: r.Value}
	switch {
	case !connected:
		s.Reason = "No contact with the drone."
	case r.AtMs == 0:
		s.Reason = "Nothing received from the drone yet."
		if text.neverReported != "" {
			s.Reason = text.neverReported
		}
	case nowMs-r.AtMs > StaleAfterMs:
		s.Reason = fmt.Sprintf("No update for %d s. The value may be out of date.", (nowMs-r.AtMs)/1000)
	case r.Value < criticalBelow:
		s.Level, s.Reason = LevelCritical, text.critical
	case r.Value < cautionBelow:
		s.Level, s.Reason = LevelCaution, text.caution
	default:
		s.Level, s.Reason = LevelHealthy, text.healthy
	}
	return s
}

// Assess returns the health of each subsystem, in display order.
func Assess(s Snapshot) []Subsystem {
	subsystems := []Subsystem{
		assess(subsystemText{"GNSS",
			"GPS position agrees with the drone's own estimate.",
			"GPS position is starting to disagree with the drone's own estimate.",
			"GPS position can't be trusted. It may be jammed or spoofed.",
			"",
		}, s.Nav, TrustHealthy, TrustCritical, s.Connected, s.NowMs),
		assess(subsystemText{"IMU",
			"Motion sensors are reading normally.",
			"Motion sensors are giving inconsistent readings.",
			"Motion sensors can't be trusted.",
			"",
		}, s.IMU, TrustHealthy, TrustCritical, s.Connected, s.NowMs),
		assess(subsystemText{"Link",
			"Connection to the drone is strong.",
			"Connection to the drone is weakening.",
			"Connection to the drone is lost or being jammed.",
			"",
		}, s.Link, TrustHealthy, TrustCritical, s.Connected, s.NowMs),
		assess(subsystemText{"Battery",
			"Charge is good.",
			"Charge is getting low. Plan to land soon.",
			"Charge is very low. Land now.",
			"",
		}, s.Battery, batteryCaution, batteryCritical, s.Connected, s.NowMs),
		assess(subsystemText{"Motors",
			"All motors are responding normally.",
			"A motor isn't responding as expected.",
			"A motor is failing.",
			"The drone isn't reporting motor health.",
		}, s.Motors, TrustHealthy, TrustCritical, s.Connected, s.NowMs),
	}
	if s.Hardware.AtMs != 0 {
		subsystems = append(subsystems, assess(subsystemText{"Hardware",
			"The drone trusts its hardware overall.",
			"The drone is losing trust in its hardware.",
			"The drone doesn't trust its hardware.",
			"",
		}, s.Hardware, TrustHealthy, TrustCritical, s.Connected, s.NowMs))
	}
	return subsystems
}

// BehaviorText describes a behavior for a non-expert operator. A behavior is what
// autonomy has decided, not proof the aircraft has done it: the mode shows that.
func BehaviorText(behavior string) string {
	switch behavior {
	case BehaviorNominal:
		return "No autonomy response active."
	case BehaviorGNSSFailover:
		return "GPS untrusted. Autonomy is switching to backup navigation."
	case BehaviorLinkHold:
		return "Link lost. Autonomy is commanding a hold while it recovers."
	case BehaviorLinkReturn:
		return "Link still lost. Autonomy is commanding a return home."
	case BehaviorSafeLand:
		return "Can't continue safely. Autonomy is commanding a landing."
	case "":
		return "The drone hasn't reported an autonomy state yet."
	}
	return "Unknown behavior: " + behavior
}
