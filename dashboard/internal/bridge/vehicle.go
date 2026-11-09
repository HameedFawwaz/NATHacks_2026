// Package bridge connects the dashboard to a vehicle over MAVLink. It turns
// incoming messages into model.Snapshot values, streams those to the browser and
// uploads missions.
package bridge

import (
	"fmt"
	"math"
	"slices"
	"strings"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/mavlink"
	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

const (
	heartbeatTimeoutMs = model.StaleAfterMs
	maxEvents          = 50

	// Prefix of core's behavior transition messages: "AUTONOMY <behavior>: <reason>".
	autonomyPrefix = "AUTONOMY "

	// Below this median RPM the motors aren't really spinning (armed on the ground,
	// throttle cut), so their RPMs say nothing about each other.
	minSpinRPM = 100
)

// copterModes names ArduCopter's custom_mode values.
var copterModes = map[uint32]string{
	0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
	6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP",
	15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW", 20: "GUIDED_NOGPS",
	21: "SMART_RTL",
}

// vehicleState accumulates telemetry from one vehicle. It isn't safe for
// concurrent use; Bridge guards it.
type vehicleState struct {
	identified    bool
	sysID, compID uint8
	lastHeartbeat int64
	wasConnected  bool
	snap          model.Snapshot
	gpsAt         int64 // when GPS_RAW_INT last arrived
	globalAt      int64 // when GLOBAL_POSITION_INT last arrived
	satsKnown     bool
	headingKnown  bool
	escReporting  [4]bool // ESC slots that have ever reported, so a motor stopping still counts
	fault         Fault   // injected from this dashboard, "" until one is; see fault.go
	events        []model.Event
	nextEventSeq  int
}

// isVehicle tells the autopilot apart from ground stations and companion
// computers, which send heartbeats on the same network.
func isVehicle(hb mavlink.Heartbeat) bool {
	return hb.Autopilot != mavlink.AutopilotInvalid &&
		hb.Type != mavlink.TypeGCS && hb.Type != mavlink.TypeOnboardController
}

// apply updates the state from one frame and reports whether the frame was from
// the vehicle's autopilot itself.
func (v *vehicleState) apply(f mavlink.Frame, nowMs int64) (fromAutopilot bool) {
	if f.MsgID == mavlink.MsgHeartbeat {
		hb := mavlink.DecodeHeartbeat(f.Payload)
		if !isVehicle(hb) {
			return false
		}
		if v.identified && (f.SysID != v.sysID || f.CompID != v.compID) {
			return false // a second vehicle; this dashboard follows the first
		}
		v.identified, v.sysID, v.compID = true, f.SysID, f.CompID
		v.applyHeartbeat(hb, nowMs)
		return true
	}

	if !v.identified || f.SysID != v.sysID {
		return false
	}

	// Core publishes these from a companion computer, under another component ID.
	switch f.MsgID {
	case mavlink.MsgNamedValueFloat:
		v.applyNamedValue(f.Payload, nowMs)
	case mavlink.MsgStatusText:
		v.applyStatusText(f.Payload, nowMs)
	}

	if f.CompID != v.compID {
		return false
	}
	switch f.MsgID {
	case mavlink.MsgSysStatus:
		v.applySysStatus(mavlink.DecodeSysStatus(f.Payload), nowMs)
	case mavlink.MsgGPSRawInt:
		v.gpsAt = nowMs
		v.applyGPS(mavlink.DecodeGPSRawInt(f.Payload))
	case mavlink.MsgGlobalPositionInt:
		v.globalAt = nowMs
		v.applyGlobalPosition(mavlink.DecodeGlobalPositionInt(f.Payload))
	case mavlink.MsgESCTelemetry1To4:
		v.applyESCTelemetry(mavlink.DecodeESCTelemetry(f.Payload), nowMs)
	}
	return true
}

func (v *vehicleState) applyHeartbeat(hb mavlink.Heartbeat, nowMs int64) {
	v.lastHeartbeat = nowMs
	v.snap.Armed = hb.BaseMode&mavlink.ModeFlagArmed != 0
	v.snap.Mode = fmt.Sprintf("Mode %d", hb.CustomMode)
	if name, ok := copterModes[hb.CustomMode]; ok && hb.Autopilot == mavlink.AutopilotArduPilot {
		v.snap.Mode = name
	}
}

func (v *vehicleState) applyNamedValue(p []byte, nowMs int64) {
	name, value := mavlink.DecodeNamedValueFloat(p)
	if !(value >= 0 && value <= 1) { // also rejects NaN; infinities would break JSON
		return
	}
	reading := model.Reading{Value: value, AtMs: nowMs}
	switch name {
	case "TRUST":
		v.snap.Overall = reading
	case "TRUST_NAV":
		v.snap.Nav = reading
	case "TRUST_LNK":
		v.snap.Link = reading
	case "TRUST_HW":
		v.snap.Hardware = reading
	}
}

func (v *vehicleState) applyStatusText(p []byte, nowMs int64) {
	_, text := mavlink.DecodeStatusText(p)
	v.addEvent(text, nowMs)
	if rest, ok := strings.CutPrefix(text, autonomyPrefix); ok {
		behavior, _, _ := strings.Cut(rest, ":")
		switch behavior {
		case model.BehaviorNominal, model.BehaviorGNSSFailover, model.BehaviorLinkHold,
			model.BehaviorLinkReturn, model.BehaviorSafeLand:
			v.snap.Behavior = behavior
		}
	}
}

// applySysStatus reads battery charge and IMU health. Health is yes or no, so the
// IMU score is only ever 0 or 1. Motors don't come from here: ArduPilot always
// sets the MOTOR_OUTPUTS health bit, which says nothing about the motors. See
// applyESCTelemetry instead.
func (v *vehicleState) applySysStatus(s mavlink.SysStatus, nowMs int64) {
	health := func(mask uint32) (model.Reading, bool) {
		if s.SensorsPresent&mask != mask {
			return model.Reading{}, false
		}
		value := 0.0
		if s.SensorsHealth&mask == mask {
			value = 1
		}
		return model.Reading{Value: value, AtMs: nowMs}, true
	}
	if r, ok := health(mavlink.SensorGyro | mavlink.SensorAccel); ok {
		v.snap.IMU = r
	}
	if s.BatteryRemaining != mavlink.UnknownBattery {
		v.snap.Battery = model.Reading{Value: float64(s.BatteryRemaining) / 100, AtMs: nowMs}
	}
}

// applyESCTelemetry scores the motors from their ESCs' RPM: the slowest motor's
// RPM as a fraction of the median. A motor that has stopped while the others spin
// scores 0. While disarmed or not spinning there is nothing to compare, so ESCs
// that are reporting at all score 1. Only motors 1-4 are read, which covers a quad.
func (v *vehicleState) applyESCTelemetry(t mavlink.ESCTelemetry, nowMs int64) {
	var rpms []float64
	for i := range 4 {
		if t.Voltage[i] != 0 || t.Count[i] != 0 || t.RPM[i] != 0 {
			v.escReporting[i] = true
		}
		if v.escReporting[i] {
			rpms = append(rpms, float64(t.RPM[i]))
		}
	}
	if len(rpms) == 0 {
		return // no ESC telemetry on this vehicle
	}

	score := 1.0
	slices.Sort(rpms)
	n := len(rpms)
	median := (rpms[(n-1)/2] + rpms[n/2]) / 2
	if v.snap.Armed && median >= minSpinRPM {
		score = min(rpms[0]/median, 1)
	}
	v.snap.Motors = model.Reading{Value: score, AtMs: nowMs}
}

func (v *vehicleState) applyGPS(g mavlink.GPSRawInt) {
	v.satsKnown = g.Satellites != mavlink.UnknownU8
	if v.satsKnown {
		v.snap.Satellites = int(g.Satellites)
	}
	if g.FixType < mavlink.GPSFix2D {
		v.snap.HasPosition = false
		return
	}
	v.snap.HasPosition = true
	v.snap.Reported = model.LatLon{Lat: g.Lat, Lon: g.Lon}
}

// applyGlobalPosition uses the autopilot's EKF as the position estimate to
// compare GPS against, because core doesn't publish its own independent estimate
// yet. The EKF fuses GPS, so a slow enough spoof that it accepts won't show up as
// drift here.
func (v *vehicleState) applyGlobalPosition(g mavlink.GlobalPositionInt) {
	v.snap.AltM = g.RelativeAltM
	v.snap.SpeedMS = math.Hypot(g.VNorth, g.VEast)
	v.headingKnown = g.HeadingCdeg != mavlink.UnknownU16
	if v.headingKnown {
		v.snap.HeadingDeg = float64(g.HeadingCdeg) / 100
	}
	if g.Lat == 0 && g.Lon == 0 {
		v.snap.HasEstimate = false
		return
	}
	v.snap.HasEstimate = true
	v.snap.Estimated = model.LatLon{Lat: g.Lat, Lon: g.Lon}
}

func (v *vehicleState) addEvent(text string, nowMs int64) {
	v.nextEventSeq++
	v.events = append(v.events, model.Event{Seq: v.nextEventSeq, AtMs: nowMs, Text: text})
	if len(v.events) > maxEvents {
		v.events = v.events[len(v.events)-maxEvents:]
	}
}

// update records connection changes. Call it periodically.
func (v *vehicleState) update(nowMs int64) {
	connected := v.connected(nowMs)
	if connected != v.wasConnected {
		if connected {
			v.addEvent("Connected to the drone.", nowMs)
		} else {
			v.addEvent("Lost contact with the drone.", nowMs)
			v.snap.Behavior = ""
			v.fault = "" // a SITL that restarted has its parameters back at their defaults
		}
		v.wasConnected = connected
	}
}

func (v *vehicleState) connected(nowMs int64) bool {
	return v.identified && nowMs-v.lastHeartbeat <= heartbeatTimeoutMs
}

func (v *vehicleState) snapshot(nowMs int64) model.Snapshot {
	s := v.snap
	s.NowMs = nowMs
	s.Connected = v.connected(nowMs)
	s.Fault = string(v.fault)

	gpsFresh := v.gpsAt != 0 && nowMs-v.gpsAt <= model.StaleAfterMs
	globalFresh := v.globalAt != 0 && nowMs-v.globalAt <= model.StaleAfterMs
	s.HasGPS = s.Connected && gpsFresh && v.satsKnown
	s.HasPosition = s.HasPosition && s.Connected && gpsFresh
	s.HasFlightData = s.Connected && globalFresh
	s.HasHeading = s.HasFlightData && v.headingKnown
	s.HasEstimate = s.HasEstimate && s.HasFlightData
	return s
}
