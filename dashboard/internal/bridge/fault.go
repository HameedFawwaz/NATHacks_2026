package bridge

import (
	"context"
	"errors"
	"fmt"
	"math"
	"slices"
	"strings"
	"time"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/mavlink"
)

// Fault is a problem injected into ArduPilot SITL through its own SIM_*
// parameters, so that core's detectors see a genuinely faulty vehicle rather
// than numbers the dashboard made up. A real autopilot has no SIM_* parameters
// and never confirms them, so on real hardware injection fails and changes
// nothing.
type Fault string

const (
	FaultNone     Fault = "none"
	FaultUnknown  Fault = "unknown"   // an unconfirmed/partial write; never offered as an injection
	FaultGPSJump  Fault = "gps_jump"  // GPS teleports ~110 m north in one fix
	FaultGPSDrift Fault = "gps_drift" // GPS walks north at driftRateMS
	FaultGPSJam   Fault = "gps_jam"   // SITL's jamming model: fix and satellites degrade
	FaultMotor    Fault = "motor"     // motor 1 delivers only motorFaultThrust of its command
)

// Faults lists the faults in the order the dashboard offers them.
var Faults = []Fault{FaultNone, FaultGPSJump, FaultGPSDrift, FaultGPSJam, FaultMotor}

var faultNames = map[Fault]string{
	FaultNone:     "all clear",
	FaultGPSJump:  "GPS spoof (jump)",
	FaultGPSDrift: "GPS spoof (drift)",
	FaultGPSJam:   "GPS jamming",
	FaultMotor:    "motor fault",
}

const (
	// About 110 m of latitude, the same jump run.sh suggests typing by hand.
	gpsJumpDeg = 0.001

	// Between issue #19's slow (1.5 m/s) and fast (4.5 m/s) ramps, and above the
	// ~1.25 m/s core's long-horizon check can detect (see core/README.md).
	driftRateMS   = 3.0
	driftInterval = 500 * time.Millisecond
	driftMaxM     = 1000.0 // the false position stops moving away after this

	metersPerDegLat = 111320.0

	// Enough to show on the ESC RPMs, but not so little that the copter can't fly.
	motorFaultThrust = 0.5
	motorFaultMask   = 1 // SIM_ENGINE_FAIL bitmask: servo 1, i.e. motor 1

	paramMaxRetries = 3
)

// A variable so tests needn't wait out the real interval.
var paramRetryInterval = time.Second

var ErrNotSITL = errors.New("the drone didn't accept SITL's fault parameters; fault injection only works against ArduPilot SITL")

// simParam is one SITL setting, under each name ArduPilot has given it. Names
// change between releases and each release ignores the others, so all are sent
// and whichever exists confirms.
type simParam struct {
	names []string
	value float32
}

var (
	glitchLat = []string{"SIM_GPS1_GLTCH_X", "SIM_GPS_GLITCH_X"}
	glitchLon = []string{"SIM_GPS1_GLTCH_Y", "SIM_GPS_GLITCH_Y"}
	gpsJam    = []string{"SIM_GPS1_JAM", "SIM_GPS_JAM"}
	engMul    = []string{"SIM_ENGINE_MUL"}
	engFail   = []string{"SIM_ENGINE_FAIL"}
)

// faultParams is what a fault sets. Every fault sets every parameter, so
// switching from one fault to another also clears the first.
func faultParams(f Fault) []simParam {
	lat, jam, mul, fail := float32(0), float32(0), float32(1), float32(0)
	switch f {
	case FaultGPSJump:
		lat = gpsJumpDeg
	case FaultGPSJam:
		jam = 1
	case FaultMotor:
		mul, fail = motorFaultThrust, motorFaultMask
	}
	return []simParam{
		{glitchLat, lat},
		{glitchLon, 0},
		{gpsJam, jam},
		{engMul, mul},
		{engFail, fail},
	}
}

// paramWait is a set of parameters being written, waiting for the autopilot to
// echo each one back in a PARAM_VALUE.
type paramWait struct {
	pending  []simParam
	progress chan struct{}
}

func (w *paramWait) sendLocked(b *Bridge) {
	for _, p := range w.pending {
		for _, name := range p.names {
			b.sendLocked(mavlink.MsgParamSet, mavlink.PackParamSet(b.vehicle.sysID, b.vehicle.compID, name, p.value))
		}
	}
}

// handleParam confirms a parameter being written. Callers hold b.mu.
func (b *Bridge) handleParam(f mavlink.Frame) {
	w := b.params
	if w == nil || f.MsgID != mavlink.MsgParamValue {
		return
	}
	name, value := mavlink.DecodeParamValue(f.Payload)
	before := len(w.pending)
	w.pending = slices.DeleteFunc(w.pending, func(p simParam) bool {
		return slices.Contains(p.names, name) && math.Abs(float64(value-p.value)) <= 1e-6+1e-4*math.Abs(float64(p.value))
	})
	if len(w.pending) < before {
		select {
		case w.progress <- struct{}{}:
		default:
		}
	}
}

// SetFault injects a fault into SITL, replacing any fault already injected, and
// waits for the autopilot to confirm every parameter it needed.
func (b *Bridge) SetFault(ctx context.Context, f Fault) error {
	if _, ok := faultNames[f]; !ok {
		return fmt.Errorf("unknown fault %q", f)
	}
	if err := ctx.Err(); err != nil {
		return err
	}

	b.mu.Lock()
	switch {
	case !b.vehicle.connected(b.nowMs()) || b.peer == nil:
		b.mu.Unlock()
		return ErrNoVehicle
	case b.params != nil:
		b.mu.Unlock()
		return ErrBusy
	}
	b.stopDriftLocked()            // its next step mustn't land on top of the new fault
	b.vehicle.fault = FaultUnknown // an unsuccessful write may still have changed some parameters
	w := &paramWait{pending: faultParams(f), progress: make(chan struct{}, 1)}
	b.params = w
	w.sendLocked(b)
	b.mu.Unlock()

	defer func() {
		b.mu.Lock()
		b.params = nil
		b.mu.Unlock()
	}()

	if err := b.awaitParams(ctx, w); err != nil {
		return fmt.Errorf("%w; some fault parameters may have changed, use All clear to reset them", err)
	}

	b.mu.Lock()
	defer b.mu.Unlock()
	b.vehicle.fault = f
	if f == FaultGPSDrift {
		b.startDriftLocked()
	}
	b.vehicle.addEvent("Fault injection: "+faultNames[f]+".", b.nowMs())
	return nil
}

func (b *Bridge) awaitParams(ctx context.Context, w *paramWait) error {
	retry := time.NewTimer(paramRetryInterval)
	defer retry.Stop()
	retries := 0
	for {
		b.mu.Lock()
		done := len(w.pending) == 0
		b.mu.Unlock()
		if done {
			return nil
		}
		select {
		case <-w.progress:
			retries = 0
			retry.Reset(paramRetryInterval)
		case <-retry.C:
			retries++
			b.mu.Lock()
			if retries > paramMaxRetries {
				var missing []string
				for _, p := range w.pending {
					missing = append(missing, p.names[0])
				}
				b.mu.Unlock()
				return fmt.Errorf("%w (no reply for %s)", ErrNotSITL, strings.Join(missing, ", "))
			}
			if !b.vehicle.connected(b.nowMs()) {
				b.mu.Unlock()
				return errors.New("lost contact with the drone while injecting the fault")
			}
			w.sendLocked(b)
			b.mu.Unlock()
			retry.Reset(paramRetryInterval)
		case <-ctx.Done():
			return ctx.Err()
		}
	}
}

// startDriftLocked walks the GPS glitch offset north at driftRateMS. Each step
// is sent without waiting for confirmation: one lost step is made up by the
// next. Callers hold b.mu.
func (b *Bridge) startDriftLocked() {
	ctx, cancel := context.WithCancel(context.Background())
	b.stopDrift = cancel
	sys, comp := b.vehicle.sysID, b.vehicle.compID
	go func() {
		ticker := time.NewTicker(driftInterval)
		defer ticker.Stop()
		began := time.Now()
		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
			}
			meters := min(driftRateMS*time.Since(began).Seconds(), driftMaxM)
			b.mu.Lock()
			if !b.vehicle.connected(b.nowMs()) {
				// Don't carry on into a SITL that restarted, unseen by the page.
				b.stopDriftLocked()
			}
			if ctx.Err() == nil {
				for _, name := range glitchLat {
					b.sendLocked(mavlink.MsgParamSet, mavlink.PackParamSet(sys, comp, name, float32(meters/metersPerDegLat)))
				}
			}
			b.mu.Unlock()
		}
	}()
}

// stopDriftLocked ends a drift in progress. Callers hold b.mu.
func (b *Bridge) stopDriftLocked() {
	if b.stopDrift != nil {
		b.stopDrift()
		b.stopDrift = nil
	}
}
