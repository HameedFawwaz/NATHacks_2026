package bridge

import (
	"context"
	"errors"
	"math"
	"time"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/mavlink"
	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

const recoveryCompID = 191
const recoveryCommand = 31000 // MAV_CMD_USER_1; only this companion handles it

var ErrRecoveryUnavailable = errors.New("core recovery controls are unavailable or stale; the local SITL launcher must enable them")

type recoveryWait struct {
	enabled bool
	reset   bool
	nonce   uint16
	done    chan struct{}
}

func (v *vehicleState) applyRecoveryValue(payload []byte, now int64) {
	name, value := mavlink.DecodeNamedValueFloat(payload)
	switch name {
	case "RECOV_EN":
		if value == 0 || value == 1 {
			v.snap.RecoveryEnabled, v.snap.RecoveryAtMs = value == 1, now
		}
	case "RECOV_CTL":
		if value == 0 || value == 1 {
			v.snap.RecoveryControllable, v.recoveryControlAt = value == 1, now
		}
	case "RECOV_ACK":
		if value >= 2 && value <= 131071 && math.Trunc(value) == value {
			v.recoveryAck = uint32(value)
			v.snap.RecoveryEnabled, v.snap.RecoveryAtMs = v.recoveryAck&1 != 0, now
		}
	case "AUTO_STATE":
		states := []string{model.BehaviorNominal, model.BehaviorGNSSFailover, model.BehaviorLinkHold,
			model.BehaviorLinkReturn, model.BehaviorSafeLand}
		if value >= 0 && value < float64(len(states)) && math.Trunc(value) == value {
			v.snap.Behavior = states[int(value)]
		}
	case "RECOV_GPS":
		states := []string{"Waiting for confirmed GPS-free backup navigation.", "Waiting for fresh, good-quality sensor data.",
			"GPS and backup position are inconsistent; recovery is waiting.", "GPS and backup velocity disagree; recovery is waiting.",
			"Revalidating GPS against backup navigation (8 seconds required).", "GPS baseline revalidated; waiting for normal trust checks."}
		if value >= 0 && value < float64(len(states)) && math.Trunc(value) == value {
			v.snap.RecoveryGPSStatus, v.snap.RecoveryGPSAtMs = states[int(value)], now
		}
	case "RECOV_SEC":
		if value >= 0 && value <= 60 {
			v.snap.RecoveryGPSSeconds = value
		}
	case "RESET_ACK":
		if value >= 1 && value <= 65535 && math.Trunc(value) == value {
			v.resetAck = uint16(value)
		}
	}
}

// SetRecovery waits for a nonce-bearing acknowledgement from the core itself.
// Neither an HTTP success nor a previously matching status may fake confirmation.
func (b *Bridge) SetRecovery(ctx context.Context, enabled bool) error {
	return b.setRecovery(ctx, enabled, false)
}

// ResetState clears only SITL's injected faults, then requests a gated ground
// rebaseline. It preserves the mission, recovery setting and aircraft mode.
func (b *Bridge) ResetState(ctx context.Context) error {
	b.mu.Lock()
	now := b.nowMs()
	var blocked error
	switch {
	case !b.vehicle.connected(now) || b.peer == nil:
		blocked = ErrNoVehicle
	case b.vehicle.snap.Armed:
		blocked = ErrArmed
	case !b.vehicle.snapshot(now).RecoveryControllable:
		blocked = ErrRecoveryUnavailable
	case b.recovery != nil || b.command != nil || b.upload != nil || b.params != nil || b.resetting:
		blocked = ErrBusy
	}
	if blocked == nil {
		b.resetting = true
	}
	b.mu.Unlock()
	if blocked != nil {
		return blocked
	}
	defer func() { b.mu.Lock(); b.resetting = false; b.mu.Unlock() }()
	if err := b.setFault(ctx, FaultNone, true); err != nil {
		return err
	}
	return b.setRecovery(ctx, false, true)
}

func (b *Bridge) setRecovery(ctx context.Context, enabled, reset bool) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	b.mu.Lock()
	now := b.nowMs()
	switch {
	case !b.vehicle.connected(now) || b.peer == nil:
		b.mu.Unlock()
		return ErrNoVehicle
	case b.recovery != nil || b.command != nil || b.upload != nil || b.params != nil || (b.resetting && !reset):
		b.mu.Unlock()
		return ErrBusy
	case !b.vehicle.snapshot(now).RecoveryControllable:
		b.mu.Unlock()
		return ErrRecoveryUnavailable
	case reset && b.vehicle.snap.Armed:
		b.mu.Unlock()
		return ErrArmed
	case reset && b.vehicle.fault != FaultNone:
		b.mu.Unlock()
		return ErrBusy
	}
	b.recoveryNonce++
	if b.recoveryNonce == 0 {
		b.recoveryNonce = 1
	}
	for b.recoveryNonce == uint16(b.vehicle.recoveryAck>>1) || b.recoveryNonce == b.vehicle.resetAck {
		b.recoveryNonce++
		if b.recoveryNonce == 0 {
			b.recoveryNonce = 1
		}
	}
	w := &recoveryWait{enabled: enabled, reset: reset, nonce: b.recoveryNonce, done: make(chan struct{}, 1)}
	b.recovery = w
	send := func() {
		var value float32
		if enabled {
			value = 1
		}
		if reset {
			value = 2
		}
		b.sendLocked(mavlink.MsgCommandLong, mavlink.PackCommandLong(b.vehicle.sysID, recoveryCompID,
			recoveryCommand, value, float32(w.nonce)))
	}
	send()
	b.mu.Unlock()
	defer func() { b.mu.Lock(); b.recovery = nil; b.mu.Unlock() }()
	ticker := time.NewTicker(300 * time.Millisecond)
	defer ticker.Stop()
	for {
		select {
		case <-w.done:
			return nil
		case <-ticker.C:
			b.mu.Lock()
			if !b.vehicle.connected(b.nowMs()) {
				b.mu.Unlock()
				return ErrNoVehicle
			}
			if reset && b.vehicle.snap.Armed {
				b.mu.Unlock()
				return ErrArmed
			}
			send()
			b.mu.Unlock()
		case <-ctx.Done():
			if reset {
				return errors.New("ground reset not confirmed; keep the drone disarmed and wait for fresh stationary GPS/IMU and acceptable position checks")
			}
			return errors.New("core did not confirm the recovery setting; its actual state may have changed, check live status")
		}
	}
}

func (b *Bridge) handleRecoveryLocked(f mavlink.Frame) {
	w := b.recovery
	if w == nil || !b.vehicle.identified || f.SysID != b.vehicle.sysID || f.CompID != recoveryCompID ||
		f.MsgID != mavlink.MsgNamedValueFloat {
		return
	}
	name, value := mavlink.DecodeNamedValueFloat(f.Payload)
	want := uint32(w.nonce) * 2
	if w.enabled {
		want++
	}
	if (!w.reset && name == "RECOV_ACK" && value == float64(want)) ||
		(w.reset && name == "RESET_ACK" && value == float64(w.nonce)) {
		select {
		case w.done <- struct{}{}:
		default:
		}
	}
}
