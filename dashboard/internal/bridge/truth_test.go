package bridge

import (
	"encoding/binary"
	"math"
	"testing"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/mavlink"
	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

func TestSITLTruthOptInIdentityFreshnessAndIsolation(t *testing.T) {
	v := identified(t)
	p := make([]byte, 92)
	lat, lon := int32(-353632620), int32(1491652370)
	binary.LittleEndian.PutUint32(p[84:], uint32(lat))
	binary.LittleEndian.PutUint32(p[88:], uint32(lon))
	apply := func(sys, comp uint8, now int64) { v.apply(frame(sys, comp, mavlink.MsgSimState, p), now) }
	apply(1, 1, 1100)
	if v.snapshot(1100).HasTruth {
		t.Fatal("truth accepted without SITL opt-in")
	}
	v.snap.SITLTruth = true
	apply(2, 1, 1100)
	apply(1, 191, 1100)
	if v.snapshot(1100).HasTruth {
		t.Fatal("non-autopilot supplied truth")
	}
	v.snap.Reported = model.LatLon{Lat: -35.36, Lon: 149.16}
	v.snap.Nav = model.Reading{Value: .1, AtMs: 1000}
	before := v.snap
	apply(1, 1, 1200)
	s := v.snapshot(1200)
	if !s.HasTruth || math.Abs(s.Truth.Lat+35.363262) > 1e-8 {
		t.Fatalf("missing truth: %+v", s)
	}
	if v.snap.Reported != before.Reported || v.snap.Nav != before.Nav || v.ekfKnown || v.estNAt != 0 {
		t.Fatal("display truth contaminated navigation data")
	}
	later := int64(1200 + model.StaleAfterMs + 1)
	v.apply(frame(1, 1, mavlink.MsgHeartbeat, heartbeatPayload(2, 3, 0, 0)), later)
	if v.snapshot(later).HasTruth {
		t.Fatal("stale truth shown as current")
	}
	// Invalid floats do not become a position (extensions zero = legacy fallback).
	p = make([]byte, 92)
	binary.LittleEndian.PutUint32(p[52:], math.Float32bits(float32(math.NaN())))
	apply(1, 1, later)
	if v.snapshot(later).HasTruth {
		t.Fatal("NaN truth accepted")
	}
}
