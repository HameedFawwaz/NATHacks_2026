package bridge

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/mavlink"
	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

func testKey(seed byte) (key [mavlink.KeyLen]byte) {
	for i := range key {
		key[i] = seed + byte(i)
	}
	return key
}

// setupSigned starts a bridge that signs with bridgeKey and a fake vehicle that
// signs with vehicleKey, or sends unsigned if vehicleKey is nil.
func setupSigned(t *testing.T, bridgeKey [mavlink.KeyLen]byte, vehicleKey *[mavlink.KeyLen]byte) (*Bridge, *fakeVehicle, context.Context) {
	t.Helper()
	ctx, cancel := context.WithCancel(context.Background())
	t.Cleanup(cancel)
	b, err := Listen("127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	b.EnableSigning(bridgeKey)
	go b.Run(ctx)
	f := startFake(t, ctx, b)
	if vehicleKey != nil {
		f.signer = mavlink.NewSigner(*vehicleKey, 1)
	}
	return b, f, ctx
}

func waitFor(t *testing.T, what string, cond func() bool) {
	t.Helper()
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		if cond() {
			return
		}
		time.Sleep(5 * time.Millisecond)
	}
	t.Fatalf("timed out waiting for %s", what)
}

func TestSignedUploadWorks(t *testing.T) {
	key := testKey(1)
	b, f, ctx := setupSigned(t, key, &key)
	go f.serve(ctx) // verifies that everything the bridge sends is signed
	connect(t, b, f)

	plan := model.DefaultPlan()
	if err := b.UploadPlan(ctx, plan); err != nil {
		t.Fatalf("upload on a signed link: %v", err)
	}
	select {
	case items := <-f.received:
		if len(items) != len(plan.Waypoints)+3 {
			t.Fatalf("vehicle received %d items", len(items))
		}
	case <-time.After(time.Second):
		t.Fatal("vehicle never finished receiving")
	}
	if s, _ := b.Snapshot(0); !s.Signing.Enabled || s.Signing.RejectedBadSig+s.Signing.RejectedUnsigned+s.Signing.RejectedReplay != 0 {
		t.Fatalf("signing state after a clean upload: %+v", s.Signing)
	}
}

func TestUploadWithWrongKeyIsRefused(t *testing.T) {
	bridgeKey, other := testKey(1), testKey(2)
	b, f, ctx := setupSigned(t, bridgeKey, &other)
	go f.serve(ctx)

	f.heartbeat()
	waitFor(t, "the bridge to reject the heartbeat", func() bool {
		s, _ := b.Snapshot(0)
		return s.Signing.RejectedBadSig > 0
	})
	if s, _ := b.Snapshot(0); s.Connected {
		t.Fatal("connected to a vehicle that signs with another key")
	}
	if err := b.UploadPlan(ctx, model.DefaultPlan()); !errors.Is(err, ErrNoVehicle) {
		t.Fatalf("upload = %v, want ErrNoVehicle", err)
	}
	select {
	case <-f.received:
		t.Fatal("vehicle received a mission")
	default:
	}
}

func TestSignedBridgeDropsUnsignedFrames(t *testing.T) {
	b, f, _ := setupSigned(t, testKey(1), nil)
	f.heartbeat()
	waitFor(t, "the bridge to count the unsigned frame", func() bool {
		s, _ := b.Snapshot(0)
		return s.Signing.RejectedUnsigned > 0
	})
	if s, _ := b.Snapshot(0); s.Connected {
		t.Fatal("an unsigned heartbeat connected a signed bridge")
	}
}

// A captured frame sent again must not change what the operator sees.
func TestReplayedFrameIsNotDisplayed(t *testing.T) {
	key := testKey(1)
	b, f, _ := setupSigned(t, key, &key)
	connect(t, b, f)

	score := func(v float32) []byte { return f.encode(mavlink.MsgNamedValueFloat, namedValue("TRUST_NAV", v)) }
	send := func(frame []byte) {
		if _, err := f.conn.WriteToUDP(frame, f.bridge); err != nil {
			t.Fatal(err)
		}
	}
	trust := func() float64 { s, _ := b.Snapshot(0); return s.Nav.Value }

	healthy, attacked := score(0.9), score(0.1)
	send(healthy)
	waitFor(t, "the first score", func() bool { return trust() == float64(float32(0.9)) })
	send(attacked)
	waitFor(t, "the second score", func() bool { return trust() == float64(float32(0.1)) })

	// Replaying the healthy score would hide the attack if it were believed.
	send(healthy)
	waitFor(t, "the replay to be counted", func() bool {
		s, _ := b.Snapshot(0)
		return s.Signing.RejectedReplay == 1
	})
	if got := trust(); got != float64(float32(0.1)) {
		t.Fatalf("a replayed frame changed the score to %v", got)
	}
}

func TestTamperedFrameIsNotDisplayed(t *testing.T) {
	key := testKey(1)
	b, f, _ := setupSigned(t, key, &key)
	connect(t, b, f)

	frame := f.encode(mavlink.MsgNamedValueFloat, namedValue("TRUST_NAV", 0.9))
	frame[10] ^= 0x01 // a payload byte; the CRC no longer matches either
	if _, err := f.conn.WriteToUDP(frame, f.bridge); err != nil {
		t.Fatal(err)
	}
	time.Sleep(100 * time.Millisecond)
	if s, _ := b.Snapshot(0); s.Nav.AtMs != 0 {
		t.Fatalf("displayed a tampered score: %+v", s.Nav)
	}
}
