package bridge

import (
	"context"
	"errors"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/mavlink"
	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

func TestCompletedCommandIgnoresDuplicateAck(t *testing.T) {
	b := &Bridge{command: &command{steps: startFlightSteps, done: make(chan error, 1)}}
	b.command.finish(nil)
	b.command.index = len(startFlightSteps)
	p := make([]byte, 10)
	p[8], p[9] = ownSysID, ownCompID
	b.handleCommand(frame(1, 1, mavlink.MsgCommandAck, p))
}

func TestStartAndUploadAreMutuallyExclusive(t *testing.T) {
	b, f, ctx := setup(t)
	connect(t, b, f)
	b.mu.Lock()
	b.upload = &upload{}
	b.mu.Unlock()
	// A live context is needed to check the busy guard, without hanging if it is absent.
	short, cancel := context.WithTimeout(ctx, 100*time.Millisecond)
	defer cancel()
	if err := b.StartFlight(short); !errors.Is(err, ErrBusy) {
		t.Fatalf("start during upload: %v", err)
	}
	b.mu.Lock()
	b.upload = nil
	b.command = &command{}
	b.mu.Unlock()
	if err := b.UploadPlan(short, model.DefaultPlan()); !errors.Is(err, ErrBusy) {
		t.Fatalf("upload during start: %v", err)
	}
}

func TestStartRequiresConfirmedMission(t *testing.T) {
	b, f, ctx := setup(t)
	connect(t, b, f)
	short, cancel := context.WithTimeout(ctx, 100*time.Millisecond)
	defer cancel()
	if err := b.StartFlight(short); !errors.Is(err, ErrNoMission) {
		t.Fatalf("start without a confirmed mission: %v", err)
	}
}

func TestSignedStartFlightSequence(t *testing.T) {
	key := testKey(7)
	b, f, ctx := setupSigned(t, key, &key)
	go f.serve(ctx)
	connect(t, b, f)
	if err := b.UploadPlan(ctx, model.DefaultPlan()); err != nil {
		t.Fatal(err)
	}
	short, cancel := context.WithTimeout(ctx, time.Second)
	defer cancel()
	result := make(chan error, 1)
	go func() { result <- b.StartFlight(short) }()
	for _, step := range startFlightSteps {
		fr := f.nextCommand(t)
		if decodeCommand(fr) != step.command {
			t.Fatalf("unexpected command: %d", decodeCommand(fr))
		}
		f.acceptAndConfirm(fr)
	}
	if err := <-result; err != nil {
		t.Fatal(err)
	}
}

func TestStartWaitsForStateNotJustDuplicateModeAcks(t *testing.T) {
	b, f, ctx := setup(t)
	go f.serve(ctx)
	connect(t, b, f)
	if err := b.UploadPlan(ctx, model.DefaultPlan()); err != nil {
		t.Fatal(err)
	}
	short, cancel := context.WithTimeout(ctx, 300*time.Millisecond)
	defer cancel()
	result := make(chan error, 1)
	go func() { result <- b.StartFlight(short) }()
	fr := f.nextCommand(t)
	f.ack(decodeCommand(fr), mavlink.ResultAccepted)
	f.ack(decodeCommand(fr), mavlink.ResultAccepted)
	if err := <-result; err == nil {
		t.Fatal("reported flight started without any confirming heartbeat")
	}
	select {
	case fr := <-f.commands:
		t.Fatalf("advanced past unconfirmed GUIDED: command %d", decodeCommand(fr))
	default:
	}
}

func TestReturningHeartbeatDoesNotResumeOldMissionOrDrift(t *testing.T) {
	b, f, _ := setup(t)
	connect(t, b, f)
	stopped := false
	b.mu.Lock()
	b.missionReady = true
	b.vehicle.fault = FaultGPSDrift
	b.stopDrift = func() { stopped = true }
	b.vehicle.lastHeartbeat = b.nowMs() - heartbeatTimeoutMs - 1
	b.mu.Unlock()
	b.handle(mavlink.Encode(0, 1, 1, mavlink.MsgHeartbeat, heartbeatPayload(2, 3, 0, 0)), f.bridge)
	b.mu.Lock()
	defer b.mu.Unlock()
	if b.missionReady || !stopped || b.stopDrift != nil || b.vehicle.fault != FaultUnknown {
		t.Fatalf("reused state after loss: ready=%v stopped=%v fault=%q", b.missionReady, stopped, b.vehicle.fault)
	}
}

func TestNewControlEndpointsRejectRemoteClients(t *testing.T) {
	b := &Bridge{}
	for _, route := range []string{"/api/start", "/api/fault"} {
		for _, remote := range []string{"192.168.1.10:1234", "[2001:db8::1]:1234", "invalid"} {
			r := httptest.NewRequest("POST", "http://localhost"+route, strings.NewReader(`{"fault":"none"}`))
			r.RemoteAddr = remote
			r.Header.Set("Content-Type", "application/json")
			r.Header.Set("X-Forwarded-For", "127.0.0.1")
			w := httptest.NewRecorder()
			b.Handler().ServeHTTP(w, r)
			if w.Code != http.StatusForbidden {
				t.Errorf("%s from %s: status %d, want 403", route, remote, w.Code)
			}
		}
	}
}
