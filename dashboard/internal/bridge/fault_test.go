package bridge

import (
	"errors"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"
)

// sitlParams are the SITL parameters current ArduPilot has, at their defaults.
func sitlParams() map[string]float32 {
	return map[string]float32{
		"SIM_GPS1_GLTCH_X": 0, "SIM_GPS1_GLTCH_Y": 0, "SIM_GPS1_JAM": 0,
		"SIM_ENGINE_MUL": 1, "SIM_ENGINE_FAIL": 0,
	}
}

func TestFaultSetsSITLParamsAndReplacesThePrevious(t *testing.T) {
	b, f, ctx := setup(t)
	f.params = sitlParams()
	go f.serve(ctx)
	connect(t, b, f)

	if err := b.SetFault(ctx, FaultGPSJump); err != nil {
		t.Fatalf("jump: %v", err)
	}
	if got := f.param("SIM_GPS1_GLTCH_X"); got != gpsJumpDeg {
		t.Errorf("glitch = %v, want %v", got, gpsJumpDeg)
	}
	snap, events := b.Snapshot(0)
	if snap.Fault != string(FaultGPSJump) {
		t.Errorf("snapshot fault = %q", snap.Fault)
	}
	if !strings.Contains(events[len(events)-1].Text, "GPS spoof (jump)") {
		t.Errorf("last event = %q", events[len(events)-1].Text)
	}

	if err := b.SetFault(ctx, FaultMotor); err != nil {
		t.Fatalf("motor: %v", err)
	}
	if got := f.param("SIM_GPS1_GLTCH_X"); got != 0 {
		t.Errorf("jump not cleared by the next fault: glitch = %v", got)
	}
	if f.param("SIM_ENGINE_MUL") != motorFaultThrust || f.param("SIM_ENGINE_FAIL") != motorFaultMask {
		t.Errorf("motor params = %v, %v", f.param("SIM_ENGINE_MUL"), f.param("SIM_ENGINE_FAIL"))
	}

	if err := b.SetFault(ctx, FaultNone); err != nil {
		t.Fatalf("clear: %v", err)
	}
	for name, want := range sitlParams() {
		if got := f.param(name); got != want {
			t.Errorf("after clearing, %s = %v, want %v", name, got, want)
		}
	}
}

func TestFaultUsesOlderArduPilotNames(t *testing.T) {
	b, f, ctx := setup(t)
	f.params = map[string]float32{
		"SIM_GPS_GLITCH_X": 0, "SIM_GPS_GLITCH_Y": 0, "SIM_GPS_JAM": 0,
		"SIM_ENGINE_MUL": 1, "SIM_ENGINE_FAIL": 0,
	}
	go f.serve(ctx)
	connect(t, b, f)

	if err := b.SetFault(ctx, FaultGPSJam); err != nil {
		t.Fatalf("jam: %v", err)
	}
	if f.param("SIM_GPS_JAM") != 1 {
		t.Errorf("SIM_GPS_JAM = %v", f.param("SIM_GPS_JAM"))
	}
}

func TestFaultRefusedByARealAutopilot(t *testing.T) {
	old := paramRetryInterval
	paramRetryInterval = 20 * time.Millisecond
	t.Cleanup(func() { paramRetryInterval = old })

	b, f, ctx := setup(t)
	go f.serve(ctx) // no params at all: nothing is ever confirmed
	connect(t, b, f)

	err := b.SetFault(ctx, FaultGPSJump)
	if !errors.Is(err, ErrNotSITL) {
		t.Fatalf("err = %v", err)
	}
	if snap, _ := b.Snapshot(0); snap.Fault != string(FaultUnknown) {
		t.Errorf("unconfirmed fault must be unknown, not clear or active: %q", snap.Fault)
	}
}

func TestFaultRefusedWithoutVehicle(t *testing.T) {
	b, _, ctx := setup(t)
	if err := b.SetFault(ctx, FaultGPSJump); err != ErrNoVehicle {
		t.Fatalf("err = %v", err)
	}
	if err := b.SetFault(ctx, "bogus"); err == nil {
		t.Fatal("unknown fault accepted")
	}
}

func TestDriftWalksUntilReplaced(t *testing.T) {
	b, f, ctx := setup(t)
	f.params = sitlParams()
	go f.serve(ctx)
	connect(t, b, f)

	if err := b.SetFault(ctx, FaultGPSDrift); err != nil {
		t.Fatalf("drift: %v", err)
	}
	// Each step goes out every driftInterval; three steps should land within ~1.5s.
	deadline := time.Now().Add(3 * time.Second)
	var last float32
	for steps := 0; steps < 3 && time.Now().Before(deadline); {
		if got := f.param("SIM_GPS1_GLTCH_X"); got > last {
			last, steps = got, steps+1
		}
		time.Sleep(20 * time.Millisecond)
	}
	if last <= 0 {
		t.Fatal("the drift never moved the glitch offset")
	}
	// The offset tracks driftRateMS: well under a few seconds' worth so far.
	if meters := float64(last) * metersPerDegLat; meters > driftRateMS*4 {
		t.Errorf("drifted %.1f m already", meters)
	}

	if err := b.SetFault(ctx, FaultNone); err != nil {
		t.Fatalf("clear: %v", err)
	}
	time.Sleep(2 * driftInterval)
	if got := f.param("SIM_GPS1_GLTCH_X"); got != 0 {
		t.Errorf("drift kept running after it was cleared: glitch = %v", got)
	}
}

func TestHTTPFault(t *testing.T) {
	b, f, ctx := setup(t)
	f.params = sitlParams()
	go f.serve(ctx)
	connect(t, b, f)
	srv := httptest.NewServer(b.Handler(""))
	defer srv.Close()

	post := func(origin, body string) int {
		req, _ := http.NewRequest("POST", srv.URL+"/api/fault", strings.NewReader(body))
		req.Header.Set("Content-Type", "application/json")
		if origin != "" {
			req.Header.Set("Origin", origin)
		}
		resp, err := http.DefaultClient.Do(req)
		if err != nil {
			t.Fatal(err)
		}
		resp.Body.Close()
		return resp.StatusCode
	}

	if got := post("http://evil.example", `{"fault":"gps_jump"}`); got != http.StatusForbidden {
		t.Errorf("cross-origin: %d", got)
	}
	if got := post("", `{"fault":"self_destruct"}`); got != http.StatusBadRequest {
		t.Errorf("unknown fault: %d", got)
	}
	if got := post("", `{"fault":"gps_jump"}`); got != http.StatusOK {
		t.Errorf("jump: %d", got)
	}
	if f.param("SIM_GPS1_GLTCH_X") != gpsJumpDeg {
		t.Error("jump not applied")
	}
}

// Fault injection goes through the same signed send and receive path as
// everything else, so it must work when the link is signed (issue #18).
func TestFaultOnSignedLink(t *testing.T) {
	key := testKey(1)
	b, f, ctx := setupSigned(t, key, &key)
	f.params = sitlParams()
	go f.serve(ctx) // only accepts what's signed with key
	connect(t, b, f)

	if err := b.SetFault(ctx, FaultGPSJump); err != nil {
		t.Fatalf("jump on a signed link: %v", err)
	}
	if f.param("SIM_GPS1_GLTCH_X") != gpsJumpDeg {
		t.Error("jump not applied")
	}
}
