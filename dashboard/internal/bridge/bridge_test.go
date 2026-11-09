package bridge

import (
	"bufio"
	"context"
	"encoding/binary"
	"encoding/json"
	"math"
	"net"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/mavlink"
	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

func heartbeatPayload(typ, autopilot, baseMode uint8, customMode uint32) []byte {
	p := make([]byte, 9)
	p[0], p[1], p[2], p[3] = byte(customMode), byte(customMode>>8), byte(customMode>>16), byte(customMode>>24)
	p[4], p[5], p[6] = typ, autopilot, baseMode
	return p
}

func frame(sys, comp uint8, msgID uint32, payload []byte) mavlink.Frame {
	return mavlink.Parse(mavlink.Encode(0, sys, comp, msgID, payload))[0]
}

func namedValue(name string, value float32) []byte {
	p := make([]byte, 18)
	bits := float32bits(value)
	p[4], p[5], p[6], p[7] = byte(bits), byte(bits>>8), byte(bits>>16), byte(bits>>24)
	copy(p[8:], name)
	return p
}

func statusText(text string) []byte {
	p := make([]byte, 54)
	p[0] = 4
	copy(p[1:51], text)
	return p
}

func identified(t *testing.T) *vehicleState {
	t.Helper()
	v := &vehicleState{}
	if !v.apply(frame(1, 1, mavlink.MsgHeartbeat, heartbeatPayload(2, 3, 209, 3)), 1000) {
		t.Fatal("autopilot heartbeat not accepted")
	}
	return v
}

func TestHeartbeatIdentifiesAutopilotOnly(t *testing.T) {
	v := &vehicleState{}
	v.apply(frame(255, 190, mavlink.MsgHeartbeat, heartbeatPayload(mavlink.TypeGCS, mavlink.AutopilotInvalid, 0, 0)), 100)
	v.apply(frame(1, 191, mavlink.MsgHeartbeat, heartbeatPayload(mavlink.TypeOnboardController, mavlink.AutopilotInvalid, 0, 0)), 100)
	if v.identified {
		t.Fatal("identified a GCS or companion as the vehicle")
	}

	v.apply(frame(1, 1, mavlink.MsgHeartbeat, heartbeatPayload(2, 3, 209, 17)), 200)
	s := v.snapshot(200)
	if !v.identified || !s.Armed || s.Mode != "BRAKE" || !s.Connected {
		t.Fatalf("snapshot: %+v", s)
	}

	// A second vehicle must not overwrite the first.
	v.apply(frame(2, 1, mavlink.MsgHeartbeat, heartbeatPayload(2, 3, 0, 9)), 300)
	if s := v.snapshot(300); !s.Armed || s.Mode != "BRAKE" {
		t.Fatalf("second vehicle overwrote state: %+v", s)
	}
}

func TestTrustScoresFromCompanion(t *testing.T) {
	v := identified(t)
	v.apply(frame(1, 191, mavlink.MsgNamedValueFloat, namedValue("TRUST_NAV", 0.25)), 1500)
	v.apply(frame(1, 191, mavlink.MsgNamedValueFloat, namedValue("TRUST_LNK", 0.5)), 1500)
	v.apply(frame(1, 191, mavlink.MsgNamedValueFloat, namedValue("TRUST_HW", 1)), 1500)
	s := v.snapshot(1500)
	if s.Nav != (model.Reading{Value: 0.25, AtMs: 1500}) || s.Link.Value != 0.5 || s.Hardware.Value != 1 {
		t.Fatalf("snapshot: %+v", s)
	}

	// Another system's scores are not this vehicle's.
	v.apply(frame(9, 191, mavlink.MsgNamedValueFloat, namedValue("TRUST_NAV", 1)), 1600)
	if s := v.snapshot(1600); s.Nav.Value != 0.25 {
		t.Fatalf("accepted a score from another system: %+v", s.Nav)
	}
}

func TestPositionComesOnlyFromAutopilot(t *testing.T) {
	v := identified(t)
	gps := mavlink.Parse(mustGolden(t, "fd1e000002010118000015cd5b070000000094feebea12cfe85840e908007900c80052032823030e0da3"))[0]
	v.apply(gps, 1100)
	pos := mavlink.Parse(mustGolden(t, "fd1c0000030101210000881300000001ecea88d1e85860370900204e00002c0170feceffbe146353"))[0]
	v.apply(pos, 1100)

	s := v.snapshot(1100)
	if !s.HasPosition || !s.HasEstimate || s.Satellites != 14 || s.AltM != 20 || s.SpeedMS != 5 || s.HeadingDeg != 53.1 {
		t.Fatalf("snapshot: %+v", s)
	}
	if meters, _ := s.Drift(); meters < 5 || meters > 15 {
		t.Fatalf("drift = %f, want about 10", meters)
	}

	fake := pos
	fake.CompID = 191
	before := s.Estimated
	fake.Payload = make([]byte, len(pos.Payload))
	v.apply(fake, 1200)
	if v.snapshot(1200).Estimated != before {
		t.Fatal("accepted a position from another component")
	}
}

func TestNoGPSFixHidesPosition(t *testing.T) {
	v := identified(t)
	p := make([]byte, 52)
	p[28] = 1 // no fix
	v.apply(frame(1, 1, mavlink.MsgGPSRawInt, p), 1100)
	if s := v.snapshot(1100); s.HasPosition {
		t.Fatal("position shown without a fix")
	}
}

func TestSensorHealthAndBattery(t *testing.T) {
	v := identified(t)
	status := func(present, health uint32, battery int8) []byte {
		p := make([]byte, 43)
		put32 := func(off int, x uint32) {
			p[off], p[off+1], p[off+2], p[off+3] = byte(x), byte(x>>8), byte(x>>16), byte(x>>24)
		}
		put32(0, present)
		put32(8, health)
		p[30] = byte(battery)
		return p
	}

	all := uint32(mavlink.SensorGyro | mavlink.SensorAccel)
	v.apply(frame(1, 1, mavlink.MsgSysStatus, status(all, all, 87)), 1100)
	s := v.snapshot(1100)
	if s.IMU.Value != 1 || s.Battery.Value != 0.87 {
		t.Fatalf("healthy: %+v", s)
	}

	v.apply(frame(1, 1, mavlink.MsgSysStatus, status(all, mavlink.SensorGyro, mavlink.UnknownBattery)), 1200)
	s = v.snapshot(1200)
	if s.IMU.Value != 0 {
		t.Fatalf("unhealthy: %+v", s)
	}
	if s.Battery != (model.Reading{Value: 0.87, AtMs: 1100}) {
		t.Fatalf("unknown battery overwrote the last value: %+v", s.Battery)
	}

	// Not fitted means no data, not healthy.
	w := identified(t)
	w.apply(frame(1, 1, mavlink.MsgSysStatus, status(0, 0, 50)), 1100)
	if got := w.snapshot(1100).IMU.AtMs; got != 0 {
		t.Fatalf("absent IMU reported at %d", got)
	}

	// ArduPilot always sets this bit, so it must not read as motor health.
	const motorOutputs = 1 << 15
	w.apply(frame(1, 1, mavlink.MsgSysStatus, status(motorOutputs, motorOutputs, 50)), 1200)
	if got := w.snapshot(1200).Motors.AtMs; got != 0 {
		t.Fatalf("motor outputs bit treated as motor health at %d", got)
	}
}

func escTelemetry(voltage uint16, rpm [4]uint16, present [4]bool) []byte {
	p := make([]byte, 44)
	for i := range 4 {
		if !present[i] {
			continue
		}
		p[2*i], p[2*i+1] = byte(voltage), byte(voltage>>8)
		p[24+2*i], p[24+2*i+1] = byte(rpm[i]), byte(rpm[i]>>8)
		p[32+2*i] = 1
	}
	return p
}

func TestMotorsFromESCTelemetry(t *testing.T) {
	all := [4]bool{true, true, true, true}
	esc := func(v *vehicleState, rpm [4]uint16, present [4]bool, at int64) model.Reading {
		t.Helper()
		v.apply(frame(1, 1, mavlink.MsgESCTelemetry1To4, escTelemetry(1680, rpm, present)), at)
		return v.snapshot(at).Motors
	}

	// Disarmed and idle: reporting ESCs are healthy.
	disarmed := &vehicleState{}
	disarmed.apply(frame(1, 1, mavlink.MsgHeartbeat, heartbeatPayload(2, 3, 0, 0)), 1000)
	if r := esc(disarmed, [4]uint16{}, all, 1100); r != (model.Reading{Value: 1, AtMs: 1100}) {
		t.Fatalf("disarmed: %+v", r)
	}

	v := identified(t) // armed
	if r := esc(v, [4]uint16{8000, 8200, 7900, 8100}, all, 1100); r.Value < 0.95 || r.AtMs != 1100 {
		t.Fatalf("even spin: %+v", r)
	}
	if r := esc(v, [4]uint16{8000, 4000, 8000, 8000}, all, 1200); r.Value != 0.5 {
		t.Fatalf("weak motor: %+v", r)
	}
	// A motor that stops dead also stops reporting RPM, but it has reported before.
	if r := esc(v, [4]uint16{9000, 0, 9000, 9000}, all, 1300); r.Value != 0 {
		t.Fatalf("stopped motor: %+v", r)
	}
	// Armed but not spinning yet: nothing to compare.
	if r := esc(v, [4]uint16{0, 0, 0, 0}, all, 1400); r.Value != 1 {
		t.Fatalf("armed, not spinning: %+v", r)
	}

	// Empty slots are absent motors, not stopped ones.
	tri := identified(t)
	if r := esc(tri, [4]uint16{8000, 8000, 8000, 0}, [4]bool{true, true, true, false}, 1100); r.Value != 1 {
		t.Fatalf("unused slot counted as a motor: %+v", r)
	}

	// No ESC telemetry at all means no data.
	none := identified(t)
	if r := esc(none, [4]uint16{}, [4]bool{}, 1100); r.AtMs != 0 {
		t.Fatalf("all-empty block reported motors: %+v", r)
	}
}

func TestTelemetryExpiresWhileHeartbeatsContinue(t *testing.T) {
	v := identified(t)
	v.apply(mavlink.Parse(mustGolden(t, "fd1e000002010118000015cd5b070000000094feebea12cfe85840e908007900c80052032823030e0da3"))[0], 1000)
	v.apply(mavlink.Parse(mustGolden(t, "fd1c0000030101210000881300000001ecea88d1e85860370900204e00002c0170feceffbe146353"))[0], 1000)
	if s := v.snapshot(1000); !s.HasPosition || !s.HasEstimate || !s.HasFlightData || !s.HasGPS {
		t.Fatalf("fresh: %+v", s)
	}

	const later = 1000 + model.StaleAfterMs + 1
	v.apply(frame(1, 1, mavlink.MsgHeartbeat, heartbeatPayload(2, 3, 209, 3)), later)
	s := v.snapshot(later)
	if !s.Connected {
		t.Fatal("should be connected")
	}
	if s.HasPosition || s.HasEstimate || s.HasFlightData || s.HasGPS {
		t.Fatalf("stale data shown as current: %+v", s)
	}
}

func TestBeforeFirstMessagesNothingIsCurrent(t *testing.T) {
	s := identified(t).snapshot(1000)
	if s.HasPosition || s.HasEstimate || s.HasFlightData || s.HasGPS {
		t.Fatalf("%+v", s)
	}
}

func TestOutOfRangeScoresDontPoisonTheStream(t *testing.T) {
	v := identified(t)
	for _, bad := range []float32{float32(math.Inf(1)), float32(math.Inf(-1)), float32(math.NaN()), 1.5, -0.5} {
		v.apply(frame(1, 191, mavlink.MsgNamedValueFloat, namedValue("TRUST_NAV", bad)), 1100)
	}
	s := v.snapshot(1100)
	if s.Nav.AtMs != 0 {
		t.Fatalf("accepted a bad score: %+v", s.Nav)
	}
	if _, err := json.Marshal(s); err != nil {
		t.Fatal(err)
	}
}

func TestAutonomyBehaviorFromStatusText(t *testing.T) {
	v := identified(t)
	v.apply(frame(1, 191, mavlink.MsgStatusText, statusText("AUTONOMY LINK_HOLD: link untrusted")), 1100)
	if s := v.snapshot(1100); s.Behavior != model.BehaviorLinkHold {
		t.Fatalf("behavior = %q", s.Behavior)
	}
	v.apply(frame(1, 1, mavlink.MsgStatusText, statusText("AUTONOMY BOGUS: nope")), 1200)
	if s := v.snapshot(1200); s.Behavior != model.BehaviorLinkHold {
		t.Fatalf("accepted an unknown behavior: %q", s.Behavior)
	}
	if len(v.events) != 2 || v.events[0].Text != "AUTONOMY LINK_HOLD: link untrusted" {
		t.Fatalf("events: %+v", v.events)
	}
}

func TestConnectionLossIsReported(t *testing.T) {
	v := identified(t)
	v.update(1000)
	v.apply(frame(1, 191, mavlink.MsgStatusText, statusText("AUTONOMY LINK_HOLD: x")), 1000)
	if !v.snapshot(1000 + heartbeatTimeoutMs).Connected {
		t.Fatal("should still be connected at the limit")
	}
	later := int64(1000 + heartbeatTimeoutMs + 1)
	v.update(later)
	s := v.snapshot(later)
	if s.Connected || s.Behavior != "" {
		t.Fatalf("after timeout: %+v", s)
	}
	if last := v.events[len(v.events)-1]; last.Text != "Lost contact with the drone." {
		t.Fatalf("last event: %+v", last)
	}
}

func TestBuildMission(t *testing.T) {
	plan := model.DefaultPlan()
	items := BuildMission(plan)
	if len(items) != len(plan.Waypoints)+3 {
		t.Fatalf("%d items", len(items))
	}
	if items[0].Command != mavlink.CmdNavWaypoint || items[0].Frame != mavlink.FrameGlobal || items[0].Lat != plan.Home.Lat {
		t.Errorf("home item: %+v", items[0])
	}
	if items[1].Command != mavlink.CmdNavTakeoff || items[1].AltM != plan.Waypoints[0].AltM {
		t.Errorf("takeoff item: %+v", items[1])
	}
	if items[2].Lat != plan.Waypoints[0].Lat || items[2].Frame != mavlink.FrameGlobalRelativeAlt {
		t.Errorf("first waypoint: %+v", items[2])
	}
	if items[len(items)-1].Command != mavlink.CmdNavRTL {
		t.Errorf("last item: %+v", items[len(items)-1])
	}
}

func TestValidatePlan(t *testing.T) {
	good := model.DefaultPlan()
	if err := ValidatePlan(good); err != nil {
		t.Fatal(err)
	}
	mutate := func(f func(*model.Plan)) model.Plan {
		p := model.DefaultPlan()
		f(&p)
		return p
	}
	bad := map[string]model.Plan{
		"long return": mutate(func(p *model.Plan) {
			p.Waypoints = []model.Waypoint{
				{LatLon: model.FromLocal(p.Home, 4000, 0), AltM: 20},
				{LatLon: model.FromLocal(p.Home, 8000, 0), AltM: 20},
			}
		}),
		"far away":     mutate(func(p *model.Plan) { p.Home = model.LatLon{Lat: 53.5, Lon: -113.5} }),
		"no waypoints": mutate(func(p *model.Plan) { p.Waypoints = nil }),
		"high":         mutate(func(p *model.Plan) { p.Waypoints[1].AltM = 500 }),
		"ground":       mutate(func(p *model.Plan) { p.Waypoints[1].AltM = 0 }),
		"bad lat":      mutate(func(p *model.Plan) { p.Waypoints[0].Lat = 95 }),
		"null island":  mutate(func(p *model.Plan) { p.Waypoints[0].LatLon = model.LatLon{} }),
		"no home":      mutate(func(p *model.Plan) { p.Home = model.LatLon{} }),
	}
	for name, plan := range bad {
		if ValidatePlan(plan) == nil {
			t.Errorf("%s: accepted", name)
		}
	}
}

// fakeVehicle speaks the mission protocol over real UDP.
type fakeVehicle struct {
	t                 *testing.T
	conn              *net.UDPConn
	bridge            *net.UDPAddr
	armed             bool
	ackWith           uint8
	dropOnce          bool // ignore the first item request's reply, to exercise a retry
	silent            bool
	loseFirstFinalAck bool // installs the mission but loses the ack, as a real link can
	received          chan []mavlink.Frame
	commands          chan mavlink.Frame // every COMMAND_LONG received, for command-sequence tests

	// Parameters the vehicle has, echoed back on PARAM_SET like ArduPilot does.
	// Names it doesn't have are ignored, so nil plays a real autopilot with no
	// SIM_* parameters.
	paramMu sync.Mutex
	params  map[string]float32
}

// param reads one of the vehicle's parameters.
func (f *fakeVehicle) param(name string) float32 {
	f.paramMu.Lock()
	defer f.paramMu.Unlock()
	return f.params[name]
}

func startFake(t *testing.T, ctx context.Context, b *Bridge) *fakeVehicle {
	t.Helper()
	conn, err := net.ListenUDP("udp", &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1)})
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { conn.Close() })
	return &fakeVehicle{
		t: t, conn: conn, bridge: b.Addr().(*net.UDPAddr),
		received: make(chan []mavlink.Frame, 1),
		commands: make(chan mavlink.Frame, 10),
	}
}

func (f *fakeVehicle) send(msgID uint32, payload []byte) {
	if _, err := f.conn.WriteToUDP(mavlink.Encode(0, 1, 1, msgID, payload), f.bridge); err != nil {
		f.t.Error(err)
	}
}

func (f *fakeVehicle) heartbeat() {
	base := uint8(81)
	if f.armed {
		base |= mavlink.ModeFlagArmed
	}
	f.send(mavlink.MsgHeartbeat, heartbeatPayload(2, 3, base, 5))
}

// serve answers the mission protocol until the upload ends.
func (f *fakeVehicle) serve(ctx context.Context) {
	buf := make([]byte, 2048)
	var items []mavlink.Frame
	want := -1
	for ctx.Err() == nil {
		f.conn.SetReadDeadline(time.Now().Add(50 * time.Millisecond))
		n, _, err := f.conn.ReadFromUDP(buf)
		if err != nil {
			continue
		}
		for _, fr := range mavlink.Parse(buf[:n]) {
			if f.silent {
				continue
			}
			switch fr.MsgID {
			case mavlink.MsgMissionCount:
				want = int(fr.Payload[0]) | int(fr.Payload[1])<<8
				items = nil
				if f.ackWith != 0 {
					f.send(mavlink.MsgMissionAck, []byte{ownSysID, ownCompID, f.ackWith, 0, 0, 0, 0, 0})
					continue
				}
				f.request(0)
			case mavlink.MsgMissionItemInt:
				seq := int(fr.Payload[28]) | int(fr.Payload[29])<<8
				if f.dropOnce {
					f.dropOnce = false
					continue
				}
				if seq != len(items) {
					if want == -1 { // like ArduPilot: not receiving any more
						f.send(mavlink.MsgMissionAck, []byte{ownSysID, ownCompID, 1, 0, 0, 0, 0, 0})
					}
					continue
				}
				items = append(items, fr)
				if len(items) == want && f.loseFirstFinalAck {
					f.loseFirstFinalAck = false
					want, items = -1, nil
					continue
				}
				if len(items) == want {
					f.send(mavlink.MsgMissionAck, []byte{ownSysID, ownCompID, 0, 0, 0, 0, 0, 0})
					f.received <- items
					continue
				}
				f.request(len(items))
			case mavlink.MsgCommandLong:
				select {
				case f.commands <- fr:
				default:
				}
			case mavlink.MsgParamSet:
				name := strings.TrimRight(string(fr.Payload[6:22]), "\x00")
				value := math.Float32frombits(binary.LittleEndian.Uint32(fr.Payload[0:]))
				f.paramMu.Lock()
				_, ok := f.params[name]
				if ok {
					f.params[name] = value
				}
				f.paramMu.Unlock()
				if ok {
					reply := make([]byte, 25)
					binary.LittleEndian.PutUint32(reply[0:], math.Float32bits(value))
					copy(reply[8:24], name)
					reply[24] = mavlink.ParamTypeReal32
					f.send(mavlink.MsgParamValue, reply)
				}
			}
		}
	}
}

// decodeCommand reads the command field of a COMMAND_LONG payload.
func decodeCommand(fr mavlink.Frame) uint16 {
	return uint16(fr.Payload[28]) | uint16(fr.Payload[29])<<8
}

// nextCommand waits for the vehicle to receive a COMMAND_LONG.
func (f *fakeVehicle) nextCommand(t *testing.T) mavlink.Frame {
	t.Helper()
	select {
	case fr := <-f.commands:
		return fr
	case <-time.After(2 * time.Second):
		t.Fatal("vehicle never received the expected command")
		return mavlink.Frame{}
	}
}

func (f *fakeVehicle) ack(command uint16, result uint8) {
	p := make([]byte, 10)
	p[0], p[1] = byte(command), byte(command>>8)
	p[2] = result
	p[8], p[9] = ownSysID, ownCompID
	f.send(mavlink.MsgCommandAck, p)
}

func (f *fakeVehicle) request(seq int) {
	f.send(mavlink.MsgMissionRequestInt, []byte{byte(seq), byte(seq >> 8), ownSysID, ownCompID, 0})
}

func setup(t *testing.T) (*Bridge, *fakeVehicle, context.Context) {
	t.Helper()
	ctx, cancel := context.WithCancel(context.Background())
	t.Cleanup(cancel)
	b, err := Listen("127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	go b.Run(ctx)
	return b, startFake(t, ctx, b), ctx
}

func connect(t *testing.T, b *Bridge, f *fakeVehicle) {
	t.Helper()
	f.heartbeat()
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		if s, _ := b.Snapshot(0); s.Connected && s.Armed == f.armed {
			return
		}
		time.Sleep(5 * time.Millisecond)
	}
	t.Fatal("never connected")
}

func TestUploadOverUDP(t *testing.T) {
	b, f, ctx := setup(t)
	f.dropOnce = true
	go f.serve(ctx)
	connect(t, b, f)

	plan := model.DefaultPlan()
	if err := b.UploadPlan(ctx, plan); err != nil {
		t.Fatal(err)
	}
	select {
	case items := <-f.received:
		if len(items) != len(plan.Waypoints)+3 {
			t.Fatalf("vehicle received %d items", len(items))
		}
	case <-time.After(time.Second):
		t.Fatal("vehicle never finished receiving")
	}
}

func TestUploadRefusedWithoutVehicleOrWhileArmed(t *testing.T) {
	b, f, ctx := setup(t)
	if err := b.UploadPlan(ctx, model.DefaultPlan()); err != ErrNoVehicle {
		t.Fatalf("no vehicle: %v", err)
	}
	f.armed = true
	connect(t, b, f)
	if err := b.UploadPlan(ctx, model.DefaultPlan()); err != ErrArmed {
		t.Fatalf("armed: %v", err)
	}
}

func TestUploadReportsRejection(t *testing.T) {
	b, f, ctx := setup(t)
	f.ackWith = 15
	go f.serve(ctx)
	connect(t, b, f)
	err := b.UploadPlan(ctx, model.DefaultPlan())
	if err == nil || !strings.Contains(err.Error(), "denied") {
		t.Fatalf("err = %v", err)
	}
}

func TestUploadStopsWhenCancelled(t *testing.T) {
	b, f, ctx := setup(t)
	f.silent = true
	go f.serve(ctx)
	connect(t, b, f)

	short, cancel := context.WithTimeout(ctx, 100*time.Millisecond)
	defer cancel()
	if err := b.UploadPlan(short, model.DefaultPlan()); err == nil {
		t.Fatal("silent vehicle reported success")
	}
	// The slot must be free again afterwards.
	short2, cancel2 := context.WithTimeout(ctx, 50*time.Millisecond)
	defer cancel2()
	if err := b.UploadPlan(short2, model.DefaultPlan()); err == ErrBusy {
		t.Fatal("upload slot leaked")
	}
}

func TestAcceptanceWithoutAllItemsIsNotSuccess(t *testing.T) {
	b, f, ctx := setup(t)
	connect(t, b, f)

	short, cancel := context.WithTimeout(ctx, 400*time.Millisecond)
	defer cancel()
	result := make(chan error, 1)
	go func() { result <- b.UploadPlan(short, model.DefaultPlan()) }()
	time.Sleep(50 * time.Millisecond)

	// A stale request for the last item, then ACCEPTED, as from an earlier transfer.
	last := len(BuildMission(model.DefaultPlan())) - 1
	f.request(last)
	f.send(mavlink.MsgMissionAck, []byte{ownSysID, ownCompID, 0, 0, 0, 0, 0, 0})
	if err := <-result; err == nil {
		t.Fatal("reported success without transferring the items")
	}
}

func TestLostFinalAckIsRetried(t *testing.T) {
	b, f, ctx := setup(t)
	f.loseFirstFinalAck = true
	go f.serve(ctx)
	connect(t, b, f)

	if err := b.UploadPlan(ctx, model.DefaultPlan()); err != nil {
		t.Fatalf("should recover by sending the mission again: %v", err)
	}
	select {
	case <-f.received:
	case <-time.After(time.Second):
		t.Fatal("second transfer never finished")
	}
}

func TestCancelledContextSendsNothing(t *testing.T) {
	b, f, ctx := setup(t)
	connect(t, b, f)
	time.Sleep(20 * time.Millisecond)

	cancelled, cancel := context.WithCancel(ctx)
	cancel()
	if err := b.transfer(cancelled, BuildMission(model.DefaultPlan())); err == nil {
		t.Fatal("cancelled transfer reported success")
	}

	buf := make([]byte, 2048)
	f.conn.SetReadDeadline(time.Now().Add(100 * time.Millisecond))
	for {
		n, _, err := f.conn.ReadFromUDP(buf)
		if err != nil {
			return
		}
		for _, fr := range mavlink.Parse(buf[:n]) {
			if fr.MsgID == mavlink.MsgMissionCount {
				t.Fatal("sent MISSION_COUNT, which can clear the stored mission")
			}
		}
	}
}

func TestAcceptanceAfterArmingStillConfirms(t *testing.T) {
	b, f, ctx := setup(t)
	connect(t, b, f)

	result := make(chan error, 1)
	items := BuildMission(model.DefaultPlan())
	go func() { result <- b.transfer(ctx, items) }()
	time.Sleep(50 * time.Millisecond)
	for i := range items {
		f.request(i)
	}
	time.Sleep(50 * time.Millisecond)
	f.armed = true
	f.heartbeat()
	time.Sleep(50 * time.Millisecond)
	f.send(mavlink.MsgMissionAck, []byte{ownSysID, ownCompID, 0, 0, 0, 0, 0, 0})

	select {
	case err := <-result:
		if err != nil {
			t.Fatalf("a stored mission was reported as failed: %v", err)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("no result")
	}
}

func TestUnknownFieldsDontShowOldValues(t *testing.T) {
	v := identified(t)
	gps := func(sats uint8) []byte {
		p := make([]byte, 52)
		p[28], p[29] = 3, sats
		return p
	}
	global := func(heading uint16) []byte {
		p := make([]byte, 28)
		p[26], p[27] = byte(heading), byte(heading>>8)
		return p
	}
	v.apply(frame(1, 1, mavlink.MsgGPSRawInt, gps(14)), 1000)
	v.apply(frame(1, 1, mavlink.MsgGlobalPositionInt, global(9000)), 1000)
	if s := v.snapshot(1000); !s.HasGPS || !s.HasHeading || s.HeadingDeg != 90 {
		t.Fatalf("known: %+v", s)
	}
	v.apply(frame(1, 1, mavlink.MsgGPSRawInt, gps(mavlink.UnknownU8)), 1100)
	v.apply(frame(1, 1, mavlink.MsgGlobalPositionInt, global(mavlink.UnknownU16)), 1100)
	s := v.snapshot(1100)
	if s.HasGPS || s.HasHeading {
		t.Fatalf("unknown values shown as current: %+v", s)
	}
	if !s.HasFlightData || !s.HasPosition {
		t.Fatalf("the rest of the message should still count: %+v", s)
	}
}

func TestArmingDuringUploadStopsIt(t *testing.T) {
	b, f, ctx := setup(t)
	connect(t, b, f)

	result := make(chan error, 1)
	go func() { result <- b.UploadPlan(ctx, model.DefaultPlan()) }()
	time.Sleep(50 * time.Millisecond)
	f.request(0)
	time.Sleep(50 * time.Millisecond)
	f.armed = true
	f.heartbeat()
	time.Sleep(50 * time.Millisecond)
	f.request(1)

	select {
	case err := <-result:
		if err == nil || !strings.Contains(err.Error(), "armed") {
			t.Fatalf("err = %v", err)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("upload kept going after arming")
	}
}

func TestStartFlightSequence(t *testing.T) {
	b, f, ctx := setup(t)
	go f.serve(ctx)
	connect(t, b, f)

	result := make(chan error, 1)
	go func() { result <- b.StartFlight(ctx) }()

	wantOrder := []uint16{mavlink.CmdDoSetMode, mavlink.CmdComponentArmDisarm, mavlink.CmdDoSetMode, mavlink.CmdMissionStart}
	for _, want := range wantOrder {
		fr := f.nextCommand(t)
		if got := decodeCommand(fr); got != want {
			t.Fatalf("command = %d, want %d", got, want)
		}
		f.ack(want, mavlink.ResultAccepted)
	}

	select {
	case err := <-result:
		if err != nil {
			t.Fatalf("StartFlight: %v", err)
		}
	case <-time.After(time.Second):
		t.Fatal("no result")
	}
}

func TestStartFlightRefusedWithoutVehicleOrWhileArmed(t *testing.T) {
	b, f, ctx := setup(t)
	if err := b.StartFlight(ctx); err != ErrNoVehicle {
		t.Fatalf("no vehicle: %v", err)
	}
	f.armed = true
	connect(t, b, f)
	if err := b.StartFlight(ctx); err != ErrAlreadyArmed {
		t.Fatalf("armed: %v", err)
	}
}

func TestStartFlightReportsRejection(t *testing.T) {
	b, f, ctx := setup(t)
	go f.serve(ctx)
	connect(t, b, f)

	result := make(chan error, 1)
	go func() { result <- b.StartFlight(ctx) }()

	fr := f.nextCommand(t)
	f.ack(decodeCommand(fr), mavlink.ResultDenied)

	select {
	case err := <-result:
		if err == nil || !strings.Contains(err.Error(), "denied") {
			t.Fatalf("err = %v", err)
		}
	case <-time.After(time.Second):
		t.Fatal("no result")
	}
}

func TestStartFlightBusy(t *testing.T) {
	b, f, ctx := setup(t)
	go f.serve(ctx)
	connect(t, b, f)

	go b.StartFlight(ctx)
	f.nextCommand(t) // wait for the sequence to actually start
	if err := b.StartFlight(ctx); err != ErrBusy {
		t.Fatalf("busy: %v", err)
	}
}

func TestHostAllowlist(t *testing.T) {
	cases := map[string]bool{
		"127.0.0.1:8080":    true,
		"localhost:8080":    true,
		"[::1]:8080":        true,
		"192.168.1.5:8080":  true, // the configured public host, by name
		"192.168.1.5:9000":  true,
		"evil.example:8080": false,
		"evil.example":      false,
		"192.168.1.6:8080":  false,
	}
	for host, want := range cases {
		if got := allowedHost(host, []string{":8080", "192.168.1.5"}); got != want {
			t.Errorf("%s = %v", host, got)
		}
	}
}

func TestHTTPMission(t *testing.T) {
	b, f, ctx := setup(t)
	go f.serve(ctx)
	connect(t, b, f)
	srv := httptest.NewServer(b.Handler(""))
	defer srv.Close()

	body, _ := json.Marshal(model.DefaultPlan())
	post := func(contentType, origin, body string) int {
		req, _ := http.NewRequest("POST", srv.URL+"/api/mission", strings.NewReader(body))
		req.Header.Set("Content-Type", contentType)
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

	if got := post("application/json", "https://evil.example", string(body)); got != http.StatusForbidden {
		t.Errorf("cross-origin = %d", got)
	}
	if got := post("text/plain", "", string(body)); got != http.StatusUnsupportedMediaType {
		t.Errorf("text/plain = %d", got)
	}
	if got := post("application/json", "", "{nope"); got != http.StatusBadRequest {
		t.Errorf("bad JSON = %d", got)
	}
	if got := post("application/json", "", `{"home":{"lat":1,"lon":1},"waypoints":[]}`); got != http.StatusBadRequest {
		t.Errorf("empty plan = %d", got)
	}
	if got := post("application/json", srv.URL, string(body)); got != http.StatusOK {
		t.Errorf("good = %d", got)
	}
}

func TestHTTPMissionRefusedWhileArmed(t *testing.T) {
	b, f, _ := setup(t)
	f.armed = true
	connect(t, b, f)
	srv := httptest.NewServer(b.Handler(""))
	defer srv.Close()

	body, _ := json.Marshal(model.DefaultPlan())
	resp, err := http.Post(srv.URL+"/api/mission", "application/json", strings.NewReader(string(body)))
	if err != nil {
		t.Fatal(err)
	}
	resp.Body.Close()
	if resp.StatusCode != http.StatusConflict {
		t.Fatalf("status = %d", resp.StatusCode)
	}
}

func TestHTTPStart(t *testing.T) {
	b, f, ctx := setup(t)
	go f.serve(ctx)
	connect(t, b, f)
	srv := httptest.NewServer(b.Handler(""))
	defer srv.Close()

	go func() {
		for i := 0; i < len(startFlightSteps); i++ {
			select {
			case fr := <-f.commands:
				f.ack(decodeCommand(fr), mavlink.ResultAccepted)
			case <-time.After(2 * time.Second):
				return
			}
		}
	}()

	resp, err := http.Post(srv.URL+"/api/start", "application/json", nil)
	if err != nil {
		t.Fatal(err)
	}
	resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		t.Fatalf("status = %d", resp.StatusCode)
	}
}

func TestHTTPStartRefusedWhileArmed(t *testing.T) {
	b, f, _ := setup(t)
	f.armed = true
	connect(t, b, f)
	srv := httptest.NewServer(b.Handler(""))
	defer srv.Close()

	resp, err := http.Post(srv.URL+"/api/start", "application/json", nil)
	if err != nil {
		t.Fatal(err)
	}
	resp.Body.Close()
	if resp.StatusCode != http.StatusConflict {
		t.Fatalf("status = %d", resp.StatusCode)
	}
}

func TestHTTPTelemetryStream(t *testing.T) {
	b, f, _ := setup(t)
	connect(t, b, f)
	srv := httptest.NewServer(b.Handler(""))
	defer srv.Close()

	resp, err := http.Get(srv.URL + "/api/telemetry")
	if err != nil {
		t.Fatal(err)
	}
	defer resp.Body.Close()
	if got := resp.Header.Get("Content-Type"); got != "text/event-stream" {
		t.Fatalf("content type %q", got)
	}

	scanner := bufio.NewScanner(resp.Body)
	var updates []model.Update
	for len(updates) < 2 && scanner.Scan() {
		data, ok := strings.CutPrefix(scanner.Text(), "data: ")
		if !ok {
			continue
		}
		var u model.Update
		if err := json.Unmarshal([]byte(data), &u); err != nil {
			t.Fatal(err)
		}
		updates = append(updates, u)
	}
	if len(updates) != 2 {
		t.Fatalf("got %d updates: %v", len(updates), scanner.Err())
	}
	if !updates[0].Snapshot.Connected || updates[0].Snapshot.Mode != "LOITER" {
		t.Errorf("snapshot: %+v", updates[0].Snapshot)
	}
	if len(updates[0].Events) == 0 {
		t.Error("first update should carry the event backlog")
	}
	if len(updates[1].Events) != 0 {
		t.Errorf("events repeated: %+v", updates[1].Events)
	}
}
