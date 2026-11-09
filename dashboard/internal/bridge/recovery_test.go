package bridge

import (
	"context"
	"encoding/binary"
	"math"
	"net"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/mavlink"
	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

func TestRecoveryStatusIdentityFreshnessAndValidation(t *testing.T) {
	v := identified(t)
	for _, comp := range []uint8{1, 190} {
		v.apply(frame(1, comp, mavlink.MsgNamedValueFloat, namedValue("RECOV_EN", 1)), 1100)
	}
	v.apply(frame(2, 191, mavlink.MsgNamedValueFloat, namedValue("RECOV_EN", 1)), 1100)
	if v.snap.RecoveryAtMs != 0 {
		t.Fatal("non-core component supplied recovery status")
	}
	for _, value := range []float32{float32(math.NaN()), float32(math.Inf(1)), 0.5, 2} {
		v.apply(frame(1, 191, mavlink.MsgNamedValueFloat, namedValue("RECOV_EN", value)), 1100)
	}
	if v.snap.RecoveryAtMs != 0 {
		t.Fatal("invalid status accepted")
	}
	v.apply(frame(1, 191, mavlink.MsgNamedValueFloat, namedValue("RECOV_EN", 1)), 1100)
	v.apply(frame(1, 191, mavlink.MsgNamedValueFloat, namedValue("RECOV_CTL", 1)), 1100)
	v.apply(frame(1, 191, mavlink.MsgNamedValueFloat, namedValue("AUTO_STATE", 1)), 1100)
	if s := v.snapshot(1100); !s.RecoveryEnabled || !s.RecoveryControllable || s.Behavior != model.BehaviorGNSSFailover {
		t.Fatalf("state: %+v", s)
	}
	later := int64(1100 + model.StaleAfterMs + 1)
	v.apply(frame(1, 1, mavlink.MsgHeartbeat, heartbeatPayload(2, 3, 0, 0)), later)
	v.apply(frame(1, 191, mavlink.MsgNamedValueFloat, namedValue("RECOV_EN", 1)), later)
	if v.snapshot(later).RecoveryControllable {
		t.Fatal("stale capability remained actionable")
	}
}

func TestRecoveryWaitsForCoreNonceConfirmation(t *testing.T) {
	b, err := Listen("127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	defer b.conn.Close()
	peer, err := net.ListenUDP("udp", &net.UDPAddr{IP: net.ParseIP("127.0.0.1")})
	if err != nil {
		t.Fatal(err)
	}
	defer peer.Close()
	addr := peer.LocalAddr().(*net.UDPAddr)
	apply := func(comp uint8, name string, value float32) {
		b.handle(mavlink.Encode(0, 1, comp, mavlink.MsgNamedValueFloat, namedValue(name, value)), addr)
	}
	b.handle(mavlink.Encode(0, 1, 1, mavlink.MsgHeartbeat, heartbeatPayload(2, 3, 0, 0)), addr)
	apply(191, "RECOV_EN", 1)
	apply(191, "RECOV_CTL", 1)
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	done := make(chan error, 1)
	go func() { done <- b.SetRecovery(ctx, false) }()
	peer.SetReadDeadline(time.Now().Add(time.Second))
	buf := make([]byte, 512)
	n, _, err := peer.ReadFromUDP(buf)
	if err != nil {
		t.Fatal(err)
	}
	f := mavlink.Parse(buf[:n])[0]
	if f.MsgID != mavlink.MsgCommandLong || f.Payload[30] != 1 || f.Payload[31] != 191 ||
		binary.LittleEndian.Uint16(f.Payload[28:]) != recoveryCommand || math.Float32frombits(binary.LittleEndian.Uint32(f.Payload)) != 0 {
		t.Fatalf("wrong command: %+v", f)
	}
	nonce := uint32(math.Float32frombits(binary.LittleEndian.Uint32(f.Payload[4:])))
	if nonce == 0 {
		t.Fatal("missing request nonce")
	}
	if snap, _ := b.Snapshot(0); !snap.RecoveryEnabled {
		t.Fatal("request optimistically changed status")
	}
	apply(1, "RECOV_ACK", float32(nonce*2))     // autopilot cannot acknowledge a core setting
	apply(191, "RECOV_EN", 0)                   // a matching status is not a request acknowledgement
	apply(191, "RECOV_ACK", float32(nonce*2+1)) // wrong requested state
	select {
	case err := <-done:
		t.Fatalf("unconfirmed request completed: %v", err)
	default:
	}
	apply(191, "RECOV_ACK", float32(nonce*2))
	if err := <-done; err != nil {
		t.Fatal(err)
	}
	if snap, _ := b.Snapshot(0); snap.RecoveryEnabled {
		t.Fatal("core's disabled state missing")
	}
}

func TestRecoveryHTTPGuards(t *testing.T) {
	b, err := Listen("127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	defer b.conn.Close()
	h := b.Handler(HandlerOptions{})
	for _, tc := range []struct {
		name, remote, host, origin, body, content string
		code                                      int
	}{
		{"remote", "192.0.2.1:99", "127.0.0.1:8080", "", `{"enabled":false}`, "application/json", 403},
		{"origin", "127.0.0.1:99", "127.0.0.1:8080", "https://evil.example", `{"enabled":false}`, "application/json", 403},
		{"host", "127.0.0.1:99", "evil.example", "", `{"enabled":false}`, "application/json", 403},
		{"content", "127.0.0.1:99", "127.0.0.1:8080", "", `{"enabled":false}`, "text/plain", 415},
		{"missing", "127.0.0.1:99", "127.0.0.1:8080", "", `{}`, "application/json", 400},
		{"trailing", "127.0.0.1:99", "127.0.0.1:8080", "", `{"enabled":false}{}`, "application/json", 400},
		{"unknown", "127.0.0.1:99", "127.0.0.1:8080", "", `{"enabled":false,"oops":1}`, "application/json", 400},
		{"no vehicle", "127.0.0.1:99", "127.0.0.1:8080", "", `{"enabled":false}`, "application/json", 409},
	} {
		t.Run(tc.name, func(t *testing.T) {
			r := httptest.NewRequest("POST", "http://127.0.0.1:8080/api/recovery", strings.NewReader(tc.body))
			r.RemoteAddr, r.Host = tc.remote, tc.host
			r.Header.Set("Content-Type", tc.content)
			if tc.origin != "" {
				r.Header.Set("Origin", tc.origin)
			}
			w := httptest.NewRecorder()
			h.ServeHTTP(w, r)
			if w.Code != tc.code {
				t.Fatalf("got %d: %s", w.Code, w.Body.String())
			}
		})
	}
}
