package bridge

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"mime"
	"net"
	"net/http"
	"net/url"
	"slices"
	"strings"
	"time"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

const (
	streamInterval5Hz = 200 * time.Millisecond
	uploadTimeout     = 30 * time.Second
	maxPlanBytes      = 1 << 20
)

// Status answers whether a bridge is running, which is how the page tells live
// mode from simulation.
type Status struct {
	Listen string `json:"listen"`
}

// Handler serves the bridge's API under /api/. allowedHosts are the names or
// host:port pairs the dashboard is reached by, besides localhost.
func (b *Bridge) Handler(allowedHosts ...string) http.Handler {
	mux := http.NewServeMux()
	mux.HandleFunc("GET /api/status", func(w http.ResponseWriter, r *http.Request) {
		writeJSON(w, http.StatusOK, Status{Listen: b.Addr().String()})
	})
	mux.HandleFunc("GET /api/telemetry", b.serveTelemetry)
	mux.HandleFunc("POST /api/mission", b.serveMission)
	mux.HandleFunc("POST /api/start", b.serveStart)
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if !allowedHost(r.Host, allowedHosts) {
			writeJSON(w, http.StatusForbidden, map[string]string{"error": "unexpected Host"})
			return
		}
		mux.ServeHTTP(w, r)
	})
}

// allowedHost defeats DNS rebinding, where an attacker's domain resolves to this
// machine so that its page counts as same-origin with the API.
func allowedHost(host string, allowed []string) bool {
	name, _, err := net.SplitHostPort(host)
	if err != nil {
		name = strings.Trim(host, "[]")
	}
	switch name {
	case "localhost", "127.0.0.1", "::1":
		return true
	}
	return slices.Contains(allowed, host) || slices.Contains(allowed, name)
}

func writeJSON(w http.ResponseWriter, status int, value any) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	json.NewEncoder(w).Encode(value)
}

// serveTelemetry streams Updates as server-sent events.
func (b *Bridge) serveTelemetry(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "text/event-stream")
	w.Header().Set("Cache-Control", "no-store")
	flusher := http.NewResponseController(w)

	ticker := time.NewTicker(streamInterval5Hz)
	defer ticker.Stop()
	lastSeq := 0
	for {
		snap, events := b.Snapshot(lastSeq)
		if n := len(events); n > 0 {
			lastSeq = events[n-1].Seq
		}
		data, err := json.Marshal(model.Update{Snapshot: snap, Events: events})
		if err != nil {
			return
		}
		if _, err := fmt.Fprintf(w, "data: %s\n\n", data); err != nil {
			return
		}
		if err := flusher.Flush(); err != nil {
			return
		}
		select {
		case <-r.Context().Done():
			return
		case <-ticker.C:
		}
	}
}

// sameOrigin stops other websites from driving this endpoint from the operator's
// browser: a page on any site can otherwise POST to localhost.
func sameOrigin(r *http.Request) bool {
	origin := r.Header.Get("Origin")
	if origin == "" {
		return true // not sent by a browser page
	}
	u, err := url.Parse(origin)
	return err == nil && u.Host == r.Host
}

func (b *Bridge) serveMission(w http.ResponseWriter, r *http.Request) {
	fail := func(status int, message string) {
		writeJSON(w, status, map[string]string{"error": message})
	}
	if !sameOrigin(r) {
		fail(http.StatusForbidden, "cross-origin request refused")
		return
	}
	if media, _, _ := mime.ParseMediaType(r.Header.Get("Content-Type")); media != "application/json" {
		fail(http.StatusUnsupportedMediaType, "expected application/json")
		return
	}

	var plan model.Plan
	if err := json.NewDecoder(http.MaxBytesReader(w, r.Body, maxPlanBytes)).Decode(&plan); err != nil {
		fail(http.StatusBadRequest, "invalid plan: "+err.Error())
		return
	}
	if err := ValidatePlan(plan); err != nil {
		fail(http.StatusBadRequest, err.Error())
		return
	}

	ctx, cancel := context.WithTimeout(r.Context(), uploadTimeout)
	defer cancel()
	switch err := b.UploadPlan(ctx, plan); {
	case err == nil:
		writeJSON(w, http.StatusOK, map[string]int{"items": len(BuildMission(plan))})
	case errors.Is(err, ErrNoVehicle), errors.Is(err, ErrArmed), errors.Is(err, ErrBusy):
		fail(http.StatusConflict, err.Error())
	default:
		fail(http.StatusBadGateway, err.Error())
	}
}

func (b *Bridge) serveStart(w http.ResponseWriter, r *http.Request) {
	fail := func(status int, message string) {
		writeJSON(w, status, map[string]string{"error": message})
	}
	if !sameOrigin(r) {
		fail(http.StatusForbidden, "cross-origin request refused")
		return
	}

	ctx, cancel := context.WithTimeout(r.Context(), uploadTimeout)
	defer cancel()
	switch err := b.StartFlight(ctx); {
	case err == nil:
		writeJSON(w, http.StatusOK, map[string]bool{"started": true})
	case errors.Is(err, ErrNoVehicle), errors.Is(err, ErrAlreadyArmed), errors.Is(err, ErrBusy):
		fail(http.StatusConflict, err.Error())
	default:
		fail(http.StatusBadGateway, err.Error())
	}
}
