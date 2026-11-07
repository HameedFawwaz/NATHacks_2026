package bridge

import (
	"context"
	"errors"
	"log"
	"net"
	"sync"
	"time"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/mavlink"
	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

const (
	// The dashboard's MAVLink identity. 254 rather than the usual 255 so it never
	// counts as the operator's primary ground station for GCS failsafes.
	ownSysID  = 254
	ownCompID = 190 // MAV_COMP_ID_MISSIONPLANNER

	heartbeatInterval = time.Second
	streamInterval    = 5 * time.Second

	// Below SITL's 5 Hz GPS: asking for faster made ArduPilot repeat cached fixes,
	// which trips the replay check in core's GNSS detector (see PR #10).
	streamRateHz = 2

	uploadRetryInterval = 1500 * time.Millisecond
	uploadMaxRetries    = 5
)

var (
	ErrNoVehicle = errors.New("no drone connected")
	ErrArmed     = errors.New("the drone is armed; land before changing its mission")
	ErrBusy      = errors.New("another upload is in progress")
)

// Bridge talks MAVLink to one vehicle over UDP.
type Bridge struct {
	conn  *net.UDPConn
	start time.Time

	mu      sync.Mutex
	vehicle vehicleState
	peer    *net.UDPAddr // where the autopilot's heartbeats come from
	seq     uint8
	upload  *upload
}

// Listen opens the UDP socket a vehicle or MAVProxy sends telemetry to.
func Listen(addr string) (*Bridge, error) {
	udpAddr, err := net.ResolveUDPAddr("udp", addr)
	if err != nil {
		return nil, err
	}
	conn, err := net.ListenUDP("udp", udpAddr)
	if err != nil {
		return nil, err
	}
	return &Bridge{conn: conn, start: time.Now()}, nil
}

// Addr returns the UDP address being listened on.
func (b *Bridge) Addr() net.Addr { return b.conn.LocalAddr() }

// Run serves until ctx is cancelled.
func (b *Bridge) Run(ctx context.Context) error {
	go b.sendPeriodic(ctx)
	go func() {
		<-ctx.Done()
		b.conn.Close()
	}()

	buf := make([]byte, 65535)
	for {
		n, from, err := b.conn.ReadFromUDP(buf)
		if err != nil {
			if ctx.Err() != nil {
				return nil
			}
			return err
		}
		b.handle(buf[:n], from)
	}
}

func (b *Bridge) nowMs() int64 { return time.Since(b.start).Milliseconds() + 1 }

func (b *Bridge) handle(data []byte, from *net.UDPAddr) {
	frames := mavlink.Parse(data)
	b.mu.Lock()
	defer b.mu.Unlock()

	now := b.nowMs()
	for _, f := range frames {
		if b.vehicle.apply(f, now) {
			if f.MsgID == mavlink.MsgHeartbeat {
				b.peer = from
			}
			b.handleMission(f)
		}
	}
	b.vehicle.update(now)
}

// sendLocked sends one message to the vehicle. Callers hold b.mu.
func (b *Bridge) sendLocked(msgID uint32, payload []byte) {
	if b.peer == nil {
		return
	}
	frame := mavlink.Encode(b.seq, ownSysID, ownCompID, msgID, payload)
	b.seq++
	if _, err := b.conn.WriteToUDP(frame, b.peer); err != nil {
		log.Printf("mavlink send: %v", err)
	}
}

// sendPeriodic keeps the vehicle aware of the dashboard. ArduPilot only forwards
// other components' messages, such as core's trust scores, to a channel it has
// heard from, so the heartbeat is needed even though nothing here commands flight.
func (b *Bridge) sendPeriodic(ctx context.Context) {
	heartbeat := time.NewTicker(heartbeatInterval)
	defer heartbeat.Stop()
	streams := time.NewTicker(streamInterval)
	defer streams.Stop()

	requestedStreams := false
	for {
		select {
		case <-ctx.Done():
			return
		case <-heartbeat.C:
			b.mu.Lock()
			b.sendLocked(mavlink.MsgHeartbeat, mavlink.PackGCSHeartbeat())
			b.vehicle.update(b.nowMs())
			if b.vehicle.identified && !requestedStreams {
				requestedStreams = true
				b.requestStreamsLocked()
			}
			b.mu.Unlock()
		case <-streams.C:
			b.mu.Lock()
			if b.vehicle.identified {
				b.requestStreamsLocked()
			}
			b.mu.Unlock()
		}
	}
}

func (b *Bridge) requestStreamsLocked() {
	b.sendLocked(mavlink.MsgRequestDataStream, mavlink.PackRequestDataStream(
		b.vehicle.sysID, b.vehicle.compID, mavlink.DataStreamAll, streamRateHz))
}

// Snapshot returns the latest vehicle state, with events after afterSeq.
func (b *Bridge) Snapshot(afterSeq int) (model.Snapshot, []model.Event) {
	b.mu.Lock()
	defer b.mu.Unlock()

	var events []model.Event
	for _, e := range b.vehicle.events {
		if e.Seq > afterSeq {
			events = append(events, e)
		}
	}
	return b.vehicle.snapshot(b.nowMs()), events
}

// UploadPlan sends a plan to the vehicle and waits for it to be accepted.
func (b *Bridge) UploadPlan(ctx context.Context, plan model.Plan) error {
	if err := ValidatePlan(plan); err != nil {
		return err
	}

	u := &upload{
		items:    BuildMission(plan),
		progress: make(chan struct{}, 1),
		done:     make(chan error, 1),
	}

	b.mu.Lock()
	now := b.nowMs()
	switch {
	case !b.vehicle.connected(now) || b.peer == nil:
		b.mu.Unlock()
		return ErrNoVehicle
	case b.vehicle.snap.Armed:
		b.mu.Unlock()
		return ErrArmed
	case b.upload != nil:
		b.mu.Unlock()
		return ErrBusy
	}
	b.upload = u
	sys, comp := b.vehicle.sysID, b.vehicle.compID
	u.lastSent = func() {
		b.sendLocked(mavlink.MsgMissionCount, mavlink.PackMissionCount(sys, comp, uint16(len(u.items))))
	}
	u.lastSent()
	b.mu.Unlock()

	defer func() {
		b.mu.Lock()
		b.upload = nil
		b.mu.Unlock()
	}()

	retry := time.NewTicker(uploadRetryInterval)
	defer retry.Stop()
	retries := 0
	for {
		select {
		case err := <-u.done:
			return err
		case <-u.progress:
			retries = 0
		case <-retry.C:
			retries++
			if retries > uploadMaxRetries {
				return errors.New("the drone stopped responding during the upload")
			}
			b.mu.Lock()
			u.lastSent()
			b.mu.Unlock()
		case <-ctx.Done():
			return ctx.Err()
		}
	}
}

// handleMission advances an upload in progress. Callers hold b.mu.
func (b *Bridge) handleMission(f mavlink.Frame) {
	u := b.upload
	if u == nil {
		return
	}
	switch f.MsgID {
	case mavlink.MsgMissionRequest, mavlink.MsgMissionRequestInt:
		seq, targetSys, targetComp, missionType := mavlink.DecodeMissionRequest(f.Payload)
		if targetSys != ownSysID || targetComp != ownCompID || missionType != mavlink.MissionTypeMission {
			return
		}
		if int(seq) >= len(u.items) {
			u.finish(missionResultError(13))
			return
		}
		sys, comp := b.vehicle.sysID, b.vehicle.compID
		u.lastSent = func() {
			b.sendLocked(mavlink.MsgMissionItemInt, mavlink.PackMissionItemInt(sys, comp, seq, u.items[seq]))
		}
		u.lastSent()
		u.sawLast = int(seq) == len(u.items)-1
		select {
		case u.progress <- struct{}{}:
		default:
		}
	case mavlink.MsgMissionAck:
		targetSys, targetComp, result, missionType := mavlink.DecodeMissionAck(f.Payload)
		if targetSys != ownSysID || targetComp != ownCompID || missionType != mavlink.MissionTypeMission {
			return
		}
		switch {
		case result != mavlink.MissionAccepted:
			u.finish(missionResultError(result))
		case u.sawLast:
			u.finish(nil)
		}
		// An "accepted" before the last item was requested is left to time out
		// rather than reported as a success.
	}
}
