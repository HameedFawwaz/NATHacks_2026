package bridge

import (
	"context"
	"errors"
	"fmt"
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
	ErrNoVehicle    = errors.New("no drone connected")
	ErrArmed        = errors.New("the drone is armed; land before changing its mission")
	ErrAlreadyArmed = errors.New("the drone is already armed")
	ErrBusy         = errors.New("another upload or flight start is in progress")
	ErrNoMission    = errors.New("upload a mission successfully before starting the flight")
)

// Bridge talks MAVLink to one vehicle over UDP.
type Bridge struct {
	conn  *net.UDPConn
	start time.Time

	mu            sync.Mutex
	vehicle       vehicleState
	peer          *net.UDPAddr // where the autopilot's heartbeats come from
	seq           uint8
	upload        *upload
	missionReady  bool // a mission uploaded and acknowledged during this connection
	command       *command
	params        *paramWait         // a fault being injected; see fault.go
	stopDrift     context.CancelFunc // ends a drifting GPS fault, nil if none is running
	recovery      *recoveryWait
	recoveryNonce uint16
	resetting     bool            // serializes the fault-clear + core-reset transaction
	signer        *mavlink.Signer // nil: the link is unsigned
}

// signingLinkID identifies the dashboard's link in its signatures.
const signingLinkID = 0

// EnableSigning makes the bridge sign everything it sends and drop everything it
// receives that isn't signed with key. Call it before Run. With it, the dashboard
// can only talk to a vehicle that has the same key; without it, the dashboard
// can't talk to one that requires signing.
func (b *Bridge) EnableSigning(key [mavlink.KeyLen]byte) {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.signer = mavlink.NewSigner(key, signingLinkID)
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
	b.mu.Lock()
	defer b.mu.Unlock()
	var frames []mavlink.Frame
	if b.signer != nil {
		frames = b.signer.Parse(data)
	} else {
		frames = mavlink.Parse(data)
	}

	now := b.nowMs()
	b.updateLocked(now) // notice a stale connection before a returning heartbeat renews it
	for _, f := range frames {
		if b.vehicle.apply(f, now) {
			if f.MsgID == mavlink.MsgHeartbeat {
				b.peer = from
			}
			b.handleMission(f)
			b.handleCommand(f)
			b.handleParam(f)
		}
		b.handleRecoveryLocked(f)
	}
	b.updateLocked(now)
}

func (b *Bridge) updateLocked(now int64) {
	b.vehicle.update(now)
	if !b.vehicle.connected(now) {
		b.missionReady = false
		b.stopDriftLocked()
		if b.command != nil {
			b.command.finish(errors.New("lost contact with the drone while starting the flight"))
		}
	}
}

// sendLocked sends one message to the vehicle. Callers hold b.mu.
func (b *Bridge) sendLocked(msgID uint32, payload []byte) {
	if b.peer == nil {
		return
	}
	var frame []byte
	if b.signer != nil {
		frame = b.signer.Encode(b.seq, ownSysID, ownCompID, msgID, payload)
	} else {
		frame = mavlink.Encode(b.seq, ownSysID, ownCompID, msgID, payload)
	}
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
			b.updateLocked(b.nowMs())
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
	// Core's estimate is relative to home, and ArduPilot only sends the home
	// position when asked.
	b.sendLocked(mavlink.MsgCommandLong, mavlink.PackRequestMessage(
		b.vehicle.sysID, b.vehicle.compID, mavlink.MsgHomePosition))
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
	snap := b.vehicle.snapshot(b.nowMs())
	snap.MissionReady = b.missionReady && snap.Connected
	if b.signer != nil {
		st := b.signer.Stats()
		snap.Signing = model.LinkSecurity{
			Enabled:          true,
			RejectedUnsigned: st.RejectedUnsigned,
			RejectedBadSig:   st.RejectedBadSig,
			RejectedReplay:   st.RejectedReplay,
		}
	}
	return snap, events
}

// UploadPlan sends a plan to the vehicle and waits for it to be accepted. An
// unconfirmed transfer is sent again once, which is safe because it fully replaces
// the mission and the vehicle must still be disarmed.
func (b *Bridge) UploadPlan(ctx context.Context, plan model.Plan) error {
	if err := ValidatePlan(plan); err != nil {
		return err
	}
	items := BuildMission(plan)
	err := b.transfer(ctx, items)
	if errors.Is(err, errUnconfirmed) {
		err = b.transfer(ctx, items)
	}
	if errors.Is(err, errUnconfirmed) {
		return errors.New("the drone didn't confirm the mission; what it has stored is unknown, so upload again")
	}
	return err
}

func (b *Bridge) transfer(ctx context.Context, items []mavlink.MissionItem) error {
	// Checked first because MISSION_COUNT alone can clear the stored mission.
	if err := ctx.Err(); err != nil {
		return err
	}

	u := &upload{
		items:    items,
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
	case b.upload != nil || b.command != nil || b.recovery != nil || b.resetting:
		b.mu.Unlock()
		return ErrBusy
	}
	b.upload = u
	b.missionReady = false // even MISSION_COUNT can clear an earlier mission
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

	retry := time.NewTimer(uploadRetryInterval)
	defer retry.Stop()
	retries := 0
	for {
		select {
		case err := <-u.done:
			if err == nil {
				b.mu.Lock()
				b.missionReady = b.vehicle.connected(b.nowMs())
				b.mu.Unlock()
			}
			return err
		case <-u.progress:
			retries = 0
			retry.Reset(uploadRetryInterval)
		case <-retry.C:
			retries++
			if retries > uploadMaxRetries {
				b.cancelUpload()
				if u.next == len(u.items) {
					return errUnconfirmed
				}
				return errors.New("the drone stopped responding during the upload")
			}
			b.mu.Lock()
			if err := b.uploadBlockedLocked(); err != nil {
				b.cancelUploadLocked()
				b.mu.Unlock()
				return err
			}
			u.resent = u.next == len(u.items)
			u.lastSent()
			b.mu.Unlock()
			retry.Reset(uploadRetryInterval)
		case <-ctx.Done():
			b.cancelUpload()
			return ctx.Err()
		}
	}
}

// uploadBlockedLocked says why a transfer under way must stop, if it must.
func (b *Bridge) uploadBlockedLocked() error {
	switch {
	case !b.vehicle.connected(b.nowMs()):
		return errors.New("lost contact with the drone during the upload; its mission may be incomplete")
	case b.vehicle.snap.Armed:
		return errors.New("the drone was armed during the upload; its mission may be incomplete")
	}
	return nil
}

func (b *Bridge) cancelUpload() {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.cancelUploadLocked()
}

// cancelUploadLocked tells the vehicle to stop waiting for items. It's best effort:
// ArduPilot ignores this and waits for its own timeout, which doesn't stop this
// bridge starting a new transfer but can briefly refuse another ground station's.
func (b *Bridge) cancelUploadLocked() {
	b.sendLocked(mavlink.MsgMissionAck, mavlink.PackMissionAck(b.vehicle.sysID, b.vehicle.compID, mavlink.MissionCancelled))
}

// StartFlight arms the vehicle and starts the uploaded mission: GUIDED, arm,
// AUTO, MISSION_START. Mode/arming steps need both an accepted COMMAND_ACK and
// a subsequent confirming heartbeat; an ACK alone is not proof of vehicle state.
func (b *Bridge) StartFlight(ctx context.Context) error {
	if err := ctx.Err(); err != nil {
		return err
	}

	c := &command{
		ctx:      ctx,
		steps:    startFlightSteps,
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
		return ErrAlreadyArmed
	case b.command != nil || b.upload != nil || b.recovery != nil || b.resetting:
		b.mu.Unlock()
		return ErrBusy
	case !b.missionReady:
		b.mu.Unlock()
		return ErrNoMission
	}
	b.command = c
	b.sendCommandStepLocked(c)
	b.mu.Unlock()

	defer func() {
		b.mu.Lock()
		b.command = nil
		b.mu.Unlock()
	}()

	retry := time.NewTimer(uploadRetryInterval)
	defer retry.Stop()
	retries := 0
	for {
		select {
		case err := <-c.done:
			return err
		case <-c.progress:
			retries = 0
			retry.Reset(uploadRetryInterval)
		case <-retry.C:
			b.mu.Lock()
			if c.finished {
				b.mu.Unlock()
				continue
			}
			retries++
			if retries > uploadMaxRetries {
				name := commandName(c.steps[c.index].command)
				err := fmt.Errorf("the drone stopped responding while %s", name)
				c.finish(err)
				b.mu.Unlock()
				return err
			}
			if !b.vehicle.connected(b.nowMs()) {
				err := errors.New("lost contact with the drone while starting the flight")
				c.finish(err)
				b.mu.Unlock()
				return err
			}
			c.lastSent()
			b.mu.Unlock()
			retry.Reset(uploadRetryInterval)
		case <-ctx.Done():
			return ctx.Err()
		}
	}
}

// sendCommandStepLocked sends the command sequence's current step. Callers hold b.mu.
func (b *Bridge) sendCommandStepLocked(c *command) {
	c.acknowledged = false
	c.heartbeatSeq = b.vehicle.heartbeatSeq
	step := c.steps[c.index]
	sys, comp := b.vehicle.sysID, b.vehicle.compID
	c.lastSent = func() {
		b.sendLocked(mavlink.MsgCommandLong, mavlink.PackCommandLong(sys, comp, step.command, step.param1, step.param2))
	}
	c.lastSent()
}

// handleCommand advances a command sequence in progress. Callers hold b.mu.
func (b *Bridge) handleCommand(f mavlink.Frame) {
	c := b.command
	if c == nil || c.finished || (c.ctx != nil && c.ctx.Err() != nil) {
		return
	}
	if f.MsgID == mavlink.MsgHeartbeat {
		if c.index >= 2 && !b.vehicle.snap.Armed {
			c.finish(errors.New("the drone disarmed while starting the flight"))
			return
		}
		b.advanceCommandLocked(c)
		return
	}
	if f.MsgID != mavlink.MsgCommandAck {
		return
	}
	cmd, result, targetSys, targetComp := mavlink.DecodeCommandAck(f.Payload)
	if targetSys != ownSysID || targetComp != ownCompID || cmd != c.steps[c.index].command {
		return // not an ack for the step currently waited on
	}

	switch result {
	case mavlink.ResultInProgress:
		// Still working on it; treat as progress so it isn't resent mid-flight.
	case mavlink.ResultAccepted:
		c.acknowledged = true
		b.advanceCommandLocked(c)
	default:
		c.finish(commandResultError(cmd, result))
		return
	}
	select {
	case c.progress <- struct{}{}:
	default:
	}
}

func (b *Bridge) advanceCommandLocked(c *command) {
	if !c.acknowledged || !b.vehicle.connected(b.nowMs()) {
		return
	}
	step := c.steps[c.index]
	if step.command != mavlink.CmdMissionStart {
		if b.vehicle.heartbeatSeq <= c.heartbeatSeq {
			return
		}
		switch step.command {
		case mavlink.CmdDoSetMode:
			if b.vehicle.customMode != uint32(step.param2) {
				return
			}
		case mavlink.CmdComponentArmDisarm:
			if !b.vehicle.snap.Armed {
				return
			}
		}
	}
	c.index++
	if c.index == len(c.steps) {
		c.finish(nil)
		return
	}
	b.sendCommandStepLocked(c)
}

// handleMission advances an upload in progress. Callers hold b.mu.
func (b *Bridge) handleMission(f mavlink.Frame) {
	u := b.upload
	if u == nil {
		return
	}
	if f.MsgID != mavlink.MsgMissionRequest && f.MsgID != mavlink.MsgMissionRequestInt && f.MsgID != mavlink.MsgMissionAck {
		return
	}

	switch f.MsgID {
	case mavlink.MsgMissionRequest, mavlink.MsgMissionRequestInt:
		seq, targetSys, targetComp, missionType := mavlink.DecodeMissionRequest(f.Payload)
		if targetSys != ownSysID || targetComp != ownCompID || missionType != mavlink.MissionTypeMission {
			return
		}
		// Only sending more is unsafe. An ack that arrives after arming can still
		// confirm a mission that was fully stored before it.
		if err := b.uploadBlockedLocked(); err != nil {
			b.cancelUploadLocked()
			u.finish(err)
			return
		}
		// Items go out strictly in order. A request from further ahead can't belong
		// to this transfer, while an earlier one is a retry.
		if int(seq) > u.next || int(seq) >= len(u.items) {
			return
		}
		sys, comp := b.vehicle.sysID, b.vehicle.compID
		u.lastSent = func() {
			b.sendLocked(mavlink.MsgMissionItemInt, mavlink.PackMissionItemInt(sys, comp, seq, u.items[seq]))
		}
		u.lastSent()
		if int(seq) == u.next {
			u.next++
		}
		select {
		case u.progress <- struct{}{}:
		default:
		}
	case mavlink.MsgMissionAck:
		targetSys, targetComp, result, missionType := mavlink.DecodeMissionAck(f.Payload)
		if targetSys != ownSysID || targetComp != ownCompID || missionType != mavlink.MissionTypeMission {
			return
		}
		allSent := u.next == len(u.items)
		switch {
		case result == mavlink.MissionAccepted && allSent:
			u.finish(nil)
		case result == mavlink.MissionAccepted:
			// Can't be for this transfer; leave it to time out rather than call it a success.
		case allSent && u.resent:
			u.finish(errUnconfirmed)
		default:
			u.finish(missionResultError(result))
		}
	}
}
