// Package mavlink is the small part of the MAVLink protocol the dashboard needs:
// frame parsing and encoding, plus the handful of messages it reads and sends.
package mavlink

import (
	"encoding/binary"
	"math"
)

// Message IDs.
const (
	MsgHeartbeat         = 0
	MsgSysStatus         = 1
	MsgGPSRawInt         = 24
	MsgGlobalPositionInt = 33
	MsgMissionRequest    = 40
	MsgMissionCount      = 44
	MsgMissionAck        = 47
	MsgMissionRequestInt = 51
	MsgRequestDataStream = 66
	MsgMissionItemInt    = 73
	MsgNamedValueFloat   = 251
	MsgStatusText        = 253
)

const (
	magicV1 = 0xFE
	magicV2 = 0xFD

	headerLenV1 = 6
	headerLenV2 = 10
	crcLen      = 2
	sigLen      = 13
	flagSigned  = 0x01
)

// messageInfo holds each message's CRC extra and full payload length, from the
// generated C headers in core/third_party/mavlink.
var messageInfo = map[uint32]struct {
	crcExtra byte
	length   int
}{
	MsgHeartbeat:         {50, 9},
	MsgSysStatus:         {124, 43},
	MsgGPSRawInt:         {24, 52},
	MsgGlobalPositionInt: {104, 28},
	MsgMissionRequest:    {230, 5},
	MsgMissionCount:      {221, 9},
	MsgMissionAck:        {153, 8},
	MsgMissionRequestInt: {196, 5},
	MsgRequestDataStream: {148, 6},
	MsgMissionItemInt:    {38, 38},
	MsgNamedValueFloat:   {170, 18},
	MsgStatusText:        {83, 54},
}

// Frame is one received message. Payload is always the message's full length,
// because MAVLink 2 trims trailing zeros on the wire.
type Frame struct {
	SysID, CompID uint8
	MsgID         uint32
	Payload       []byte
}

// Parse returns the valid frames of the messages this package knows about.
// Everything else in data, including corrupt frames, is skipped.
func Parse(data []byte) []Frame {
	var frames []Frame
	for len(data) > 0 {
		frame, size, ok := parseOne(data)
		if ok {
			frames = append(frames, frame)
		}
		data = data[size:]
	}
	return frames
}

// parseOne reads the frame at the start of data and returns how many bytes to
// advance. Anything unverifiable only advances one byte, because the magic byte
// may have been noise with a real frame starting inside it.
func parseOne(data []byte) (frame Frame, size int, ok bool) {
	var headerLen, payloadLen, extra int
	var msgID uint32
	switch data[0] {
	case magicV1:
		if len(data) < headerLenV1+crcLen {
			return Frame{}, len(data), false
		}
		headerLen, payloadLen = headerLenV1, int(data[1])
		msgID = uint32(data[5])
		frame.SysID, frame.CompID = data[3], data[4]
	case magicV2:
		if len(data) < headerLenV2+crcLen {
			return Frame{}, len(data), false
		}
		headerLen, payloadLen = headerLenV2, int(data[1])
		if data[2]&flagSigned != 0 {
			extra = sigLen
		}
		msgID = uint32(data[7]) | uint32(data[8])<<8 | uint32(data[9])<<16
		frame.SysID, frame.CompID = data[5], data[6]
	default:
		return Frame{}, 1, false
	}

	total := headerLen + payloadLen + crcLen + extra
	if len(data) < total {
		return Frame{}, 1, false
	}
	info, known := messageInfo[msgID]
	if !known {
		return Frame{}, 1, false
	}

	crcEnd := headerLen + payloadLen
	want := binary.LittleEndian.Uint16(data[crcEnd:])
	if crc(data[1:crcEnd], info.crcExtra) != want {
		return Frame{}, 1, false
	}

	frame.MsgID = msgID
	frame.Payload = make([]byte, max(info.length, payloadLen))
	copy(frame.Payload, data[headerLen:crcEnd])
	return frame, total, true
}

// Encode returns a MAVLink 2 frame. The message must be one this package knows.
func Encode(seq, sysID, compID uint8, msgID uint32, payload []byte) []byte {
	n := len(payload)
	for n > 1 && payload[n-1] == 0 { // MAVLink 2 trims trailing zeros, keeping at least one byte
		n--
	}
	out := make([]byte, headerLenV2, headerLenV2+n+crcLen)
	out[0] = magicV2
	out[1] = byte(n)
	out[4] = seq
	out[5] = sysID
	out[6] = compID
	out[7], out[8], out[9] = byte(msgID), byte(msgID>>8), byte(msgID>>16)
	out = append(out, payload[:n]...)
	return binary.LittleEndian.AppendUint16(out, crc(out[1:], messageInfo[msgID].crcExtra))
}

// crc is MAVLink's CRC-16/MCRF4XX, with the message's CRC extra appended.
func crc(data []byte, extra byte) uint16 {
	sum := uint16(0xFFFF)
	add := func(b byte) {
		tmp := b ^ byte(sum)
		tmp ^= tmp << 4
		sum = sum>>8 ^ uint16(tmp)<<8 ^ uint16(tmp)<<3 ^ uint16(tmp)>>4
	}
	for _, b := range data {
		add(b)
	}
	add(extra)
	return sum
}

func u16(b []byte) uint16 { return binary.LittleEndian.Uint16(b) }
func u32(b []byte) uint32 { return binary.LittleEndian.Uint32(b) }
func i32(b []byte) int32  { return int32(binary.LittleEndian.Uint32(b)) }
func f32(b []byte) float32 {
	return math.Float32frombits(binary.LittleEndian.Uint32(b))
}
