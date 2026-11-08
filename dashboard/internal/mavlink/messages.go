package mavlink

import (
	"bytes"
	"encoding/binary"
	"math"
)

// Values from the MAVLink common message set.
const (
	AutopilotArduPilot = 3
	AutopilotInvalid   = 8

	TypeGCS               = 6
	TypeOnboardController = 18

	ModeFlagArmed = 128

	SensorGyro  = 1 << 0
	SensorAccel = 1 << 1

	GPSFix2D = 2

	CmdNavWaypoint = 16
	CmdNavRTL      = 20
	CmdNavTakeoff  = 22

	FrameGlobal            = 0
	FrameGlobalRelativeAlt = 3

	MissionTypeMission = 0
	MissionAccepted    = 0
	MissionCancelled   = 14

	DataStreamAll = 0
)

// Field values meaning "unknown".
const (
	UnknownU16             = math.MaxUint16
	UnknownU8              = math.MaxUint8
	UnknownBattery         = -1
	degE7          float64 = 1e7
)

type Heartbeat struct {
	CustomMode uint32
	Type       uint8
	Autopilot  uint8
	BaseMode   uint8
}

func DecodeHeartbeat(p []byte) Heartbeat {
	return Heartbeat{CustomMode: u32(p), Type: p[4], Autopilot: p[5], BaseMode: p[6]}
}

type SysStatus struct {
	SensorsPresent, SensorsEnabled, SensorsHealth uint32
	BatteryRemaining                              int8 // percent, UnknownBattery if unknown
}

func DecodeSysStatus(p []byte) SysStatus {
	return SysStatus{
		SensorsPresent:   u32(p),
		SensorsEnabled:   u32(p[4:]),
		SensorsHealth:    u32(p[8:]),
		BatteryRemaining: int8(p[30]),
	}
}

type GPSRawInt struct {
	Lat, Lon   float64 // degrees
	FixType    uint8
	Satellites uint8 // UnknownU8 if unknown
}

func DecodeGPSRawInt(p []byte) GPSRawInt {
	return GPSRawInt{
		Lat:        float64(i32(p[8:])) / degE7,
		Lon:        float64(i32(p[12:])) / degE7,
		FixType:    p[28],
		Satellites: p[29],
	}
}

type GlobalPositionInt struct {
	Lat, Lon      float64 // degrees
	RelativeAltM  float64
	VNorth, VEast float64 // m/s
	HeadingCdeg   uint16  // UnknownU16 if unknown
}

func DecodeGlobalPositionInt(p []byte) GlobalPositionInt {
	return GlobalPositionInt{
		Lat:          float64(i32(p[4:])) / degE7,
		Lon:          float64(i32(p[8:])) / degE7,
		RelativeAltM: float64(i32(p[16:])) / 1000,
		VNorth:       float64(int16(u16(p[20:]))) / 100,
		VEast:        float64(int16(u16(p[22:]))) / 100,
		HeadingCdeg:  u16(p[26:]),
	}
}

func DecodeNamedValueFloat(p []byte) (name string, value float64) {
	return cString(p[8:18]), float64(f32(p[4:]))
}

func DecodeStatusText(p []byte) (severity uint8, text string) {
	return p[0], cString(p[1:51])
}

// DecodeMissionRequest reads MISSION_REQUEST and MISSION_REQUEST_INT, which
// share a layout.
func DecodeMissionRequest(p []byte) (seq uint16, targetSys, targetComp, missionType uint8) {
	return u16(p), p[2], p[3], p[4]
}

func DecodeMissionAck(p []byte) (targetSys, targetComp, result, missionType uint8) {
	return p[0], p[1], p[2], p[3]
}

func cString(b []byte) string {
	if i := bytes.IndexByte(b, 0); i >= 0 {
		b = b[:i]
	}
	return string(b)
}

// PackGCSHeartbeat returns the payload of a ground station's HEARTBEAT.
func PackGCSHeartbeat() []byte {
	p := make([]byte, 9)
	p[4] = TypeGCS
	p[5] = AutopilotInvalid
	p[8] = 3 // mavlink_version
	return p
}

func PackRequestDataStream(targetSys, targetComp, streamID uint8, rateHz uint16) []byte {
	p := make([]byte, 6)
	binary.LittleEndian.PutUint16(p, rateHz)
	p[2], p[3], p[4], p[5] = targetSys, targetComp, streamID, 1
	return p
}

func PackMissionCount(targetSys, targetComp uint8, count uint16) []byte {
	p := make([]byte, 9)
	binary.LittleEndian.PutUint16(p, count)
	p[2], p[3], p[4] = targetSys, targetComp, MissionTypeMission
	return p
}

func PackMissionAck(targetSys, targetComp, result uint8) []byte {
	p := make([]byte, 8)
	p[0], p[1], p[2], p[3] = targetSys, targetComp, result, MissionTypeMission
	return p
}

// MissionItem is one step of a mission to upload.
type MissionItem struct {
	Command  uint16
	Frame    uint8
	Lat, Lon float64 // degrees
	AltM     float64
}

func PackMissionItemInt(targetSys, targetComp uint8, seq uint16, item MissionItem) []byte {
	p := make([]byte, 38)
	binary.LittleEndian.PutUint32(p[16:], uint32(int32(math.Round(item.Lat*degE7))))
	binary.LittleEndian.PutUint32(p[20:], uint32(int32(math.Round(item.Lon*degE7))))
	binary.LittleEndian.PutUint32(p[24:], math.Float32bits(float32(item.AltM)))
	binary.LittleEndian.PutUint16(p[28:], seq)
	binary.LittleEndian.PutUint16(p[30:], item.Command)
	p[32], p[33] = targetSys, targetComp
	p[34] = item.Frame
	p[36] = 1 // autocontinue
	p[37] = MissionTypeMission
	return p
}
