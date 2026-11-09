package mavlink

import (
	"encoding/binary"
	"math"
	"testing"
)

func TestSimStateWireAndLegacyCoordinates(t *testing.T) {
	p := make([]byte, 84) // legacy SIM_STATE, no integer extensions
	binary.LittleEndian.PutUint32(p[52:], math.Float32bits(-35.36))
	binary.LittleEndian.PutUint32(p[56:], math.Float32bits(149.16))
	frames := Parse(Encode(0, 1, 1, MsgSimState, p))
	if len(frames) != 1 {
		t.Fatal("SIM_STATE CRC/length not supported")
	}
	lat, lon := DecodeSimState(frames[0].Payload)
	if math.Abs(lat+35.36) > .00001 || math.Abs(lon-149.16) > .00001 {
		t.Fatal(lat, lon)
	}
	p = frames[0].Payload
	x, y := int32(-353632620), int32(1491652370)
	binary.LittleEndian.PutUint32(p[84:], uint32(x))
	binary.LittleEndian.PutUint32(p[88:], uint32(y))
	lat, lon = DecodeSimState(Parse(Encode(1, 1, 1, MsgSimState, p))[0].Payload)
	if lat != float64(x)/1e7 || lon != float64(y)/1e7 {
		t.Fatal("integer precision lost", lat, lon)
	}
}
