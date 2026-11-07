package bridge

import (
	"encoding/hex"
	"math"
	"testing"
)

func float32bits(f float32) uint32 { return math.Float32bits(f) }

func mustGolden(t *testing.T, s string) []byte {
	t.Helper()
	b, err := hex.DecodeString(s)
	if err != nil {
		t.Fatal(err)
	}
	return b
}
