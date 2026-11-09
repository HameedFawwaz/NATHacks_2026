package mavlink

import (
	"crypto/sha256"
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// These frames were signed by the C MAVLink library that core and ArduPilot use,
// with key[i] = 7*i+1, link ID 3, and timestamps 0xA1B2C3D4E5, +1 and +2. They
// come from a small program built against core/third_party/mavlink.
const (
	cHeartbeat  = "fd090100000101000000030000000203d1040307bf03e5d4c3b2a10071385cbc50a5"
	cNamedValue = "fd1101000101bffb0000e80300000000803e54525553545f4e4156c58203e6d4c3b2a100886f07643d30"
	cStatusText = "fd270100020101fd0000044155544f4e4f4d5920534146455f4c414e443a20686172647761726520756e747275737465644f5d03e7d4c3b2a1007965df026013"

	cBaseTimestamp = 0xA1B2C3D4E5
	cLinkID        = 3
)

func cKey() (key [KeyLen]byte) {
	for i := range key {
		key[i] = byte(7*i + 1)
	}
	return key
}

// signerAt returns a signer whose clock starts at the golden frames' time.
func signerAt(key [KeyLen]byte) *Signer {
	s := NewSigner(key, cLinkID)
	s.timestamp = cBaseTimestamp
	return s
}

func TestAcceptsFramesSignedByC(t *testing.T) {
	s := signerAt(cKey())
	var data []byte
	data = append(data, mustHex(t, cHeartbeat)...)
	data = append(data, mustHex(t, cNamedValue)...)
	data = append(data, mustHex(t, cStatusText)...)

	frames := s.Parse(data)
	if len(frames) != 3 {
		t.Fatalf("got %d frames, want 3; stats %+v", len(frames), s.Stats())
	}
	if frames[0].MsgID != MsgHeartbeat || frames[1].MsgID != MsgNamedValueFloat || frames[2].MsgID != MsgStatusText {
		t.Fatalf("message IDs: %d %d %d", frames[0].MsgID, frames[1].MsgID, frames[2].MsgID)
	}
	if name, v := DecodeNamedValueFloat(frames[1].Payload); name != "TRUST_NAV" || v != 0.25 || frames[1].CompID != 191 {
		t.Errorf("named value: %q %v comp %d", name, v, frames[1].CompID)
	}
	if _, text := DecodeStatusText(frames[2].Payload); text != "AUTONOMY SAFE_LAND: hardware untrusted" {
		t.Errorf("status text: %q", text)
	}
	if st := s.Stats(); st.SignedAccepted != 3 || st.RejectedBadSig+st.RejectedReplay+st.RejectedUnsigned != 0 {
		t.Errorf("stats %+v", st)
	}
}

func TestEncodeMatchesC(t *testing.T) {
	s := signerAt(cKey())
	heartbeat := []byte{3, 0, 0, 0, 2, 3, 209, 4, 3}
	got := s.Encode(0, 1, 1, MsgHeartbeat, heartbeat)
	if want := mustHex(t, cHeartbeat); string(got) != string(want) {
		t.Errorf("heartbeat\n got %x\nwant %x", got, want)
	}

	named := namedValuePayload("TRUST_NAV", 0.25, 1000)
	got = s.Encode(1, 1, 191, MsgNamedValueFloat, named)
	if want := mustHex(t, cNamedValue); string(got) != string(want) {
		t.Errorf("named value\n got %x\nwant %x", got, want)
	}

	text := make([]byte, 54)
	text[0] = 4
	copy(text[1:], "AUTONOMY SAFE_LAND: hardware untrusted")
	got = s.Encode(2, 1, 1, MsgStatusText, text)
	if want := mustHex(t, cStatusText); string(got) != string(want) {
		t.Errorf("status text\n got %x\nwant %x", got, want)
	}
}

func namedValuePayload(name string, value float32, timeMs uint32) []byte {
	p := make([]byte, 18)
	p[0], p[1], p[2], p[3] = byte(timeMs), byte(timeMs>>8), byte(timeMs>>16), byte(timeMs>>24)
	bits := math.Float32bits(value)
	p[4], p[5], p[6], p[7] = byte(bits), byte(bits>>8), byte(bits>>16), byte(bits>>24)
	copy(p[8:], name)
	return p
}

func TestRoundTrip(t *testing.T) {
	sender, receiver := signerAt(cKey()), signerAt(cKey())
	for i := range 5 {
		frames := receiver.Parse(sender.Encode(uint8(i), 254, 190, MsgHeartbeat, PackGCSHeartbeat()))
		if len(frames) != 1 || frames[0].SysID != 254 {
			t.Fatalf("frame %d dropped; stats %+v", i, receiver.Stats())
		}
	}
}

func TestRejectsReplay(t *testing.T) {
	s := signerAt(cKey())
	hb := mustHex(t, cHeartbeat)
	if len(s.Parse(hb)) != 1 {
		t.Fatal("first copy dropped")
	}
	if len(s.Parse(hb)) != 0 {
		t.Fatal("replayed frame accepted")
	}
	// An older frame from the same sender is a replay too, not just an exact repeat.
	s = signerAt(cKey())
	if len(s.Parse(mustHex(t, cNamedValue))) != 1 { // timestamp +1, but a different sender
		t.Fatal("named value dropped")
	}
	if len(s.Parse(mustHex(t, cNamedValue))) != 0 {
		t.Fatal("replayed named value accepted")
	}
	if st := s.Stats(); st.RejectedReplay != 1 || st.RejectedBadSig != 0 {
		t.Errorf("stats %+v", st)
	}
}

func TestRejectsTamperedFrames(t *testing.T) {
	good := mustHex(t, cNamedValue)
	for _, i := range []int{
		1,              // length
		5,              // system ID
		10,             // payload
		len(good) - 1,  // last signature byte
		len(good) - 8,  // timestamp
		len(good) - 14, // last CRC byte, so a bad CRC rather than a bad signature
		len(good) - 13, // link ID
	} {
		s := signerAt(cKey())
		bad := append([]byte(nil), good...)
		bad[i] ^= 0x01
		if got := s.Parse(bad); len(got) != 0 {
			t.Errorf("byte %d flipped: frame accepted", i)
		}
	}

	// A valid CRC with a changed payload still fails: only the key makes signatures.
	s := signerAt(cKey())
	forged := mustHex(t, cNamedValue)
	forged[10] ^= 0x01
	crcAt := len(forged) - sigLen - crcLen
	c := crc(forged[1:crcAt], messageInfo[MsgNamedValueFloat].crcExtra)
	forged[crcAt], forged[crcAt+1] = byte(c), byte(c>>8)
	if len(s.Parse(forged)) != 0 {
		t.Fatal("forged frame with a fixed-up CRC accepted")
	}
	if st := s.Stats(); st.RejectedBadSig != 1 {
		t.Errorf("stats %+v", st)
	}
}

func TestRejectsWrongKey(t *testing.T) {
	other := cKey()
	other[0] ^= 0xff
	s := signerAt(other)
	if got := s.Parse(mustHex(t, cHeartbeat)); len(got) != 0 {
		t.Fatal("accepted a frame signed with another key")
	}
	if st := s.Stats(); st.RejectedBadSig != 1 || st.LastRejectMs == 0 {
		t.Errorf("stats %+v", st)
	}
}

func TestRejectsUnsignedWhenSigning(t *testing.T) {
	unsigned := Encode(0, 1, 1, MsgHeartbeat, PackGCSHeartbeat())
	if len(Parse(unsigned)) != 1 {
		t.Fatal("unsigned parse changed")
	}
	s := signerAt(cKey())
	if len(s.Parse(unsigned)) != 0 {
		t.Fatal("signer accepted an unsigned frame")
	}
	// MAVLink 1 can't carry a signature.
	v1 := []byte{magicV1, 9, 0, 1, 1, MsgHeartbeat, 0, 0, 0, 0, 2, 3, 0, 0, 3}
	c := crc(v1[1:], messageInfo[MsgHeartbeat].crcExtra)
	v1 = append(v1, byte(c), byte(c>>8))
	if len(Parse(v1)) != 1 {
		t.Fatal("test frame isn't a valid MAVLink 1 heartbeat")
	}
	if len(s.Parse(v1)) != 0 {
		t.Fatal("signer accepted a MAVLink 1 frame")
	}
	if st := s.Stats(); st.RejectedUnsigned != 2 {
		t.Errorf("stats %+v", st)
	}
}

func TestPlainParseStillSkipsSignatures(t *testing.T) {
	// Without a key there is nothing to check against, so signed frames stay readable.
	if len(Parse(mustHex(t, cHeartbeat))) != 1 {
		t.Fatal("signed frame not readable without a key")
	}
}

func TestRejectsStaleNewSender(t *testing.T) {
	sender := signerAt(cKey())
	frame := sender.Encode(0, 1, 1, MsgHeartbeat, PackGCSHeartbeat())

	receiver := signerAt(cKey())
	receiver.timestamp = cBaseTimestamp + 61*100_000 // a minute after the frame was signed
	if len(receiver.Parse(frame)) != 0 {
		t.Fatal("accepted a new sender's frame signed over a minute ago")
	}
	if st := receiver.Stats(); st.RejectedReplay != 1 {
		t.Errorf("stats %+v", st)
	}

	receiver = signerAt(cKey())
	receiver.timestamp = cBaseTimestamp + 30*100_000
	if len(receiver.Parse(frame)) != 1 {
		t.Fatal("rejected a recent frame from a new sender")
	}
}

func TestSendersAreTrackedSeparately(t *testing.T) {
	a, b := signerAt(cKey()), signerAt(cKey())
	b.timestamp += 100 // b's timestamps run ahead
	receiver := signerAt(cKey())
	for range 3 {
		if len(receiver.Parse(b.Encode(0, 2, 1, MsgHeartbeat, PackGCSHeartbeat()))) != 1 {
			t.Fatal("sender b dropped")
		}
	}
	// a's timestamps are older than b's, but they're a different sender.
	if len(receiver.Parse(a.Encode(0, 1, 1, MsgHeartbeat, PackGCSHeartbeat()))) != 1 {
		t.Fatal("sender a dropped because of sender b's timestamps")
	}
}

func TestSenderLimit(t *testing.T) {
	sender, receiver := signerAt(cKey()), signerAt(cKey())
	for id := 1; id <= maxSigningStreams+1; id++ {
		ok := len(receiver.Parse(sender.Encode(0, uint8(id), 1, MsgHeartbeat, PackGCSHeartbeat()))) == 1
		if want := id <= maxSigningStreams; ok != want {
			t.Fatalf("sender %d accepted = %v, want %v", id, ok, want)
		}
	}
}

func TestParseKey(t *testing.T) {
	hex := strings.Repeat("0a", KeyLen)
	for _, text := range []string{hex, hex + "\n", " " + strings.ToUpper(hex) + "\r\n"} {
		if key, err := ParseKey(text); err != nil || key[0] != 0x0a || key[KeyLen-1] != 0x0a {
			t.Errorf("ParseKey(%q) = %x, %v", text, key, err)
		}
	}
	for _, text := range []string{"", hex[:62], hex + "00", strings.Repeat("zz", KeyLen)} {
		if _, err := ParseKey(text); err == nil {
			t.Errorf("ParseKey(%q) succeeded", text)
		}
	}
}

func TestLoadKeyFile(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "signing.key")
	// signing_keygen writes 64 hex characters and a newline.
	if err := os.WriteFile(path, []byte(strings.Repeat("ab", KeyLen)+"\n"), 0o600); err != nil {
		t.Fatal(err)
	}
	if key, err := LoadKeyFile(path); err != nil || key[0] != 0xab {
		t.Fatalf("LoadKeyFile = %x, %v", key, err)
	}
	if _, err := LoadKeyFile(filepath.Join(dir, "missing.key")); err == nil {
		t.Error("loaded a file that doesn't exist")
	}
	bad := filepath.Join(dir, "bad.key")
	os.WriteFile(bad, []byte("not a key\n"), 0o600)
	if _, err := LoadKeyFile(bad); err == nil {
		t.Error("loaded a malformed key")
	}
	// A file written by core's signing_keygen --from-passphrase, for the passphrase
	// "correct-horse-battery", which is the SHA-256 of the passphrase.
	fromC := filepath.Join(dir, "from-c.key")
	os.WriteFile(fromC, []byte("62249369389075490555a758353aec61500c6218fa597252d52dc4bd0148f12d\n"), 0o600)
	if key, err := LoadKeyFile(fromC); err != nil || key != sha256.Sum256([]byte("correct-horse-battery")) {
		t.Errorf("key from signing_keygen: %x, %v", key, err)
	}
	empty := filepath.Join(dir, "empty.key")
	os.WriteFile(empty, nil, 0o600)
	if _, err := LoadKeyFile(empty); err == nil {
		t.Error("loaded an empty key file")
	}
}
