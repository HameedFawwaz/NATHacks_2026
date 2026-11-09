package mavlink

import (
	"crypto/sha256"
	"crypto/subtle"
	"encoding/hex"
	"errors"
	"fmt"
	"io"
	"log"
	"os"
	"strings"
	"sync"
	"time"
)

// MAVLink 2 message signing, matching the reference implementation that core and
// ArduPilot use. A signed frame carries 13 more bytes after its CRC: a link ID, a
// 48-bit timestamp in 10 us units since 2015-01-01, and the first 6 bytes of
// SHA-256(key, header, payload, CRC, link ID, timestamp).
//
// Signing authenticates; it doesn't encrypt.

const (
	// KeyLen is the signing key length in bytes.
	KeyLen = 32

	maxSigningStreams = 16 // the reference implementation's limit on distinct senders
	// A sender we haven't heard from before is only accepted if its timestamp is
	// within this of the newest we've seen: 60 s in 10 us units.
	newStreamWindow = 60 * 100_000

	signedFlag = flagSigned
)

// epoch2015 is where signature timestamps start counting.
var epoch2015 = time.Date(2015, time.January, 1, 0, 0, 0, 0, time.UTC)

// LinkSecurity counts frames a Signer has dropped. A rising count means somebody
// is sending to this link without the key, or replaying what it carried.
type LinkSecurity struct {
	Enabled          bool
	SignedAccepted   uint32
	RejectedUnsigned uint32 // no signature at all
	RejectedBadSig   uint32 // signature present but wrong key, or tampered
	RejectedReplay   uint32 // valid signature but a stale or repeated timestamp
	LastRejectMs     int64  // Unix ms of the most recent rejection; 0 if none
}

type streamKey struct {
	sysID, compID, linkID uint8
}

// Signer signs outgoing frames and verifies incoming ones. It is safe for
// concurrent use.
type Signer struct {
	key    [KeyLen]byte
	linkID uint8
	now    func() time.Time

	mu        sync.Mutex
	timestamp uint64               // next outgoing timestamp, and the newest one seen
	streams   map[streamKey]uint64 // newest timestamp from each sender
	stats     LinkSecurity
}

// NewSigner returns a Signer for key. linkID identifies this link in the
// signatures it makes.
func NewSigner(key [KeyLen]byte, linkID uint8) *Signer {
	s := &Signer{
		key:     key,
		linkID:  linkID,
		now:     time.Now,
		streams: make(map[streamKey]uint64),
		stats:   LinkSecurity{Enabled: true},
	}
	s.timestamp = s.nowUnits()
	return s
}

// Stats returns the counters so far.
func (s *Signer) Stats() LinkSecurity {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.stats
}

// ParseKey reads a key as 64 hex characters, case-insensitive, with optional
// whitespace around it. This is the format core's signing_keygen writes.
func ParseKey(text string) ([KeyLen]byte, error) {
	var key [KeyLen]byte
	text = strings.TrimSpace(text)
	if len(text) != 2*KeyLen {
		return key, fmt.Errorf("a signing key is exactly %d hex characters, not %d", 2*KeyLen, len(text))
	}
	b, err := hex.DecodeString(text)
	if err != nil {
		return key, errors.New("a signing key must be hex characters only")
	}
	copy(key[:], b)
	return key, nil
}

// LoadKeyFile reads a key file. It warns, but still loads, if other users can
// read the file, like core does.
func LoadKeyFile(path string) ([KeyLen]byte, error) {
	var key [KeyLen]byte
	f, err := os.Open(path)
	if err != nil {
		return key, fmt.Errorf("cannot open signing key file: %w", err)
	}
	defer f.Close()
	if st, err := f.Stat(); err == nil && st.Mode().Perm()&0o077 != 0 {
		log.Printf("signing key file %s is accessible by group/others; run chmod 600", path)
	}
	const maxKeyFileBytes = 2*KeyLen + 64 // room for whitespace, not an arbitrary file
	buf, err := io.ReadAll(io.LimitReader(f, maxKeyFileBytes+1))
	defer clear(buf)
	if err != nil {
		return key, fmt.Errorf("cannot read signing key file: %w", err)
	}
	if len(buf) > maxKeyFileBytes {
		return key, fmt.Errorf("signing key file %s is too large (maximum %d bytes)", path, maxKeyFileBytes)
	}
	key, err = ParseKey(string(buf))
	if err != nil {
		return key, fmt.Errorf("signing key file %s: %w", path, err)
	}
	return key, nil
}

// nowUnits is the current time as a signature timestamp.
func (s *Signer) nowUnits() uint64 {
	d := s.now().Sub(epoch2015)
	if d < 0 {
		return 0
	}
	return uint64(d / (10 * time.Microsecond))
}

// signature returns the 6 bytes that authenticate a frame. frame is everything
// before the signature block: header, payload and CRC. stamp is the link ID and
// timestamp, the first 7 bytes of the block.
func (s *Signer) signature(frame, stamp []byte) []byte {
	h := sha256.New()
	h.Write(s.key[:])
	h.Write(frame)
	h.Write(stamp)
	return h.Sum(nil)[:6]
}

// Encode is the signed version of the package's Encode.
func (s *Signer) Encode(seq, sysID, compID uint8, msgID uint32, payload []byte) []byte {
	frame := encode(seq, sysID, compID, msgID, payload, signedFlag)

	// Like the reference implementation, each frame takes the next timestamp, which
	// started at the time the signer was made.
	s.mu.Lock()
	ts := s.timestamp
	s.timestamp++
	s.mu.Unlock()

	stamp := make([]byte, 7)
	stamp[0] = s.linkID
	for i := range 6 {
		stamp[1+i] = byte(ts >> (8 * i))
	}
	frame = append(frame, stamp...)
	return append(frame, s.signature(frame[:len(frame)-7], stamp)...)
}

// Parse is the verifying version of the package's Parse. It drops unsigned
// frames, frames whose signature doesn't match, and replays, and counts them.
func (s *Signer) Parse(data []byte) []Frame { return parse(data, s) }

// check decides whether a frame is allowed through. frame is the whole frame as
// received. It also records the frame's timestamp, so call it once per frame.
func (s *Signer) check(frame []byte, signed bool, sysID, compID uint8) bool {
	s.mu.Lock()
	defer s.mu.Unlock()

	reject := func(counter *uint32) bool {
		*counter++
		s.stats.LastRejectMs = s.now().UnixMilli()
		return false
	}
	if !signed {
		return reject(&s.stats.RejectedUnsigned)
	}

	body, sig := frame[:len(frame)-sigLen], frame[len(frame)-sigLen:]
	if subtle.ConstantTimeCompare(s.signature(body, sig[:7]), sig[7:]) != 1 {
		return reject(&s.stats.RejectedBadSig)
	}

	var ts uint64
	for i := range 6 {
		ts |= uint64(sig[1+i]) << (8 * i)
	}
	key := streamKey{sysID, compID, sig[0]}
	last, known := s.streams[key]
	switch {
	case known && ts <= last:
		return reject(&s.stats.RejectedReplay) // repeating an old timestamp
	case !known && len(s.streams) >= maxSigningStreams:
		return reject(&s.stats.RejectedBadSig)
	case !known && ts+newStreamWindow < s.timestamp:
		return reject(&s.stats.RejectedReplay) // a new sender with a stale timestamp
	}
	s.streams[key] = ts
	s.timestamp = max(s.timestamp, ts)
	s.stats.SignedAccepted++
	return true
}
