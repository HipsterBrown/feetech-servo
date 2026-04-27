package transports

import (
	"bytes"
	"fmt"
)

// Step describes one expected send + corresponding reply in a scripted transcript.
// An empty Send matches any write; an empty Reply means the device returns nothing.
type Step struct {
	Send  []byte
	Reply []byte
}

// Script drives a MockTransport through a declarative transcript. It is not safe for
// concurrent use — tests should drive a single goroutine.
type Script struct {
	Steps []Step

	cursor   int
	replyBuf []byte
}

// recordWrite is invoked by MockTransport.Write when Script != nil. It validates the
// caller's bytes against the next step's Send (if non-empty) and queues the Reply.
func (s *Script) recordWrite(p []byte) error {
	if s.cursor >= len(s.Steps) {
		return fmt.Errorf("script exhausted: extra write of %d bytes (%X)", len(p), p)
	}
	step := s.Steps[s.cursor]
	if len(step.Send) > 0 && !bytes.Equal(p, step.Send) {
		return fmt.Errorf("script step %d: send mismatch\n  got:  %X\n  want: %X",
			s.cursor, p, step.Send)
	}
	s.replyBuf = append(s.replyBuf, step.Reply...)
	s.cursor++
	return nil
}

// drainRead is invoked by MockTransport.Read. It copies up to len(p) bytes from the
// pending replyBuf, returning (0, nil) when no more bytes are available.
func (s *Script) drainRead(p []byte) (int, error) {
	if len(s.replyBuf) == 0 {
		return 0, nil
	}
	n := copy(p, s.replyBuf)
	s.replyBuf = s.replyBuf[n:]
	return n, nil
}
