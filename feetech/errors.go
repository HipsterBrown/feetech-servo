package feetech

import (
	"errors"
	"fmt"
	"sort"
	"strings"
)

// Sentinel errors for common failure modes.
var (
	ErrTimeout       = errors.New("communication timeout")
	ErrNoResponse    = errors.New("no response from servo")
	ErrInvalidPacket = errors.New("invalid packet format")
	ErrBusClosed     = errors.New("bus is closed")
	ErrInvalidID     = errors.New("invalid servo ID")
)

// CommError represents a communication-level error.
type CommError struct {
	Op  string // Operation that failed (e.g., "read", "write", "ping")
	Err error  // Underlying error
}

func (e *CommError) Error() string {
	return fmt.Sprintf("communication error during %s: %v", e.Op, e.Err)
}

func (e *CommError) Unwrap() error {
	return e.Err
}

// ServoError represents an error from a specific servo.
type ServoError struct {
	ID     int         // Servo ID
	Op     string      // Operation that failed
	Status StatusError // Status flags from servo (if applicable)
	Err    error       // Underlying error (if applicable)
}

func (e *ServoError) Error() string {
	if e.Status != 0 {
		return fmt.Sprintf("servo %d %s failed: %s", e.ID, e.Op, e.Status.Error())
	}
	if e.Err != nil {
		return fmt.Sprintf("servo %d %s failed: %v", e.ID, e.Op, e.Err)
	}
	return fmt.Sprintf("servo %d %s failed", e.ID, e.Op)
}

func (e *ServoError) Unwrap() error {
	return e.Err
}

// SyncReadError reports condition flags observed from a SyncRead across a
// group of servos, keyed by servo ID. Unlike ServoError.ID — which names a
// single servo — a sync read can have several servos answer with a flag at
// once, and this preserves which one said what instead of collapsing them
// into one anonymous report.
type SyncReadError struct {
	Op     string
	Status map[int]StatusError // servo ID -> condition flags reported by that servo
}

func (e *SyncReadError) Error() string {
	if len(e.Status) == 0 {
		return fmt.Sprintf("%s: no servo flags recorded", e.Op)
	}

	ids := make([]int, 0, len(e.Status))
	for id := range e.Status {
		ids = append(ids, id)
	}
	sort.Ints(ids) // map order is random; the message must not be

	parts := make([]string, len(ids))
	for i, id := range ids {
		parts[i] = fmt.Sprintf("servo %d %v", id, e.Status[id].flagNames())
	}
	return fmt.Sprintf("%s: %s", e.Op, strings.Join(parts, ", "))
}

// Pins the one property of the As hook below that can break invisibly: a
// mistyped signature (As(error) bool, As(interface{}) error) still compiles and
// is simply never called by errors.As, silently disabling ConditionStatus for
// this type.
var _ interface{ As(any) bool } = (*SyncReadError)(nil)

// As implements the errors.As matching hook (see the errors.As docs) so that
// ConditionStatus(err) keeps working unchanged: errors.As(err, &someStatusError)
// resolves to the OR of every per-servo flag, exactly as if a single servo
// had reported the combination.
func (e *SyncReadError) As(target any) bool {
	status, ok := target.(*StatusError)
	if !ok {
		return false
	}
	var combined StatusError
	for _, s := range e.Status {
		combined |= s
	}
	if combined == 0 {
		// Don't claim to be a StatusError with nothing set — a caller using
		// errors.As directly, without going through ConditionStatus, would
		// read that as a successful match reporting "no flags".
		return false
	}
	*status = combined
	return true
}

// IsTimeout returns true if the error is a timeout error.
func IsTimeout(err error) bool {
	return errors.Is(err, ErrTimeout)
}

// IsNoResponse returns true if the error indicates no response was received.
func IsNoResponse(err error) bool {
	return errors.Is(err, ErrNoResponse)
}

// GetServoError extracts a ServoError from an error chain, if present.
func GetServoError(err error) (*ServoError, bool) {
	var servoErr *ServoError
	if errors.As(err, &servoErr) {
		return servoErr, true
	}
	return nil, false
}

// ConditionStatus reports the servo condition flags carried by err, if err is
// purely a condition report (overload, overheat, voltage, angle limit).
//
// ok is true only when the data returned alongside err is safe to use. It is
// false for transport failures, and false for request-rejection flags
// (checksum, instruction, range) where the servo never answered the question.
//
//	pos, err := servo.Position(ctx)
//	if flags, ok := ConditionStatus(err); ok {
//	    // pos is valid; flags says why the servo is unhappy
//	} else if err != nil {
//	    return err
//	}
func ConditionStatus(err error) (StatusError, bool) {
	if err == nil {
		return 0, false
	}

	var status StatusError
	if !errors.As(err, &status) {
		var servoErr *ServoError
		if errors.As(err, &servoErr) && servoErr.Status != 0 {
			status = servoErr.Status
		} else {
			return 0, false
		}
	}

	if !isConditionOnly(status) {
		return 0, false
	}
	return status, true
}
