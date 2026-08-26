package feetech

import (
	"errors"
	"fmt"
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
