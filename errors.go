// errors.go - Custom error types for the feetech package
package feetech

import "fmt"

// ServoError represents a servo-specific error
type ServoError struct {
	ServoID int
	Op      string
	Err     error
}

func (e *ServoError) Error() string {
	return fmt.Sprintf("servo %d %s: %v", e.ServoID, e.Op, e.Err)
}

func (e *ServoError) Unwrap() error {
	return e.Err
}

// CommunicationError represents a communication failure
type CommunicationError struct {
	Code int
	Msg  string
}

func (e *CommunicationError) Error() string {
	return fmt.Sprintf("communication error (code %d): %s", e.Code, e.Msg)
}

// NewCommunicationError creates a new communication error
func NewCommunicationError(code int) error {
	var msg string
	switch code {
	case CommPortBusy:
		msg = "port is busy"
	case CommTxFail:
		msg = "failed to transmit packet"
	case CommRxFail:
		msg = "failed to receive packet"
	case CommTxError:
		msg = "incorrect instruction packet"
	case CommRxWaiting:
		msg = "waiting for status packet"
	case CommRxTimeout:
		msg = "no status packet received"
	case CommRxCorrupt:
		msg = "incorrect status packet"
	case CommNotAvail:
		msg = "protocol does not support this function"
	default:
		msg = "unknown error"
	}
	return &CommunicationError{Code: code, Msg: msg}
}
