package feetech

import (
	"io"
	"time"
)

// Transport is the interface for low-level communication with the servo bus.
// This abstraction allows for testing with mock implementations.
type Transport interface {
	io.ReadWriteCloser

	// SetReadTimeout sets the read timeout duration.
	SetReadTimeout(timeout time.Duration) error

	// Flush discards any buffered input data.
	Flush() error
}
