package feetech

import "time"

// serialPort interface allows using either OS serial port or UART on microcontroller.
type serialPort interface {
	Read(p []byte) (n int, err error)
	Write(p []byte) (n int, err error)
	SetReadTimeout(t time.Duration) error
	Close() error
}
