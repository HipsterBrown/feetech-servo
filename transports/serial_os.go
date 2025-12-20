//go:build !baremetal

package transports

import (
	"errors"
	"fmt"
	"time"

	"go.bug.st/serial"
)

// SerialTransport implements Transport using a hardware serial port.
type SerialTransport struct {
	port     serial.Port
	portName string
	timeout  time.Duration
}

// SerialConfig holds configuration for opening a serial port.
type SerialConfig struct {
	Port     string
	BaudRate int
	Timeout  time.Duration
}

// OpenSerial opens a serial port with the given configuration.
func OpenSerial(cfg SerialConfig) (*SerialTransport, error) {
	if cfg.Port == "" {
		return nil, errors.New("serial port path is required")
	}

	if cfg.BaudRate == 0 {
		cfg.BaudRate = 1000000
	}

	if cfg.Timeout == 0 {
		cfg.Timeout = time.Second
	}

	mode := &serial.Mode{
		BaudRate: cfg.BaudRate,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	}

	port, err := serial.Open(cfg.Port, mode)
	if err != nil {
		return nil, fmt.Errorf("failed to open serial port: %w", err)
	}

	if err := port.SetReadTimeout(cfg.Timeout); err != nil {
		port.Close()
		return nil, fmt.Errorf("failed to set read timeout: %w", err)
	}

	return &SerialTransport{
		port:     port,
		portName: cfg.Port,
		timeout:  cfg.Timeout,
	}, nil
}

func (t *SerialTransport) Read(p []byte) (int, error) {
	return t.port.Read(p)
}

func (t *SerialTransport) Write(p []byte) (int, error) {
	return t.port.Write(p)
}

func (t *SerialTransport) Close() error {
	return t.port.Close()
}

func (t *SerialTransport) SetReadTimeout(timeout time.Duration) error {
	t.timeout = timeout
	return t.port.SetReadTimeout(timeout)
}

func (t *SerialTransport) Flush() error {
	// Read and discard any buffered data
	buf := make([]byte, 4096)
	t.port.SetReadTimeout(10 * time.Millisecond)
	for {
		n, err := t.port.Read(buf)
		if n == 0 || err != nil {
			break
		}
	}
	// Restore original timeout
	t.port.SetReadTimeout(t.timeout)
	return nil
}

// PortName returns the serial port name.
func (t *SerialTransport) PortName() string {
	return t.portName
}
