package feetech

import (
	"errors"
	"fmt"
	"io"
	"time"

	"go.bug.st/serial"
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

// MockTransport implements Transport for testing.
type MockTransport struct {
	ReadData    []byte
	ReadErr     error
	WriteData   []byte
	WriteErr    error
	Closed      bool
	ReadTimeout time.Duration
	Flushed     bool

	// ReadFunc allows custom read behavior for complex tests
	ReadFunc func(p []byte) (int, error)
}

func (m *MockTransport) Read(p []byte) (int, error) {
	if m.ReadFunc != nil {
		return m.ReadFunc(p)
	}
	if m.ReadErr != nil {
		return 0, m.ReadErr
	}
	n := copy(p, m.ReadData)
	m.ReadData = m.ReadData[n:]
	if n == 0 {
		return 0, io.EOF
	}
	return n, nil
}

func (m *MockTransport) Write(p []byte) (int, error) {
	if m.WriteErr != nil {
		return 0, m.WriteErr
	}
	m.WriteData = append(m.WriteData, p...)
	return len(p), nil
}

func (m *MockTransport) Close() error {
	m.Closed = true
	return nil
}

func (m *MockTransport) SetReadTimeout(timeout time.Duration) error {
	m.ReadTimeout = timeout
	return nil
}

func (m *MockTransport) Flush() error {
	m.Flushed = true
	// Don't clear ReadData - tests need to preserve mock response data
	return nil
}
