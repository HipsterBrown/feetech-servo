package transports

import (
	"io"
	"time"
)

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

	// Script, if non-nil, takes precedence over ReadData/ReadFunc for scripted
	// transcripts. See transports.Script.
	Script *Script
}

func (m *MockTransport) Read(p []byte) (int, error) {
	if m.Script != nil {
		return m.Script.drainRead(p)
	}
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
	if m.Script != nil {
		if err := m.Script.recordWrite(p); err != nil {
			return 0, err
		}
	}
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
