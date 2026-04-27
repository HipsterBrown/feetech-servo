package transports

import (
	"bytes"
	"errors"
	"io"
	"testing"
	"time"
)

func TestMockTransport_Write_AppendsAndReturnsLen(t *testing.T) {
	m := &MockTransport{}
	n, err := m.Write([]byte{0x01, 0x02})
	if err != nil || n != 2 {
		t.Fatalf("unexpected write: n=%d err=%v", n, err)
	}
	if !bytes.Equal(m.WriteData, []byte{0x01, 0x02}) {
		t.Errorf("WriteData = %X", m.WriteData)
	}
	n2, _ := m.Write([]byte{0x03})
	if n2 != 1 || len(m.WriteData) != 3 {
		t.Errorf("appends should accumulate: %X", m.WriteData)
	}
}

func TestMockTransport_Write_ErrorPath(t *testing.T) {
	want := errors.New("boom")
	m := &MockTransport{WriteErr: want}
	_, err := m.Write([]byte{0x01})
	if !errors.Is(err, want) {
		t.Errorf("got %v want %v", err, want)
	}
}

func TestMockTransport_Read_ConsumesReadData(t *testing.T) {
	m := &MockTransport{ReadData: []byte{0xAA, 0xBB}}
	buf := make([]byte, 1)
	n, err := m.Read(buf)
	if err != nil || n != 1 || buf[0] != 0xAA {
		t.Errorf("first read: n=%d buf=%X err=%v", n, buf, err)
	}
	n, err = m.Read(buf)
	if err != nil || n != 1 || buf[0] != 0xBB {
		t.Errorf("second read: n=%d buf=%X err=%v", n, buf, err)
	}
	n, err = m.Read(buf)
	if !errors.Is(err, io.EOF) || n != 0 {
		t.Errorf("third read should EOF: n=%d err=%v", n, err)
	}
}

func TestMockTransport_Read_ReadFuncTakesPriority(t *testing.T) {
	m := &MockTransport{
		ReadData: []byte{0xAA},
		ReadFunc: func(p []byte) (int, error) {
			p[0] = 0x99
			return 1, nil
		},
	}
	buf := make([]byte, 1)
	n, _ := m.Read(buf)
	if n != 1 || buf[0] != 0x99 {
		t.Errorf("ReadFunc should take precedence: %X", buf)
	}
}

func TestMockTransport_Read_ErrorPath(t *testing.T) {
	want := errors.New("rx fail")
	m := &MockTransport{ReadErr: want}
	_, err := m.Read(make([]byte, 1))
	if !errors.Is(err, want) {
		t.Errorf("got %v want %v", err, want)
	}
}

func TestMockTransport_Close(t *testing.T) {
	m := &MockTransport{}
	if err := m.Close(); err != nil || !m.Closed {
		t.Errorf("Close: err=%v closed=%v", err, m.Closed)
	}
}

func TestMockTransport_SetReadTimeout(t *testing.T) {
	m := &MockTransport{}
	if err := m.SetReadTimeout(time.Second); err != nil {
		t.Fatal(err)
	}
	if m.ReadTimeout != time.Second {
		t.Errorf("ReadTimeout = %v", m.ReadTimeout)
	}
}

func TestMockTransport_Flush_DoesNotClearReadData(t *testing.T) {
	// Documented behavior: tests need to preserve mock response data across Flush.
	m := &MockTransport{ReadData: []byte{0xAA}}
	if err := m.Flush(); err != nil {
		t.Fatal(err)
	}
	if !m.Flushed {
		t.Error("Flushed flag not set")
	}
	if len(m.ReadData) != 1 {
		t.Errorf("Flush should not drain ReadData; got %X", m.ReadData)
	}
}
