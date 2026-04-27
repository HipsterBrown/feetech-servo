package transports

import (
	"bytes"
	"testing"
)

func TestScript_BasicSendReceive(t *testing.T) {
	mock := &MockTransport{}
	mock.Script = &Script{
		Steps: []Step{
			{
				Send:  []byte{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB},
				Reply: []byte{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC},
			},
		},
	}

	if _, err := mock.Write([]byte{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}); err != nil {
		t.Fatal(err)
	}

	buf := make([]byte, 6)
	n, err := mock.Read(buf)
	if err != nil {
		t.Fatal(err)
	}
	want := []byte{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC}
	if !bytes.Equal(buf[:n], want) {
		t.Fatalf("read: got %X want %X", buf[:n], want)
	}
}

func TestScript_DiffsOnSendMismatch(t *testing.T) {
	mock := &MockTransport{}
	mock.Script = &Script{
		Steps: []Step{
			{Send: []byte{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}, Reply: nil},
		},
	}

	_, err := mock.Write([]byte{0xFF, 0xFF, 0x99, 0x02, 0x01, 0xFB})
	if err == nil {
		t.Fatal("expected error on send mismatch")
	}
	if !bytes.Contains([]byte(err.Error()), []byte("step 0")) {
		t.Errorf("error should mention step index, got: %v", err)
	}
}

func TestScript_ExhaustedScript(t *testing.T) {
	mock := &MockTransport{}
	mock.Script = &Script{Steps: []Step{}}

	if _, err := mock.Write([]byte{0xFF}); err == nil {
		t.Error("expected error when script is empty")
	}
}

func TestScript_RemainsCompatibleWithReadData(t *testing.T) {
	// When Script is nil, MockTransport keeps its existing ReadData behavior.
	mock := &MockTransport{ReadData: []byte{0xAA, 0xBB}}
	buf := make([]byte, 2)
	n, err := mock.Read(buf)
	if err != nil || n != 2 || buf[0] != 0xAA || buf[1] != 0xBB {
		t.Fatalf("ReadData path broken: n=%d err=%v buf=%X", n, err, buf)
	}
}
