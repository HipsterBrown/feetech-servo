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

func TestScript_WildcardSendAcceptsAnyBytes(t *testing.T) {
	mock := &MockTransport{}
	mock.Script = &Script{
		Steps: []Step{{Send: nil, Reply: []byte{0xAB}}},
	}
	if _, err := mock.Write([]byte{0x01, 0x02, 0x03}); err != nil {
		t.Fatalf("wildcard Send should accept any write, got: %v", err)
	}
	buf := make([]byte, 1)
	n, _ := mock.Read(buf)
	if n != 1 || buf[0] != 0xAB {
		t.Fatalf("reply not queued after wildcard match: n=%d buf=%X", n, buf)
	}
}

func TestScript_ReplyConsumedAcrossMultipleReads(t *testing.T) {
	mock := &MockTransport{}
	mock.Script = &Script{
		Steps: []Step{{Send: nil, Reply: []byte{0x01, 0x02, 0x03, 0x04}}},
	}
	if _, err := mock.Write([]byte{0xFF}); err != nil {
		t.Fatal(err)
	}
	buf := make([]byte, 2)
	if n, _ := mock.Read(buf); n != 2 || buf[0] != 0x01 || buf[1] != 0x02 {
		t.Fatalf("first read: n=%d buf=%X", n, buf)
	}
	if n, _ := mock.Read(buf); n != 2 || buf[0] != 0x03 || buf[1] != 0x04 {
		t.Fatalf("second read: n=%d buf=%X", n, buf)
	}
	if n, _ := mock.Read(buf); n != 0 {
		t.Fatalf("third read should drain to 0, got n=%d", n)
	}
}
