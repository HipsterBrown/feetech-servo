package feetech

import (
	"bytes"
	"testing"
)

func TestProtocol_Checksum(t *testing.T) {
	p := NewProtocol(ProtocolSTS)

	// Test case from Feetech protocol spec:
	// Ping servo ID 1: FF FF 01 02 01 FB
	// Checksum = ~(01 + 02 + 01) = ~04 = FB
	packet := p.PingPacket(0x01)
	expected := []byte{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}

	if !bytes.Equal(packet, expected) {
		t.Errorf("PingPacket: got %X, want %X", packet, expected)
	}
}

func TestProtocol_ReadPacket(t *testing.T) {
	p := NewProtocol(ProtocolSTS)

	// Test case from spec:
	// Read 2 bytes from address 0x38 on servo ID 1
	// FF FF 01 04 02 38 02 BE
	packet := p.ReadPacket(0x01, 0x38, 0x02)
	expected := []byte{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE}

	if !bytes.Equal(packet, expected) {
		t.Errorf("ReadPacket: got %X, want %X", packet, expected)
	}
}

func TestProtocol_WritePacket(t *testing.T) {
	p := NewProtocol(ProtocolSTS)

	// Test case: Write ID value 1 to address 5 using broadcast
	// FF FF FE 04 03 05 01 F4
	packet := p.WritePacket(BroadcastID, 0x05, []byte{0x01})
	expected := []byte{0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x05, 0x01, 0xF4}

	if !bytes.Equal(packet, expected) {
		t.Errorf("WritePacket: got %X, want %X", packet, expected)
	}
}

func TestProtocol_DecodeResponse(t *testing.T) {
	p := NewProtocol(ProtocolSTS)

	// Response to ping: FF FF 01 02 00 FC
	data := []byte{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC}

	pkt, consumed, err := p.Decode(data)
	if err != nil {
		t.Fatalf("Decode failed: %v", err)
	}

	if consumed != 6 {
		t.Errorf("consumed: got %d, want 6", consumed)
	}
	if pkt.ID != 0x01 {
		t.Errorf("ID: got %d, want 1", pkt.ID)
	}
	if pkt.Error != 0 {
		t.Errorf("Error: got %d, want 0", pkt.Error)
	}
}

func TestProtocol_DecodeWithData(t *testing.T) {
	p := NewProtocol(ProtocolSTS)

	// Response to read position: FF FF 01 04 00 18 05 DD
	// Position value: 0x0518 = 1304 (little-endian)
	data := []byte{0xFF, 0xFF, 0x01, 0x04, 0x00, 0x18, 0x05, 0xDD}

	pkt, _, err := p.Decode(data)
	if err != nil {
		t.Fatalf("Decode failed: %v", err)
	}

	if pkt.ID != 0x01 {
		t.Errorf("ID: got %d, want 1", pkt.ID)
	}
	if len(pkt.Parameters) != 2 {
		t.Fatalf("Parameters length: got %d, want 2", len(pkt.Parameters))
	}

	position := p.DecodeWord(pkt.Parameters)
	if position != 0x0518 {
		t.Errorf("Position: got %d, want %d", position, 0x0518)
	}
}

func TestProtocol_DecodeWithGarbage(t *testing.T) {
	p := NewProtocol(ProtocolSTS)

	// Data with garbage before valid packet
	data := []byte{0x00, 0x12, 0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC}

	pkt, consumed, err := p.Decode(data)
	if err != nil {
		t.Fatalf("Decode failed: %v", err)
	}

	// Should skip garbage and find packet at offset 2
	if consumed != 8 { // 2 garbage + 6 packet
		t.Errorf("consumed: got %d, want 8", consumed)
	}
	if pkt.ID != 0x01 {
		t.Errorf("ID: got %d, want 1", pkt.ID)
	}
}

func TestProtocol_DecodeMultiple(t *testing.T) {
	p := NewProtocol(ProtocolSTS)

	// Two responses concatenated
	data := []byte{
		0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2, // ID 1, position 2048
		0xFF, 0xFF, 0x02, 0x04, 0x00, 0x00, 0x10, 0xE9, // ID 2, position 4096
	}

	packets, err := p.DecodeMultiple(data, 2)
	if err != nil {
		t.Fatalf("DecodeMultiple failed: %v", err)
	}

	if len(packets) != 2 {
		t.Fatalf("got %d packets, want 2", len(packets))
	}

	if packets[0].ID != 1 {
		t.Errorf("packet 0 ID: got %d, want 1", packets[0].ID)
	}
	if packets[1].ID != 2 {
		t.Errorf("packet 1 ID: got %d, want 2", packets[1].ID)
	}
}

func TestProtocol_ByteOrderSTS(t *testing.T) {
	p := NewProtocol(ProtocolSTS)

	// STS is little-endian
	data := p.EncodeWord(0x1234)
	if data[0] != 0x34 || data[1] != 0x12 {
		t.Errorf("EncodeWord: got %X, want [34 12]", data)
	}

	decoded := p.DecodeWord([]byte{0x34, 0x12})
	if decoded != 0x1234 {
		t.Errorf("DecodeWord: got %X, want 1234", decoded)
	}
}

func TestProtocol_ByteOrderSCS(t *testing.T) {
	p := NewProtocol(ProtocolSCS)

	// SCS is big-endian
	data := p.EncodeWord(0x1234)
	if data[0] != 0x12 || data[1] != 0x34 {
		t.Errorf("EncodeWord: got %X, want [12 34]", data)
	}

	decoded := p.DecodeWord([]byte{0x12, 0x34})
	if decoded != 0x1234 {
		t.Errorf("DecodeWord: got %X, want 1234", decoded)
	}
}

func TestProtocol_SyncWritePacket(t *testing.T) {
	p := NewProtocol(ProtocolSTS)

	// From spec: sync write position 0x0800 to servos 1-4
	servoData := map[byte][]byte{
		1: {0x00, 0x08, 0x00, 0x00, 0xE8, 0x03},
		2: {0x00, 0x08, 0x00, 0x00, 0xE8, 0x03},
		3: {0x00, 0x08, 0x00, 0x00, 0xE8, 0x03},
		4: {0x00, 0x08, 0x00, 0x00, 0xE8, 0x03},
	}

	packet := p.SyncWritePacket(0x2A, 6, servoData)

	// Verify structure
	if packet[0] != 0xFF || packet[1] != 0xFF {
		t.Error("missing header")
	}
	if packet[2] != BroadcastID {
		t.Error("not broadcast ID")
	}
	if packet[4] != InstSyncWrite {
		t.Error("wrong instruction")
	}
	if packet[5] != 0x2A {
		t.Error("wrong address")
	}
	if packet[6] != 6 {
		t.Error("wrong data length")
	}
}

func TestStatusError(t *testing.T) {
	tests := []struct {
		status   StatusError
		hasError bool
	}{
		{0, false},
		{ErrVoltage, true},
		{ErrOverheat, true},
		{ErrOverload | ErrOverheat, true},
	}

	for _, tt := range tests {
		if tt.status.HasError() != tt.hasError {
			t.Errorf("StatusError(%X).HasError(): got %v, want %v",
				tt.status, tt.status.HasError(), tt.hasError)
		}
	}
}

func TestStatusError_String(t *testing.T) {
	err := ErrOverheat | ErrOverload
	s := err.Error()

	if s == "" {
		t.Error("expected non-empty error string")
	}
}

func TestProtocol_DecodeChecksumError(t *testing.T) {
	p := NewProtocol(ProtocolSTS)

	// Valid packet with corrupted checksum
	data := []byte{0xFF, 0xFF, 0x01, 0x02, 0x00, 0x00} // Checksum should be 0xFC

	_, _, err := p.Decode(data)
	if err == nil {
		t.Error("expected checksum error")
	}
}
