package feetech

import (
	"context"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/transports"
)

// ackPacket returns a 6-byte status packet acknowledging an instruction from servo `id`
// with no error. Format: FF FF id 02 00 chk where chk = ^(id + 02 + 00).
func ackPacket(id byte) []byte {
	chk := ^(id + 0x02 + 0x00)
	return []byte{0xFF, 0xFF, id, 0x02, 0x00, chk}
}

// TestServo_WriteRegister_AutoUnlocksEEPROM verifies that a write to a register
// marked EEPROM=true triggers the unlock-write-relock packet sequence.
func TestServo_WriteRegister_AutoUnlocksEEPROM(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			// Step 1: unlock (write 0 to addr 55 on STS3215).
			{Send: nil, Reply: ackPacket(1)},
			// Step 2: target write (write 5 to addr 5 = ID register).
			{Send: nil, Reply: ackPacket(1)},
			// Step 3: re-lock (write 1 to addr 55).
			{Send: nil, Reply: ackPacket(1)},
		},
	}

	bus, err := NewBus(BusConfig{
		Transport: mock,
		Timeout:   100 * time.Millisecond,
	})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	servo := NewServo(bus, 1, nil) // default STS3215, LockAddress=55

	// Note: SetID currently calls SetTorqueEnabled first, which would consume an
	// extra dance step. To keep this initial test focused, use the public
	// WriteRegister(name) entry point with the "id" register name.
	if err := servo.WriteRegister(context.Background(), "id", []byte{5}); err != nil {
		t.Fatalf("WriteRegister: %v", err)
	}

	// Mock should have recorded three write packets — verify the last byte
	// of each is the expected lock/data payload by parsing the writes.
	// Each write packet is 8 bytes: FF FF id 04 03 addr value chk.
	if len(mock.WriteData) != 24 {
		t.Fatalf("expected 24 bytes (3 writes × 8 bytes), got %d: %X", len(mock.WriteData), mock.WriteData)
	}
	// Packet 1: unlock at addr 55 with value 0
	if got := mock.WriteData[5]; got != 55 {
		t.Errorf("packet 1 addr: got %d want 55 (lock register)", got)
	}
	if got := mock.WriteData[6]; got != 0 {
		t.Errorf("packet 1 value: got %d want 0 (unlock)", got)
	}
	// Packet 2: ID write at addr 5 with value 5
	if got := mock.WriteData[13]; got != 5 {
		t.Errorf("packet 2 addr: got %d want 5 (id register)", got)
	}
	if got := mock.WriteData[14]; got != 5 {
		t.Errorf("packet 2 value: got %d want 5", got)
	}
	// Packet 3: re-lock at addr 55 with value 1
	if got := mock.WriteData[21]; got != 55 {
		t.Errorf("packet 3 addr: got %d want 55 (lock register)", got)
	}
	if got := mock.WriteData[22]; got != 1 {
		t.Errorf("packet 3 value: got %d want 1 (re-lock)", got)
	}
}
