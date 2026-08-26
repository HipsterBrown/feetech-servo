package feetech

import (
	"context"
	"errors"
	"strings"
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

// TestServo_WriteRegister_NoUnlockForSRAM verifies that a write to a non-EEPROM
// register skips the lock dance entirely.
func TestServo_WriteRegister_NoUnlockForSRAM(t *testing.T) {
	mock := &transports.MockTransport{
		ReadData: ackPacket(1),
	}
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	// goal_position is at addr 42, EEPROM=false. Two-byte payload.
	if err := servo.WriteRegister(context.Background(), "goal_position", []byte{0x00, 0x08}); err != nil {
		t.Fatalf("WriteRegister: %v", err)
	}

	// Single 9-byte packet: FF FF 01 05 03 2A 00 08 chk.
	if len(mock.WriteData) != 9 {
		t.Fatalf("expected single 9-byte packet, got %d: %X", len(mock.WriteData), mock.WriteData)
	}
	if mock.WriteData[5] != RegGoalPosition.Address {
		t.Errorf("addr: got %02X want %02X", mock.WriteData[5], RegGoalPosition.Address)
	}
}

// TestServo_SetID_AutoUnlocks confirms SetID performs the dance after disabling torque.
// Sequence: torque-disable (addr 40 = 0) → unlock (addr 55 = 0) → id-write (addr 5 = 7) → relock (addr 55 = 1).
func TestServo_SetID_AutoUnlocks(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Reply: ackPacket(1)}, {Reply: ackPacket(1)},
			{Reply: ackPacket(1)}, {Reply: ackPacket(1)},
		},
	}
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	if err := servo.SetID(context.Background(), 7); err != nil {
		t.Fatalf("SetID: %v", err)
	}

	// 4 × 8-byte packets = 32 bytes.
	if len(mock.WriteData) != 32 {
		t.Fatalf("expected 32 bytes (4 packets × 8 bytes), got %d: %X", len(mock.WriteData), mock.WriteData)
	}
	// Packet 1: torque-disable (addr 40, val 0).
	if mock.WriteData[5] != RegTorqueEnable.Address || mock.WriteData[6] != 0 {
		t.Errorf("packet 1: addr=%02X val=%02X want %02X 00", mock.WriteData[5], mock.WriteData[6], RegTorqueEnable.Address)
	}
	// Packet 2: unlock (addr 55, val 0).
	if mock.WriteData[8+5] != 55 || mock.WriteData[8+6] != 0 {
		t.Errorf("packet 2: addr=%02X val=%02X want 37 00", mock.WriteData[8+5], mock.WriteData[8+6])
	}
	// Packet 3: id write (addr 5, val 7).
	if mock.WriteData[16+5] != RegID.Address || mock.WriteData[16+6] != 7 {
		t.Errorf("packet 3: addr=%02X val=%02X want 05 07", mock.WriteData[16+5], mock.WriteData[16+6])
	}
	// Packet 4: re-lock (addr 55, val 1).
	if mock.WriteData[24+5] != 55 || mock.WriteData[24+6] != 1 {
		t.Errorf("packet 4: addr=%02X val=%02X want 37 01", mock.WriteData[24+5], mock.WriteData[24+6])
	}
}

// TestServo_SetBaudRate_AutoUnlocks confirms SetBaudRate also performs the dance
// after disabling torque.
func TestServo_SetBaudRate_AutoUnlocks(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Reply: ackPacket(1)}, {Reply: ackPacket(1)},
			{Reply: ackPacket(1)}, {Reply: ackPacket(1)},
		},
	}
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	// 1000000 baud = index 0 in DefaultBaudRates.
	if err := servo.SetBaudRate(context.Background(), 1000000); err != nil {
		t.Fatalf("SetBaudRate: %v", err)
	}

	if len(mock.WriteData) != 32 {
		t.Fatalf("expected 32 bytes, got %d", len(mock.WriteData))
	}
	// Packet 1: torque-disable.
	if mock.WriteData[5] != RegTorqueEnable.Address || mock.WriteData[6] != 0 {
		t.Errorf("packet 1 not torque-disable: addr=%02X val=%02X", mock.WriteData[5], mock.WriteData[6])
	}
	// Packet 3: baud-rate write (addr 6, val 0 = 1Mbps index).
	if mock.WriteData[16+5] != RegBaudRate.Address || mock.WriteData[16+6] != 0 {
		t.Errorf("packet 3 not baud write: addr=%02X val=%02X want 06 00", mock.WriteData[16+5], mock.WriteData[16+6])
	}
}

// TestServo_SetID_ConditionFlagOnTorqueDisable_Proceeds verifies that Task 1's
// write-tolerant contract flows through SetID's safety step for free:
// SetTorqueEnabled writes RegTorqueEnable (SRAM, non-EEPROM), which routes
// through Bus.WriteRegister -> writeRegisterLocked and now returns nil on a
// condition flag. A flagged torque-disable ack must not abort SetID -- the
// dance proceeds through unlock, ID write, and re-lock, and the servo's ID
// is updated.
func TestServo_SetID_ConditionFlagOnTorqueDisable_Proceeds(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Reply: errPacket(1, byte(ErrOverload))}, // torque-disable, condition flag only
			{Reply: ackPacket(1)},                    // unlock
			{Reply: ackPacket(1)},                    // id write
			{Reply: ackPacket(1)},                    // relock
		},
	}
	bus := newTestBus(t, mock)
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	if err := servo.SetID(context.Background(), 7); err != nil {
		t.Fatalf("SetID: condition flag on torque-disable must not abort: %v", err)
	}
	if servo.ID() != 7 {
		t.Errorf("servo.ID() = %d, want 7", servo.ID())
	}

	// The full 4-packet sequence must have been sent: torque-disable, unlock,
	// id-write, relock.
	if len(mock.WriteData) != 32 {
		t.Fatalf("expected 32 bytes (4 packets), got %d: %X", len(mock.WriteData), mock.WriteData)
	}
	if mock.WriteData[5] != RegTorqueEnable.Address || mock.WriteData[6] != 0 {
		t.Errorf("packet 1 not torque-disable: addr=%02X val=%02X", mock.WriteData[5], mock.WriteData[6])
	}
	if mock.WriteData[8+5] != 55 || mock.WriteData[8+6] != 0 {
		t.Errorf("packet 2 not unlock: addr=%02X val=%02X", mock.WriteData[8+5], mock.WriteData[8+6])
	}
	if mock.WriteData[16+5] != RegID.Address || mock.WriteData[16+6] != 7 {
		t.Errorf("packet 3 not id write: addr=%02X val=%02X", mock.WriteData[16+5], mock.WriteData[16+6])
	}
	if mock.WriteData[24+5] != 55 || mock.WriteData[24+6] != 1 {
		t.Errorf("packet 4 not relock: addr=%02X val=%02X", mock.WriteData[24+5], mock.WriteData[24+6])
	}
}

// TestServo_SetID_RejectionOnTorqueDisable_Aborts pins the half of the
// contract that must keep working: a rejection flag (checksum/instruction/
// range, not a condition flag) on the torque-disable ack means the servo
// genuinely did not accept the instruction, so SetID must still abort with
// "failed to disable torque" and never transmit the unlock or ID-write
// packets.
func TestServo_SetID_RejectionOnTorqueDisable_Aborts(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Reply: errPacket(1, byte(ErrChecksum))}, // torque-disable, rejected
		},
	}
	bus := newTestBus(t, mock)
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	err := servo.SetID(context.Background(), 7)
	if err == nil {
		t.Fatal("expected SetID to abort on a rejected torque-disable")
	}
	if !strings.Contains(err.Error(), "failed to disable torque") {
		t.Errorf("error = %v, want it to mention %q", err, "failed to disable torque")
	}
	if !errors.Is(err, ErrChecksum) {
		t.Errorf("expected ErrChecksum in error chain, got: %v", err)
	}
	if servo.ID() != 1 {
		t.Errorf("servo.ID() = %d, want unchanged 1", servo.ID())
	}

	// Only the torque-disable packet should have been sent -- unlock and the
	// ID write must never be transmitted.
	if len(mock.WriteData) != 8 {
		t.Fatalf("expected 8 bytes (torque-disable only, aborted), got %d: %X", len(mock.WriteData), mock.WriteData)
	}
}

// TestServo_SetBaudRate_ConditionFlagOnTorqueDisable_Proceeds mirrors
// TestServo_SetID_ConditionFlagOnTorqueDisable_Proceeds for SetBaudRate.
func TestServo_SetBaudRate_ConditionFlagOnTorqueDisable_Proceeds(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Reply: errPacket(1, byte(ErrOverload))}, // torque-disable, condition flag only
			{Reply: ackPacket(1)},                    // unlock
			{Reply: ackPacket(1)},                    // baud-rate write
			{Reply: ackPacket(1)},                    // relock
		},
	}
	bus := newTestBus(t, mock)
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	// 1000000 baud = index 0 in DefaultBaudRates.
	if err := servo.SetBaudRate(context.Background(), 1000000); err != nil {
		t.Fatalf("SetBaudRate: condition flag on torque-disable must not abort: %v", err)
	}

	if len(mock.WriteData) != 32 {
		t.Fatalf("expected 32 bytes (4 packets), got %d: %X", len(mock.WriteData), mock.WriteData)
	}
	if mock.WriteData[5] != RegTorqueEnable.Address || mock.WriteData[6] != 0 {
		t.Errorf("packet 1 not torque-disable: addr=%02X val=%02X", mock.WriteData[5], mock.WriteData[6])
	}
	if mock.WriteData[8+5] != 55 || mock.WriteData[8+6] != 0 {
		t.Errorf("packet 2 not unlock: addr=%02X val=%02X", mock.WriteData[8+5], mock.WriteData[8+6])
	}
	if mock.WriteData[16+5] != RegBaudRate.Address || mock.WriteData[16+6] != 0 {
		t.Errorf("packet 3 not baud write: addr=%02X val=%02X", mock.WriteData[16+5], mock.WriteData[16+6])
	}
	if mock.WriteData[24+5] != 55 || mock.WriteData[24+6] != 1 {
		t.Errorf("packet 4 not relock: addr=%02X val=%02X", mock.WriteData[24+5], mock.WriteData[24+6])
	}
}

// TestServo_SetBaudRate_RejectionOnTorqueDisable_Aborts mirrors
// TestServo_SetID_RejectionOnTorqueDisable_Aborts for SetBaudRate.
func TestServo_SetBaudRate_RejectionOnTorqueDisable_Aborts(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Reply: errPacket(1, byte(ErrChecksum))}, // torque-disable, rejected
		},
	}
	bus := newTestBus(t, mock)
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	err := servo.SetBaudRate(context.Background(), 1000000)
	if err == nil {
		t.Fatal("expected SetBaudRate to abort on a rejected torque-disable")
	}
	if !strings.Contains(err.Error(), "failed to disable torque") {
		t.Errorf("error = %v, want it to mention %q", err, "failed to disable torque")
	}
	if !errors.Is(err, ErrChecksum) {
		t.Errorf("expected ErrChecksum in error chain, got: %v", err)
	}

	// Only the torque-disable packet should have been sent -- unlock and the
	// baud-rate write must never be transmitted.
	if len(mock.WriteData) != 8 {
		t.Fatalf("expected 8 bytes (torque-disable only, aborted), got %d: %X", len(mock.WriteData), mock.WriteData)
	}
}

// TestServo_SetPositionLimits_AutoUnlocks verifies that each of the two
// register writes (min and max angle limits) is wrapped in its own dance.
// Total: 2 dances × 3 packets = 6 packets.
func TestServo_SetPositionLimits_AutoUnlocks(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Reply: ackPacket(1)}, {Reply: ackPacket(1)}, {Reply: ackPacket(1)},
			{Reply: ackPacket(1)}, {Reply: ackPacket(1)}, {Reply: ackPacket(1)},
		},
	}
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	if err := servo.SetPositionLimits(context.Background(), 100, 4000); err != nil {
		t.Fatalf("SetPositionLimits: %v", err)
	}

	// 50 bytes total: unlock(8) + min(9) + relock(8) + unlock(8) + max(9) + relock(8).
	if len(mock.WriteData) != 50 {
		t.Fatalf("expected 50 bytes, got %d: %X", len(mock.WriteData), mock.WriteData)
	}
	// Sanity: first packet is unlock at addr 55, last packet is relock at addr 55.
	if mock.WriteData[5] != 55 || mock.WriteData[6] != 0 {
		t.Errorf("first packet not unlock at 55: addr=%02X val=%02X", mock.WriteData[5], mock.WriteData[6])
	}
	if mock.WriteData[len(mock.WriteData)-2] != 1 {
		t.Errorf("last packet value not 1 (re-lock): %02X", mock.WriteData[len(mock.WriteData)-2])
	}
}

// TestServo_SetOperatingMode_AutoUnlocks verifies the addr-33 write is dance-wrapped.
func TestServo_SetOperatingMode_AutoUnlocks(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Reply: ackPacket(1)}, {Reply: ackPacket(1)}, {Reply: ackPacket(1)},
		},
	}
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	if err := servo.SetOperatingMode(context.Background(), ModeVelocity); err != nil {
		t.Fatalf("SetOperatingMode: %v", err)
	}

	// 24 bytes: unlock(8) + mode(8) + relock(8).
	if len(mock.WriteData) != 24 {
		t.Fatalf("expected 24 bytes, got %d: %X", len(mock.WriteData), mock.WriteData)
	}
	// Packet 2 is the mode write at addr 33.
	if mock.WriteData[8+5] != RegOperatingMode.Address {
		t.Errorf("mode addr: got %02X want %02X", mock.WriteData[8+5], RegOperatingMode.Address)
	}
}

// errPacket returns a 6-byte status packet with the given error flag set.
func errPacket(id byte, errFlag byte) []byte {
	chk := ^(id + 0x02 + errFlag)
	return []byte{0xFF, 0xFF, id, 0x02, errFlag, chk}
}

// TestServo_WriteEEPROM_ToleratesConditionFlagOnWrite verifies that when the
// target write's ack carries a condition flag (overload/overheat/voltage/
// angle limit), the write is treated as landed — not an error — and the
// re-lock still runs normally. Renamed from the old
// TestServo_WriteEEPROM_RelocksOnWriteError, which asserted the pre-Task-1
// contract (any flag = error). Hardware evidence (see bus.go) showed a
// condition flag on a write ack does not mean the write was rejected, so this
// scenario no longer produces an error at all.
func TestServo_WriteEEPROM_ToleratesConditionFlagOnWrite(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Send: nil, Reply: ackPacket(1)},                    // unlock OK
			{Send: nil, Reply: errPacket(1, byte(ErrOverload))}, // write, condition flag only
			{Send: nil, Reply: ackPacket(1)},                    // relock OK
		},
	}
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	werr := servo.WriteRegister(context.Background(), "id", []byte{5})
	if werr != nil {
		t.Fatalf("condition flag on write ack must not error: %v", werr)
	}

	// Verify all three packets were sent (unlock, write, re-lock).
	if len(mock.WriteData) != 24 {
		t.Errorf("expected 24 bytes (3 packets), got %d: %X", len(mock.WriteData), mock.WriteData)
	}
	// Packet 3 should still be the relock at addr 55 with value 1.
	if mock.WriteData[16+5] != 55 || mock.WriteData[16+6] != 1 {
		t.Errorf("relock packet missing or wrong: addr=%02X val=%02X", mock.WriteData[16+5], mock.WriteData[16+6])
	}
}

// TestServo_WriteEEPROM_ToleratesConditionFlagOnRelock verifies that when the
// relock ack carries a condition flag, that's tolerated too — the relock
// landed. Renamed from the old TestServo_WriteEEPROM_JoinedErrorOnRelockError,
// which asserted the pre-Task-1 contract.
func TestServo_WriteEEPROM_ToleratesConditionFlagOnRelock(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Send: nil, Reply: ackPacket(1)},                    // unlock OK
			{Send: nil, Reply: ackPacket(1)},                    // write OK
			{Send: nil, Reply: errPacket(1, byte(ErrOverheat))}, // relock, condition flag only
		},
	}
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	werr := servo.WriteRegister(context.Background(), "id", []byte{5})
	if werr != nil {
		t.Fatalf("condition flag on relock ack must not error: %v", werr)
	}
}

// TestServo_WriteEEPROM_ToleratesConditionFlagsOnBothSteps covers write and
// relock each carrying their own (different) condition flag. Renamed from the
// old TestServo_WriteEEPROM_BothFailJoined, which exercised the errors.Join
// branch in writeEEPROM for a "both steps fail" scenario. Under the new
// write-tolerant contract, a lone condition flag is never a failure, so that
// scenario can no longer be constructed this way — both steps report nil and
// errors.Join is never reached. Coverage of the Join branch itself belongs to
// a genuine (request-flag) failure scenario, which is outside this task.
func TestServo_WriteEEPROM_ToleratesConditionFlagsOnBothSteps(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Send: nil, Reply: ackPacket(1)},                    // unlock OK
			{Send: nil, Reply: errPacket(1, byte(ErrOverload))}, // write, condition flag only
			{Send: nil, Reply: errPacket(1, byte(ErrOverheat))}, // relock, condition flag only
		},
	}
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	werr := servo.WriteRegister(context.Background(), "id", []byte{5})
	if werr != nil {
		t.Fatalf("condition flags on both steps must not error: %v", werr)
	}

	// All three packets should still have been sent.
	if len(mock.WriteData) != 24 {
		t.Errorf("expected 24 bytes (3 packets), got %d", len(mock.WriteData))
	}
}

// TestServo_WriteEEPROM_ToleratesConditionFlagOnUnlock verifies that a
// condition flag on the unlock ack does not abort the dance: the unlock
// genuinely landed (hardware evidence: the lock register read back 0 despite
// the flagged ack), so the write and re-lock still proceed. Renamed from the
// old TestServo_WriteEEPROM_UnlockFailDoesNotWrite, which asserted the
// pre-Task-1 contract that any flag on the unlock step aborts the dance.
func TestServo_WriteEEPROM_ToleratesConditionFlagOnUnlock(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Send: nil, Reply: errPacket(1, byte(ErrOverheat))}, // unlock, condition flag only
			{Send: nil, Reply: ackPacket(1)},                    // write OK
			{Send: nil, Reply: ackPacket(1)},                    // relock OK
		},
	}
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	werr := servo.WriteRegister(context.Background(), "id", []byte{5})
	if werr != nil {
		t.Fatalf("condition flag on unlock ack must not error: %v", werr)
	}
	// All three packets should have been sent — the dance was not aborted.
	if len(mock.WriteData) != 24 {
		t.Errorf("expected 24 bytes (3 packets), got %d: %X", len(mock.WriteData), mock.WriteData)
	}
}

// TestServo_WriteEEPROM_RejectionAbortsUnlock verifies that when the unlock
// write itself is rejected (a request flag -- checksum/instruction/range, not
// a condition flag), writeEEPROM aborts immediately: neither the target write
// nor the relock packet is sent, and the error propagates. Restores coverage
// of servo.go:446-448 ("abort — no further packets are sent"), which the four
// Tolerates* tests above no longer exercise because they now script condition
// flags, which never abort the dance under the new write-tolerant contract.
func TestServo_WriteEEPROM_RejectionAbortsUnlock(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Send: nil, Reply: errPacket(1, byte(ErrChecksum))}, // unlock rejected
		},
	}
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	werr := servo.WriteRegister(context.Background(), "id", []byte{5})
	if werr == nil {
		t.Fatal("expected error when unlock is rejected")
	}
	if !errors.Is(werr, ErrChecksum) {
		t.Errorf("expected ErrChecksum in error chain, got: %v", werr)
	}
	// Only the unlock packet (8 bytes) should have been sent -- the dance
	// aborted before the target write or the relock.
	if len(mock.WriteData) != 8 {
		t.Errorf("expected 8 bytes (unlock only, dance aborted), got %d: %X", len(mock.WriteData), mock.WriteData)
	}
}

// TestServo_WriteEEPROM_RejectionErrors covers the three ways a completed
// dance (unlock always succeeds) can still return an error under the
// write-tolerant contract: a rejection flag (checksum/instruction/range) on
// the target write, on the relock, or on both. In every case the relock
// packet is still sent -- restoring the "relock is always attempted" pin from
// servo.go:453 alongside the specific return-value branches at
// servo.go:458 (both failed -> errors.Join), :460 (write failed only), and
// :462 (relock failed only). The condition-flag versions of these scenarios
// no longer error at all post-Task-1, which is what dropped this coverage.
func TestServo_WriteEEPROM_RejectionErrors(t *testing.T) {
	tests := []struct {
		name       string
		writeFlag  StatusError // 0 = clean ack
		relockFlag StatusError // 0 = clean ack
		wantFlags  []StatusError
	}{
		{
			name:      "target write rejected, relock clean",
			writeFlag: ErrChecksum,
			wantFlags: []StatusError{ErrChecksum},
		},
		{
			name:       "target write clean, relock rejected",
			relockFlag: ErrChecksum,
			wantFlags:  []StatusError{ErrChecksum},
		},
		{
			name:       "both target write and relock rejected",
			writeFlag:  ErrChecksum,
			relockFlag: ErrInstruction,
			wantFlags:  []StatusError{ErrChecksum, ErrInstruction},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			writeReply := ackPacket(1)
			if tt.writeFlag != 0 {
				writeReply = errPacket(1, byte(tt.writeFlag))
			}
			relockReply := ackPacket(1)
			if tt.relockFlag != 0 {
				relockReply = errPacket(1, byte(tt.relockFlag))
			}

			mock := &transports.MockTransport{}
			mock.Script = &transports.Script{
				Steps: []transports.Step{
					{Send: nil, Reply: ackPacket(1)}, // unlock OK
					{Send: nil, Reply: writeReply},
					{Send: nil, Reply: relockReply},
				},
			}
			bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
			if err != nil {
				t.Fatalf("NewBus: %v", err)
			}
			defer bus.Close()

			servo := NewServo(bus, 1, nil)
			werr := servo.WriteRegister(context.Background(), "id", []byte{5})
			if werr == nil {
				t.Fatal("expected error")
			}
			for _, flag := range tt.wantFlags {
				if !errors.Is(werr, flag) {
					t.Errorf("expected %v in error chain, got: %v", flag, werr)
				}
			}
			// The relock is always attempted, even when the target write
			// failed -- all three packets must have been sent regardless of
			// which step(s) reported a rejection.
			if len(mock.WriteData) != 24 {
				t.Errorf("expected 24 bytes (3 packets), got %d: %X", len(mock.WriteData), mock.WriteData)
			}
		})
	}
}

// TestServo_SCS_LockUsesAddr48 verifies that on a Bus configured for ProtocolSCS
// using ModelSCS0009, the lock writes target addr 48, not addr 55.
func TestServo_SCS_LockUsesAddr48(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Reply: ackPacket(1)}, {Reply: ackPacket(1)}, {Reply: ackPacket(1)},
		},
	}
	bus, err := NewBus(BusConfig{
		Transport: mock,
		Protocol:  ProtocolSCS,
		Timeout:   100 * time.Millisecond,
	})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	scs, _ := GetModel("scs0009")
	servo := NewServo(bus, 1, scs)

	if err := servo.WriteRegister(context.Background(), "id", []byte{5}); err != nil {
		t.Fatalf("WriteRegister: %v", err)
	}

	// Three 8-byte packets.
	if len(mock.WriteData) != 24 {
		t.Fatalf("expected 24 bytes, got %d: %X", len(mock.WriteData), mock.WriteData)
	}
	// Packet 1: unlock at addr 48 (0x30).
	if mock.WriteData[5] != 48 {
		t.Errorf("packet 1 lock addr: got %02X want 30 (48)", mock.WriteData[5])
	}
	// Packet 3: re-lock at addr 48.
	if mock.WriteData[16+5] != 48 {
		t.Errorf("packet 3 lock addr: got %02X want 30 (48)", mock.WriteData[16+5])
	}
}

// TestServo_NoLockAddress_SkipsDance constructs a custom model with no lock
// register and verifies EEPROM-marked writes go straight through without the dance.
func TestServo_NoLockAddress_SkipsDance(t *testing.T) {
	mock := &transports.MockTransport{
		ReadData: ackPacket(1),
	}
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	noLock := &Model{
		Name:        "test_nolock",
		Number:      9999,
		Protocol:    ProtocolSTS,
		Resolution:  4096,
		MaxPosition: 4095,
		BaudRates:   DefaultBaudRates,
		LockAddress: 0, // explicit: no lock register
	}
	servo := NewServo(bus, 1, noLock)
	// id is EEPROM=true on STS, but the model says no lock — should still be a single write.
	if err := servo.WriteRegister(context.Background(), "id", []byte{5}); err != nil {
		t.Fatalf("WriteRegister: %v", err)
	}

	// Single 8-byte packet, no lock writes.
	if len(mock.WriteData) != 8 {
		t.Fatalf("expected 8 bytes (1 packet only), got %d: %X", len(mock.WriteData), mock.WriteData)
	}
	if mock.WriteData[5] != RegID.Address {
		t.Errorf("addr: got %02X want %02X", mock.WriteData[5], RegID.Address)
	}
}
