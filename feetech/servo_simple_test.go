package feetech

import (
	"context"
	"encoding/binary"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/transports"
)

func ackResponse() []byte {
	return []byte{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC}
}

func newTestBus(t *testing.T, mock *transports.MockTransport) *Bus {
	t.Helper()
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	return bus
}

func TestServo_Velocity_DecodesSignMagnitude(t *testing.T) {
	// Negative velocity: 0x8064 = sign bit set, magnitude 0x0064 = 100 -> -100.
	// Little-endian encoding: bytes 64 80.
	mock := &transports.MockTransport{
		ReadData: []byte{0xFF, 0xFF, 0x01, 0x04, 0x00, 0x64, 0x80, 0x16},
	}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)

	v, err := servo.Velocity(context.Background())
	if err != nil {
		t.Fatalf("Velocity: %v", err)
	}
	if v != -100 {
		t.Errorf("velocity: got %d want -100", v)
	}
}

func TestServo_SetVelocity_EncodesSignMagnitude(t *testing.T) {
	mock := &transports.MockTransport{ReadData: ackResponse()}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)

	if err := servo.SetVelocity(context.Background(), -100); err != nil {
		t.Fatalf("SetVelocity: %v", err)
	}
	// Address (RegGoalVelocity = 46) + 2 bytes encoded little-endian sign-magnitude.
	// -100 with bit15 set: 0x8064 -> bytes 64 80.
	if mock.WriteData[5] != RegGoalVelocity.Address {
		t.Errorf("address: got %02X want %02X", mock.WriteData[5], RegGoalVelocity.Address)
	}
	got := binary.LittleEndian.Uint16(mock.WriteData[6:8])
	if got != 0x8064 {
		t.Errorf("encoded value: got %04X want 8064", got)
	}
}

func TestServo_Voltage(t *testing.T) {
	mock := &transports.MockTransport{
		ReadData: []byte{0xFF, 0xFF, 0x01, 0x03, 0x00, 0x78, 0x83},
	}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	v, err := servo.Voltage(context.Background())
	if err != nil || v != 0x78 {
		t.Errorf("Voltage: got %d err=%v want 120", v, err)
	}
}

func TestServo_Temperature(t *testing.T) {
	mock := &transports.MockTransport{
		ReadData: []byte{0xFF, 0xFF, 0x01, 0x03, 0x00, 0x2A, 0xD1},
	}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	temp, err := servo.Temperature(context.Background())
	if err != nil || temp != 0x2A {
		t.Errorf("Temperature: got %d err=%v want 42", temp, err)
	}
}

func TestServo_Moving(t *testing.T) {
	mock := &transports.MockTransport{
		ReadData: []byte{0xFF, 0xFF, 0x01, 0x03, 0x00, 0x01, 0xFA},
	}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	moving, err := servo.Moving(context.Background())
	if err != nil || !moving {
		t.Errorf("Moving: got %v err=%v want true", moving, err)
	}
}

func TestServo_TorqueEnabled(t *testing.T) {
	mock := &transports.MockTransport{
		ReadData: []byte{0xFF, 0xFF, 0x01, 0x03, 0x00, 0x01, 0xFA},
	}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	en, err := servo.TorqueEnabled(context.Background())
	if err != nil || !en {
		t.Errorf("TorqueEnabled: got %v err=%v want true", en, err)
	}
}

func TestServo_OperatingMode(t *testing.T) {
	mock := &transports.MockTransport{
		ReadData: []byte{0xFF, 0xFF, 0x01, 0x03, 0x00, 0x01, 0xFA},
	}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	mode, err := servo.OperatingMode(context.Background())
	if err != nil || mode != ModeVelocity {
		t.Errorf("OperatingMode: got %d err=%v want %d", mode, err, ModeVelocity)
	}
}

func TestServo_SetOperatingMode(t *testing.T) {
	// Auto-unlock dance: unlock (addr 55, 0) → mode write (addr 33) → re-lock (addr 55, 1).
	// Use Script (not bytes.Repeat in ReadData) because the bus reads up to 12 bytes
	// per Read call, which silently drops trailing acks pre-loaded in ReadData.
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Reply: ackResponse()},
			{Reply: ackResponse()},
			{Reply: ackResponse()},
		},
	}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	if err := servo.SetOperatingMode(context.Background(), ModeVelocity); err != nil {
		t.Fatalf("SetOperatingMode: %v", err)
	}

	// Three 8-byte write packets total.
	if len(mock.WriteData) != 24 {
		t.Fatalf("expected 24 bytes (3 packets × 8 bytes), got %d: %X", len(mock.WriteData), mock.WriteData)
	}
	// Packet 2 (offset 8) is the mode write.
	const p2Addr = 8 + 5
	const p2Value = 8 + 6
	if mock.WriteData[p2Addr] != RegOperatingMode.Address {
		t.Errorf("wrong mode address: %02X", mock.WriteData[p2Addr])
	}
	if mock.WriteData[p2Value] != byte(ModeVelocity) {
		t.Errorf("wrong mode: %02X", mock.WriteData[p2Value])
	}
}

func TestServo_PositionLimits(t *testing.T) {
	// Two reads: min limit + max limit. Use ReadFunc to deliver each response separately.
	responses := [][]byte{
		{0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x00, 0xFA}, // min = 0
		{0xFF, 0xFF, 0x01, 0x04, 0x00, 0xFF, 0x0F, 0xEC}, // max = 4095
	}
	idx := 0
	mock := &transports.MockTransport{}
	mock.ReadFunc = func(p []byte) (int, error) {
		if idx >= len(responses) {
			return 0, nil
		}
		n := copy(p, responses[idx])
		idx++
		return n, nil
	}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	min, max, err := servo.PositionLimits(context.Background())
	if err != nil {
		t.Fatalf("PositionLimits: %v", err)
	}
	if min != 0 || max != 4095 {
		t.Errorf("limits: got min=%d max=%d want 0/4095", min, max)
	}
}

func TestServo_SetPositionLimits(t *testing.T) {
	// Two register writes, each its own dance. Total: 6 packets.
	// Order: unlock → write min → relock → unlock → write max → relock.
	// Use Script: pre-loading multiple acks in ReadData breaks because the bus reads
	// up to 12 bytes per Read call, silently dropping trailing acks.
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			{Reply: ackResponse()}, {Reply: ackResponse()}, {Reply: ackResponse()},
			{Reply: ackResponse()}, {Reply: ackResponse()}, {Reply: ackResponse()},
		},
	}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	if err := servo.SetPositionLimits(context.Background(), 100, 4000); err != nil {
		t.Fatalf("SetPositionLimits: %v", err)
	}

	// Min limit is a 2-byte write, so its packet is 9 bytes (vs 8 for 1-byte writes).
	// Lock writes are 1-byte: 8 bytes each.
	// Layout: unlock(8) | min(9) | relock(8) | unlock(8) | max(9) | relock(8) = 50 bytes.
	if len(mock.WriteData) != 50 {
		t.Fatalf("expected 50 bytes, got %d: %X", len(mock.WriteData), mock.WriteData)
	}

	// Packet 2 (offset 8) is the min-limit write at addr RegMinAngleLimit.Address.
	if mock.WriteData[8+5] != RegMinAngleLimit.Address {
		t.Errorf("first write address: %02X", mock.WriteData[8+5])
	}
	// Packet 5 (offset 8+9+8+8 = 33) is the max-limit write at addr RegMaxAngleLimit.Address.
	if mock.WriteData[33+5] != RegMaxAngleLimit.Address {
		t.Errorf("second write address: %02X", mock.WriteData[33+5])
	}
}

func TestServo_Disable(t *testing.T) {
	mock := &transports.MockTransport{ReadData: ackResponse()}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	if err := servo.Disable(context.Background()); err != nil {
		t.Fatalf("Disable: %v", err)
	}
	if mock.WriteData[5] != RegTorqueEnable.Address {
		t.Errorf("wrong address: %02X", mock.WriteData[5])
	}
	if mock.WriteData[6] != 0 {
		t.Errorf("wrong value: %02X", mock.WriteData[6])
	}
}

func TestServo_ModelGetSet(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	if servo.Model().Name != "sts3215" {
		t.Errorf("default model: got %s", servo.Model().Name)
	}
	scs, _ := GetModel("scs0009")
	servo.SetModel(scs)
	if servo.Model().Name != "scs0009" {
		t.Errorf("after SetModel: got %s", servo.Model().Name)
	}
}

func TestBus_RegWrite(t *testing.T) {
	mock := &transports.MockTransport{ReadData: ackResponse()}
	bus := newTestBus(t, mock)
	defer bus.Close()
	if err := bus.RegWrite(context.Background(), 1, RegGoalPosition.Address, []byte{0x00, 0x08}); err != nil {
		t.Fatalf("RegWrite: %v", err)
	}
	if mock.WriteData[4] != InstRegWrite {
		t.Errorf("instruction: got %02X want %02X", mock.WriteData[4], InstRegWrite)
	}
}

func TestBus_Action(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	if err := bus.Action(context.Background()); err != nil {
		t.Fatalf("Action: %v", err)
	}
	if mock.WriteData[2] != BroadcastID {
		t.Errorf("Action should broadcast: %02X", mock.WriteData[2])
	}
	if mock.WriteData[4] != InstAction {
		t.Errorf("Action instruction: %02X", mock.WriteData[4])
	}
}

func TestBus_Scan(t *testing.T) {
	// One ack for ID 1 ping + one model-number read (777=0x0309), then nothing for ID 2 (timeout).
	// Scan proceeds in order: ping ID 1 -> ack -> read model -> success; ping ID 2 -> timeout, skip.
	mock := &transports.MockTransport{}
	calls := 0
	mock.ReadFunc = func(p []byte) (int, error) {
		calls++
		switch calls {
		case 1: // ping ID 1
			return copy(p, []byte{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC}), nil
		case 2: // model-number read for ID 1
			return copy(p, []byte{0xFF, 0xFF, 0x01, 0x04, 0x00, 0x09, 0x03, 0xEE}), nil
		default: // ping ID 2 -> nothing
			return 0, nil
		}
	}
	bus := newTestBus(t, mock)
	defer bus.Close()
	ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
	defer cancel()
	found, err := bus.Scan(ctx, 1, 2)
	if err != nil {
		t.Fatalf("Scan: %v", err)
	}
	if len(found) != 1 || found[0].ID != 1 || found[0].ModelNumber != 777 {
		t.Errorf("found = %+v", found)
	}
}

func TestProtocol_ByteOrder(t *testing.T) {
	if NewProtocol(ProtocolSTS).ByteOrder() != binary.LittleEndian {
		t.Error("STS should be little-endian")
	}
	if NewProtocol(ProtocolSCS).ByteOrder() != binary.BigEndian {
		t.Error("SCS should be big-endian")
	}
}

func TestProtocol_RegWritePacket(t *testing.T) {
	p := NewProtocol(ProtocolSTS)
	got := p.RegWritePacket(0x01, 0x2A, []byte{0x00, 0x08})
	// header(2) + id(1) + length(1) + inst(1) + addr(1) + data(2) + chk(1) = 9 bytes.
	if len(got) != 9 {
		t.Errorf("len: got %d want 9", len(got))
	}
	if got[4] != InstRegWrite {
		t.Errorf("inst: %02X", got[4])
	}
}

func TestServoGroup_Introspection(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	g := NewServoGroupByIDs(bus, 1, 2, 3)

	if len(g.Servos()) != 3 {
		t.Errorf("Servos len = %d", len(g.Servos()))
	}
	if len(g.IDs()) != 3 {
		t.Errorf("IDs len = %d", len(g.IDs()))
	}
	if g.Servo(0) == nil || g.Servo(2) == nil {
		t.Error("Servo(0)/Servo(2) should be non-nil")
	}
	if g.Servo(-1) != nil || g.Servo(3) != nil {
		t.Error("Servo(out-of-range) should be nil")
	}
}

func TestServoGroup_DisableAll(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	g := NewServoGroupByIDs(bus, 1, 2)
	if err := g.DisableAll(context.Background()); err != nil {
		t.Fatalf("DisableAll: %v", err)
	}
	if mock.WriteData[2] != BroadcastID {
		t.Errorf("not broadcast: %02X", mock.WriteData[2])
	}
	if mock.WriteData[4] != InstSyncWrite {
		t.Errorf("instruction: %02X", mock.WriteData[4])
	}
	if mock.WriteData[5] != RegTorqueEnable.Address {
		t.Errorf("address: %02X", mock.WriteData[5])
	}
}

// TestServo_Accessors_ReturnValueWithConditionFlag asserts every read accessor
// hands back its decoded value even when the servo reports a condition flag.
func TestServo_Accessors_ReturnValueWithConditionFlag(t *testing.T) {
	ovl := byte(ErrOverload)

	tests := []struct {
		name  string
		reply []byte
		read  func(*Servo) (int, error)
		want  int
	}{
		{
			// Hardware capture: position 2221 while overloaded.
			name:  "Position",
			reply: readReplyPacket(6, ovl, 0xAD, 0x08),
			read:  func(s *Servo) (int, error) { return s.Position(context.Background()) },
			want:  2221,
		},
		{
			// Hardware capture: load 200 (post-trip protection torque).
			name:  "Load",
			reply: readReplyPacket(6, ovl, 0xC8, 0x00),
			read:  func(s *Servo) (int, error) { return s.Load(context.Background()) },
			want:  200,
		},
		{
			name:  "Temperature",
			reply: readReplyPacket(6, ovl, 39),
			read:  func(s *Servo) (int, error) { return s.Temperature(context.Background()) },
			want:  39,
		},
		{
			name:  "Voltage",
			reply: readReplyPacket(6, ovl, 74),
			read:  func(s *Servo) (int, error) { return s.Voltage(context.Background()) },
			want:  74,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			mock := &transports.MockTransport{}
			mock.Script = &transports.Script{Steps: []transports.Step{{Reply: tt.reply}}}

			bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
			if err != nil {
				t.Fatalf("NewBus: %v", err)
			}
			defer bus.Close()

			got, err := tt.read(NewServo(bus, 6, nil))

			if got != tt.want {
				t.Errorf("value: got %d, want %d", got, tt.want)
			}
			flags, ok := ConditionStatus(err)
			if !ok || flags != ErrOverload {
				t.Errorf("ConditionStatus: got (%v, %v), want (ErrOverload, true)", flags, ok)
			}
		})
	}
}

// TestServo_PositionLimits_ConditionFlags covers PositionLimits' non-mechanical
// branches: both reads flagged (values survive, flag reported), and the second
// read hard-failing after the first succeeded (good min discarded, 0,0 returned).
func TestServo_PositionLimits_ConditionFlags(t *testing.T) {
	t.Run("min error wins when both differ", func(t *testing.T) {
		// Min and max carry different condition flags: this is the only case that
		// actually distinguishes minErr-first precedence from maxErr-first — when
		// both flags are equal (as in "both flagged overload" below), either order
		// produces the same visible error.
		mock := &transports.MockTransport{}
		mock.Script = &transports.Script{Steps: []transports.Step{
			{Reply: readReplyPacket(1, byte(ErrVoltage), 0x64, 0x00)},  // min = 100
			{Reply: readReplyPacket(1, byte(ErrOverheat), 0xA0, 0x0F)}, // max = 4000
		}}

		bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
		if err != nil {
			t.Fatalf("NewBus: %v", err)
		}
		defer bus.Close()

		min, max, err := NewServo(bus, 1, nil).PositionLimits(context.Background())
		if min != 100 || max != 4000 {
			t.Errorf("limits: got min=%d max=%d, want 100/4000", min, max)
		}
		flags, ok := ConditionStatus(err)
		if !ok || flags != ErrVoltage {
			t.Errorf("ConditionStatus: got (%v, %v), want (ErrVoltage, true) — minErr must win", flags, ok)
		}
	})

	t.Run("both flagged overload", func(t *testing.T) {
		mock := &transports.MockTransport{}
		mock.Script = &transports.Script{Steps: []transports.Step{
			{Reply: readReplyPacket(1, byte(ErrOverload), 0x64, 0x00)}, // min = 100
			{Reply: readReplyPacket(1, byte(ErrOverload), 0xA0, 0x0F)}, // max = 4000
		}}

		bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
		if err != nil {
			t.Fatalf("NewBus: %v", err)
		}
		defer bus.Close()

		min, max, err := NewServo(bus, 1, nil).PositionLimits(context.Background())
		if min != 100 || max != 4000 {
			t.Errorf("limits: got min=%d max=%d, want 100/4000", min, max)
		}
		flags, ok := ConditionStatus(err)
		if !ok || flags != ErrOverload {
			t.Errorf("ConditionStatus: got (%v, %v), want (ErrOverload, true)", flags, ok)
		}
	})

	t.Run("max hard-fails after min succeeds", func(t *testing.T) {
		mock := &transports.MockTransport{}
		mock.Script = &transports.Script{Steps: []transports.Step{
			{Reply: readReplyPacket(1, 0x00, 0x64, 0x00)},              // min = 100, no flag
			{Reply: readReplyPacket(1, byte(ErrChecksum), 0xA0, 0x0F)}, // request-rejected, no payload
		}}

		bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
		if err != nil {
			t.Fatalf("NewBus: %v", err)
		}
		defer bus.Close()

		min, max, err := NewServo(bus, 1, nil).PositionLimits(context.Background())
		if min != 0 || max != 0 {
			t.Errorf("limits: got min=%d max=%d, want 0/0 (good min discarded)", min, max)
		}
		if err == nil {
			t.Fatal("expected an error")
		}
		if _, ok := ConditionStatus(err); ok {
			t.Error("ConditionStatus: got ok=true, want false for a request-rejection flag")
		}
	})
}
