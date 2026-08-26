package feetech

import (
	"context"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/transports"
)

// scriptedReader returns a ReadFunc that yields each response in order,
// then returns 0,nil (which the bus treats as "more data not yet available").
func scriptedReader(responses [][]byte) func(p []byte) (int, error) {
	idx := 0
	return func(p []byte) (int, error) {
		if idx >= len(responses) {
			return 0, nil
		}
		n := copy(p, responses[idx])
		idx++
		return n, nil
	}
}

func TestServo_Ping(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.ReadFunc = scriptedReader([][]byte{
		{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC},             // ping ack
		{0xFF, 0xFF, 0x01, 0x04, 0x00, 0x09, 0x03, 0xEE}, // model 777
	})
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	num, err := servo.Ping(context.Background())
	if err != nil {
		t.Fatalf("Ping: %v", err)
	}
	if num != 777 {
		t.Errorf("model num: got %d want 777", num)
	}
}

func TestServo_DetectModel(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.ReadFunc = scriptedReader([][]byte{
		{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC},             // ping ack
		{0xFF, 0xFF, 0x01, 0x04, 0x00, 0x09, 0x03, 0xEE}, // model 777
	})
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	if err := servo.DetectModel(context.Background()); err != nil {
		t.Fatalf("DetectModel: %v", err)
	}
	if servo.Model().Name != "sts3215" {
		t.Errorf("detected model: got %s want sts3215", servo.Model().Name)
	}
}

func TestServo_SetPositionWithSpeed(t *testing.T) {
	mock := &transports.MockTransport{ReadData: ackResponse()}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	if err := servo.SetPositionWithSpeed(context.Background(), 2048, 500); err != nil {
		t.Fatalf("SetPositionWithSpeed: %v", err)
	}
	// Address byte at index 5 should be RegGoalPosition.
	if mock.WriteData[5] != RegGoalPosition.Address {
		t.Errorf("address: %02X", mock.WriteData[5])
	}
	// Length byte (data length 6 + 3) at index 3.
	if mock.WriteData[3] != 9 {
		t.Errorf("length: got %d want 9", mock.WriteData[3])
	}
}

func TestServo_SetPositionWithTime(t *testing.T) {
	mock := &transports.MockTransport{ReadData: ackResponse()}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	if err := servo.SetPositionWithTime(context.Background(), 1024, 1000); err != nil {
		t.Fatalf("SetPositionWithTime: %v", err)
	}
	if mock.WriteData[5] != RegGoalPosition.Address {
		t.Errorf("address: %02X", mock.WriteData[5])
	}
}

func TestServo_Load(t *testing.T) {
	// Load is at RegPresentLoad (size 2, signbit 10). Encode +50: 0x0032 -> bytes 32 00.
	mock := &transports.MockTransport{
		ReadData: []byte{0xFF, 0xFF, 0x01, 0x04, 0x00, 0x32, 0x00, 0xC8},
	}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	v, err := servo.Load(context.Background())
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if v != 50 {
		t.Errorf("load: got %d want 50", v)
	}
}

func TestServo_ReadRegister_ByName(t *testing.T) {
	mock := &transports.MockTransport{
		ReadData: []byte{0xFF, 0xFF, 0x01, 0x03, 0x00, 0x01, 0xFA},
	}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	data, err := servo.ReadRegister(context.Background(), "torque_enable")
	if err != nil {
		t.Fatalf("ReadRegister: %v", err)
	}
	if len(data) != 1 || data[0] != 1 {
		t.Errorf("data: got %v want [1]", data)
	}
}

func TestServo_ReadRegister_UnknownName(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	if _, err := servo.ReadRegister(context.Background(), "no_such_reg"); err == nil {
		t.Error("ReadRegister(unknown): expected error")
	}
}

func TestServo_WriteRegister_ByName(t *testing.T) {
	mock := &transports.MockTransport{ReadData: ackResponse()}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	if err := servo.WriteRegister(context.Background(), "torque_enable", []byte{1}); err != nil {
		t.Fatalf("WriteRegister: %v", err)
	}
}

func TestServo_WriteRegister_UnknownName(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	if err := servo.WriteRegister(context.Background(), "no_such_reg", []byte{0}); err == nil {
		t.Error("WriteRegister(unknown): expected error")
	}
}

func TestServo_WriteRegister_ReadOnly(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	if err := servo.WriteRegister(context.Background(), "present_position", []byte{0, 0}); err == nil {
		t.Error("WriteRegister(read-only): expected error")
	}
}

func TestServo_WriteRegister_SizeMismatch(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	servo := NewServo(bus, 1, nil)
	// torque_enable is 1 byte, give it 2.
	if err := servo.WriteRegister(context.Background(), "torque_enable", []byte{0, 0}); err == nil {
		t.Error("WriteRegister(size mismatch): expected error")
	}
}

func TestServoGroup_SetPositionsWithSpeed(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	g := NewServoGroupByIDs(bus, 1, 2)
	positions := PositionMap{1: 1024, 2: 2048}
	speeds := PositionMap{1: 500, 2: 600}
	if err := g.SetPositionsWithSpeed(context.Background(), positions, speeds); err != nil {
		t.Fatalf("SetPositionsWithSpeed: %v", err)
	}
	// Should be a sync write (broadcast).
	if mock.WriteData[2] != BroadcastID {
		t.Errorf("expected broadcast: %02X", mock.WriteData[2])
	}
	if mock.WriteData[4] != InstSyncWrite {
		t.Errorf("instruction: %02X", mock.WriteData[4])
	}
}

func TestServoGroup_SetPositionsWithSpeed_EmptyMap(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	g := NewServoGroupByIDs(bus, 1)
	// Empty maps -> no-op, no write.
	if err := g.SetPositionsWithSpeed(context.Background(), PositionMap{}, PositionMap{}); err != nil {
		t.Fatalf("SetPositionsWithSpeed(empty): %v", err)
	}
	if len(mock.WriteData) != 0 {
		t.Errorf("expected no write, got %d bytes", len(mock.WriteData))
	}
}

func TestServoGroup_SetPositionsWithSpeed_NoIntersection(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	g := NewServoGroupByIDs(bus, 1, 2)
	// No common keys -> no write.
	if err := g.SetPositionsWithSpeed(context.Background(), PositionMap{1: 100}, PositionMap{2: 500}); err != nil {
		t.Fatalf("SetPositionsWithSpeed(no intersection): %v", err)
	}
	if len(mock.WriteData) != 0 {
		t.Errorf("expected no write, got %d bytes", len(mock.WriteData))
	}
}

func TestServoGroup_SetPositionsWithTime(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	g := NewServoGroupByIDs(bus, 1, 2)
	positions := PositionMap{1: 1024, 2: 2048}
	times := PositionMap{1: 1000, 2: 2000}
	if err := g.SetPositionsWithTime(context.Background(), positions, times); err != nil {
		t.Fatalf("SetPositionsWithTime: %v", err)
	}
	if mock.WriteData[2] != BroadcastID {
		t.Errorf("expected broadcast: %02X", mock.WriteData[2])
	}
}

func TestServoGroup_SetPositionsWithTime_EmptyMap(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	g := NewServoGroupByIDs(bus, 1)
	if err := g.SetPositionsWithTime(context.Background(), PositionMap{}, PositionMap{}); err != nil {
		t.Fatalf("SetPositionsWithTime(empty): %v", err)
	}
	if len(mock.WriteData) != 0 {
		t.Errorf("expected no write, got %d bytes", len(mock.WriteData))
	}
}

func TestServoGroup_SetPositionsWithTime_NoIntersection(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	g := NewServoGroupByIDs(bus, 1, 2)
	if err := g.SetPositionsWithTime(context.Background(), PositionMap{1: 100}, PositionMap{2: 500}); err != nil {
		t.Fatalf("SetPositionsWithTime(no intersection): %v", err)
	}
	if len(mock.WriteData) != 0 {
		t.Errorf("expected no write, got %d bytes", len(mock.WriteData))
	}
}

func TestServoGroup_RegWritePositions(t *testing.T) {
	// Two RegWrite packets (each expects an ack response).
	mock := &transports.MockTransport{}
	mock.ReadFunc = scriptedReader([][]byte{ackResponse(), ackResponse()})
	bus := newTestBus(t, mock)
	defer bus.Close()
	g := NewServoGroupByIDs(bus, 1, 2)
	if err := g.RegWritePositions(context.Background(), PositionMap{1: 1024, 2: 2048}); err != nil {
		t.Fatalf("RegWritePositions: %v", err)
	}
	// Each packet should have InstRegWrite at offset 4.
	if mock.WriteData[4] != InstRegWrite {
		t.Errorf("first packet instruction: %02X", mock.WriteData[4])
	}
}

func TestServoGroup_RegWritePositions_Empty(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	g := NewServoGroupByIDs(bus, 1)
	if err := g.RegWritePositions(context.Background(), PositionMap{}); err != nil {
		t.Fatalf("RegWritePositions(empty): %v", err)
	}
	if len(mock.WriteData) != 0 {
		t.Errorf("expected no write, got %d bytes", len(mock.WriteData))
	}
}

func TestServoGroup_RegWritePositions_UnknownID(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	g := NewServoGroupByIDs(bus, 1)
	if err := g.RegWritePositions(context.Background(), PositionMap{99: 100}); err == nil {
		t.Error("RegWritePositions(unknown ID): expected error")
	}
}

func TestServoGroup_ReadRegister(t *testing.T) {
	// Use scs0009 model (has populated Registers map).
	mock := &transports.MockTransport{}
	// SyncRead response: per servo, header(2)+id(1)+len(1)+err(1)+data(2)+chk(1)=8 bytes.
	// For 2 servos reading torque_enable (1 byte each):
	// header(2)+id(1)+len(1)+err(1)+data(1)+chk(1)=7 bytes per servo.
	mock.ReadData = []byte{
		0xFF, 0xFF, 0x01, 0x03, 0x00, 0x01, 0xFA, // ID 1, value 1
		0xFF, 0xFF, 0x02, 0x03, 0x00, 0x00, 0xFA, // ID 2, value 0
	}
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond, Protocol: ProtocolSTS})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()
	scs, _ := GetModel("scs0009")
	servos := []*Servo{NewServo(bus, 1, scs), NewServo(bus, 2, scs)}
	g := NewServoGroup(bus, servos...)
	data, err := g.ReadRegister(context.Background(), "torque_enable")
	if err != nil {
		t.Fatalf("ReadRegister: %v", err)
	}
	if len(data) != 2 {
		t.Errorf("expected 2 entries, got %d", len(data))
	}
}

func TestServoGroup_ReadRegister_NoServosHaveIt(t *testing.T) {
	// Use sts3215 (Registers map is nil), then ReadRegister loop won't find any servos.
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	g := NewServoGroupByIDs(bus, 1)
	if _, err := g.ReadRegister(context.Background(), "torque_enable"); err == nil {
		t.Error("ReadRegister: expected error when no servos have register")
	}
}

func TestServoGroup_WriteRegister(t *testing.T) {
	mock := &transports.MockTransport{}
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond, Protocol: ProtocolSTS})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()
	scs, _ := GetModel("scs0009")
	servos := []*Servo{NewServo(bus, 1, scs), NewServo(bus, 2, scs)}
	g := NewServoGroup(bus, servos...)
	data := map[int][]byte{1: {1}, 2: {0}}
	if err := g.WriteRegister(context.Background(), "torque_enable", data); err != nil {
		t.Fatalf("WriteRegister: %v", err)
	}
	if mock.WriteData[4] != InstSyncWrite {
		t.Errorf("instruction: %02X", mock.WriteData[4])
	}
}

func TestServoGroup_WriteRegister_Empty(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	g := NewServoGroupByIDs(bus, 1)
	if err := g.WriteRegister(context.Background(), "torque_enable", map[int][]byte{}); err != nil {
		t.Fatalf("WriteRegister(empty): %v", err)
	}
	if len(mock.WriteData) != 0 {
		t.Errorf("expected no write, got %d bytes", len(mock.WriteData))
	}
}

func TestServoGroup_WriteRegister_UnknownID(t *testing.T) {
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	g := NewServoGroupByIDs(bus, 1)
	if err := g.WriteRegister(context.Background(), "torque_enable", map[int][]byte{99: {1}}); err == nil {
		t.Error("WriteRegister(unknown ID): expected error")
	}
}

func TestServoGroup_WriteRegister_NoServosHaveRegister(t *testing.T) {
	// sts3215 has nil Registers; map lookup fails for all.
	mock := &transports.MockTransport{}
	bus := newTestBus(t, mock)
	defer bus.Close()
	g := NewServoGroupByIDs(bus, 1)
	// Provide data for an ID in the group, but the model has no register map.
	if err := g.WriteRegister(context.Background(), "torque_enable", map[int][]byte{1: {1}}); err != nil {
		t.Fatalf("WriteRegister: %v", err)
	}
	if len(mock.WriteData) != 0 {
		t.Errorf("expected no write (no register found), got %d bytes", len(mock.WriteData))
	}
}
