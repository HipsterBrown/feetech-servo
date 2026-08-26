package feetech

import (
	"context"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/transports"
)

func TestServoGroup_SetPositions_EmptyMapIsNoOp(t *testing.T) {
	mock := &transports.MockTransport{}
	bus, _ := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	defer bus.Close()

	g := NewServoGroupByIDs(bus, 1, 2)
	if err := g.SetPositions(context.Background(), PositionMap{}); err != nil {
		t.Errorf("empty map should be no-op: %v", err)
	}
	if len(mock.WriteData) != 0 {
		t.Errorf("expected zero writes, got %X", mock.WriteData)
	}
}

func TestServoGroup_SetPositions_RejectsUnknownID(t *testing.T) {
	mock := &transports.MockTransport{}
	bus, _ := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	defer bus.Close()

	g := NewServoGroupByIDs(bus, 1, 2)
	err := g.SetPositions(context.Background(), PositionMap{99: 2048})
	if err == nil {
		t.Fatal("expected error for unknown ID")
	}
}

func TestServoGroup_Positions_DecodesSyncReadResponses(t *testing.T) {
	// Reply: ID 1 -> position 2048, ID 2 -> position 1024 (little-endian, STS).
	mock := &transports.MockTransport{
		ReadData: loadHexFixture(t, "sync_read_two_responses"),
	}
	bus, _ := NewBus(BusConfig{
		Transport: mock,
		Protocol:  ProtocolSTS,
		Timeout:   100 * time.Millisecond,
	})
	defer bus.Close()

	g := NewServoGroupByIDs(bus, 1, 2)
	positions, err := g.Positions(context.Background())
	if err != nil {
		t.Fatalf("Positions: %v", err)
	}
	if positions[1] != 2048 {
		t.Errorf("servo 1: got %d want 2048", positions[1])
	}
	if positions[2] != 1024 {
		t.Errorf("servo 2: got %d want 1024", positions[2])
	}
}

// TestServoGroup_Positions_ConditionFlagKeepsAllPositions verifies that a
// condition flag on one servo doesn't blank out the whole group read: every
// servo's position is still returned, alongside an error ConditionStatus
// recognizes.
func TestServoGroup_Positions_ConditionFlagKeepsAllPositions(t *testing.T) {
	mock := &transports.MockTransport{
		ReadData: append(
			readReplyPacket(1, byte(ErrOverload), 0x00, 0x08), // servo 1: flagged, position 2048
			readReplyPacket(2, 0x00, 0x00, 0x04)...,           // servo 2: clean, position 1024
		),
	}
	bus, _ := NewBus(BusConfig{
		Transport: mock,
		Protocol:  ProtocolSTS,
		Timeout:   100 * time.Millisecond,
	})
	defer bus.Close()

	g := NewServoGroupByIDs(bus, 1, 2)
	positions, err := g.Positions(context.Background())
	if positions[1] != 2048 {
		t.Errorf("servo 1: got %d want 2048", positions[1])
	}
	if positions[2] != 1024 {
		t.Errorf("servo 2: got %d want 1024", positions[2])
	}
	if flags, ok := ConditionStatus(err); !ok || flags != ErrOverload {
		t.Errorf("ConditionStatus(err) = (%v, %v), want (ErrOverload, true)", flags, ok)
	}
}

// TestServoGroup_Positions_RequestFlagReturnsNil verifies that a
// request-rejection flag still discards the whole result: nil map, and
// ConditionStatus reports ok == false so callers can't mistake it for a
// motor-condition report.
func TestServoGroup_Positions_RequestFlagReturnsNil(t *testing.T) {
	mock := &transports.MockTransport{
		ReadData: append(
			readReplyPacket(1, byte(ErrChecksum), 0x00, 0x08),
			readReplyPacket(2, 0x00, 0x00, 0x04)...,
		),
	}
	bus, _ := NewBus(BusConfig{
		Transport: mock,
		Protocol:  ProtocolSTS,
		Timeout:   100 * time.Millisecond,
	})
	defer bus.Close()

	g := NewServoGroupByIDs(bus, 1, 2)
	positions, err := g.Positions(context.Background())
	if positions != nil {
		t.Errorf("expected nil PositionMap, got %v", positions)
	}
	if _, ok := ConditionStatus(err); ok {
		t.Errorf("ConditionStatus ok = true, want false for request-rejection flag")
	}
}

// TestServoGroup_Positions_EmptyGroupReturnsEmptyMap verifies that a group
// with no servos returns an empty non-nil PositionMap rather than nil, so a
// caller that writes into the returned map doesn't panic.
func TestServoGroup_Positions_EmptyGroupReturnsEmptyMap(t *testing.T) {
	mock := &transports.MockTransport{}
	bus, _ := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	defer bus.Close()

	g := NewServoGroupByIDs(bus)
	positions, err := g.Positions(context.Background())
	if err != nil {
		t.Fatalf("Positions: %v", err)
	}
	if positions == nil {
		t.Fatal("expected non-nil empty PositionMap, got nil")
	}
	if len(positions) != 0 {
		t.Errorf("expected empty map, got %v", positions)
	}
	positions[1] = 100 // must not panic on a nil map
}

func TestServoGroup_EnableAll_WritesSyncWritePacket(t *testing.T) {
	mock := &transports.MockTransport{}
	bus, _ := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	defer bus.Close()

	g := NewServoGroupByIDs(bus, 1, 2)
	if err := g.EnableAll(context.Background()); err != nil {
		t.Fatalf("EnableAll: %v", err)
	}

	// Sync write to broadcast ID, instruction 0x83, address = RegTorqueEnable.Address (40), dataLen = 1.
	if len(mock.WriteData) < 8 {
		t.Fatalf("write too short: %X", mock.WriteData)
	}
	if mock.WriteData[2] != BroadcastID {
		t.Errorf("not broadcast: %02X", mock.WriteData[2])
	}
	if mock.WriteData[4] != InstSyncWrite {
		t.Errorf("wrong instruction: %02X", mock.WriteData[4])
	}
	if mock.WriteData[5] != RegTorqueEnable.Address {
		t.Errorf("wrong address: %02X", mock.WriteData[5])
	}
	if mock.WriteData[6] != 1 {
		t.Errorf("wrong dataLen: %02X", mock.WriteData[6])
	}
}

func TestServoGroup_ServoByID(t *testing.T) {
	mock := &transports.MockTransport{}
	bus, _ := NewBus(BusConfig{Transport: mock})
	defer bus.Close()

	g := NewServoGroupByIDs(bus, 1, 2, 3)
	if g.ServoByID(2) == nil {
		t.Error("ServoByID(2) returned nil")
	}
	if g.ServoByID(99) != nil {
		t.Error("ServoByID(99) should be nil for unknown ID")
	}
}
