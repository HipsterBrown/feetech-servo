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
