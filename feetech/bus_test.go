package feetech

import (
	"bytes"
	"context"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/transports"
)

func TestBus_Ping(t *testing.T) {
	// Set up mock with ping response + model number read response
	mock := &transports.MockTransport{}
	readIdx := 0
	responses := [][]byte{
		{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC},             // Ping response
		{0xFF, 0xFF, 0x01, 0x04, 0x00, 0x09, 0x03, 0xEE}, // Model number 777 (0x0309)
	}
	mock.ReadFunc = func(p []byte) (int, error) {
		if readIdx >= len(responses) {
			return 0, nil
		}
		n := copy(p, responses[readIdx])
		readIdx++
		return n, nil
	}

	bus, err := NewBus(BusConfig{
		Transport: mock,
		Timeout:   100 * time.Millisecond,
	})
	if err != nil {
		t.Fatalf("NewBus failed: %v", err)
	}
	defer bus.Close()

	ctx := context.Background()
	modelNum, err := bus.Ping(ctx, 1)
	if err != nil {
		t.Fatalf("Ping failed: %v", err)
	}

	if modelNum != 777 {
		t.Errorf("model number: got %d, want 777", modelNum)
	}

	// Verify ping packet was sent
	// Expected: FF FF 01 02 01 FB
	if len(mock.WriteData) < 6 {
		t.Fatalf("no packet written")
	}
	if mock.WriteData[4] != InstPing {
		t.Errorf("wrong instruction: got %02X, want %02X", mock.WriteData[4], InstPing)
	}
}

func TestBus_ReadRegister(t *testing.T) {
	// Mock response for reading 2 bytes
	mock := &transports.MockTransport{
		ReadData: []byte{0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2}, // Position 2048
	}

	bus, _ := NewBus(BusConfig{
		Transport: mock,
		Timeout:   100 * time.Millisecond,
	})
	defer bus.Close()

	ctx := context.Background()
	data, err := bus.ReadRegister(ctx, 1, RegPresentPosition.Address, 2)
	if err != nil {
		t.Fatalf("ReadRegister failed: %v", err)
	}

	if len(data) != 2 {
		t.Fatalf("data length: got %d, want 2", len(data))
	}

	position := bus.Protocol().DecodeWord(data)
	if position != 2048 {
		t.Errorf("position: got %d, want 2048", position)
	}
}

func TestBus_WriteRegister(t *testing.T) {
	mock := &transports.MockTransport{
		ReadData: []byte{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC}, // Ack response
	}

	bus, _ := NewBus(BusConfig{
		Transport: mock,
		Timeout:   100 * time.Millisecond,
	})
	defer bus.Close()

	ctx := context.Background()
	data := bus.Protocol().EncodeWord(2048)
	err := bus.WriteRegister(ctx, 1, RegGoalPosition.Address, data)
	if err != nil {
		t.Fatalf("WriteRegister failed: %v", err)
	}

	// Verify write packet structure
	if mock.WriteData[4] != InstWrite {
		t.Errorf("wrong instruction: got %02X, want %02X", mock.WriteData[4], InstWrite)
	}
	if mock.WriteData[5] != RegGoalPosition.Address {
		t.Errorf("wrong address: got %02X, want %02X", mock.WriteData[5], RegGoalPosition.Address)
	}
}

func TestBus_SyncWrite(t *testing.T) {
	mock := &transports.MockTransport{}

	bus, _ := NewBus(BusConfig{
		Transport: mock,
		Timeout:   100 * time.Millisecond,
	})
	defer bus.Close()

	ctx := context.Background()
	servoData := map[int][]byte{
		1: {0x00, 0x08}, // Position 2048
		2: {0x00, 0x08},
	}

	err := bus.SyncWrite(ctx, RegGoalPosition.Address, 2, servoData)
	if err != nil {
		t.Fatalf("SyncWrite failed: %v", err)
	}

	// Verify sync write packet
	if mock.WriteData[2] != BroadcastID {
		t.Errorf("not broadcast: got %02X, want %02X", mock.WriteData[2], BroadcastID)
	}
	if mock.WriteData[4] != InstSyncWrite {
		t.Errorf("wrong instruction: got %02X, want %02X", mock.WriteData[4], InstSyncWrite)
	}
}

func TestBus_SyncRead(t *testing.T) {
	// Mock two servo responses
	mock := &transports.MockTransport{
		ReadData: []byte{
			0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2, // ID 1, position 2048
			0xFF, 0xFF, 0x02, 0x04, 0x00, 0x00, 0x04, 0xF5, // ID 2, position 1024
		},
	}

	bus, _ := NewBus(BusConfig{
		Transport: mock,
		Protocol:  ProtocolSTS,
		Timeout:   100 * time.Millisecond,
	})
	defer bus.Close()

	ctx := context.Background()
	data, err := bus.SyncRead(ctx, RegPresentPosition.Address, 2, []int{1, 2})
	if err != nil {
		t.Fatalf("SyncRead failed: %v", err)
	}

	if len(data) != 2 {
		t.Fatalf("got %d results, want 2", len(data))
	}

	proto := bus.Protocol()
	pos1 := proto.DecodeWord(data[1])
	pos2 := proto.DecodeWord(data[2])

	if pos1 != 2048 {
		t.Errorf("servo 1 position: got %d, want 2048", pos1)
	}
	if pos2 != 1024 {
		t.Errorf("servo 2 position: got %d, want 1024", pos2)
	}
}

func TestBus_SyncRead_SCSUnsupported(t *testing.T) {
	mock := &transports.MockTransport{}

	bus, _ := NewBus(BusConfig{
		Transport: mock,
		Protocol:  ProtocolSCS, // SCS doesn't support sync read
	})
	defer bus.Close()

	ctx := context.Background()
	_, err := bus.SyncRead(ctx, RegPresentPosition.Address, 2, []int{1, 2})
	if err == nil {
		t.Error("expected error for SCS sync read")
	}
}

func TestBus_InvalidID(t *testing.T) {
	mock := &transports.MockTransport{}
	bus, _ := NewBus(BusConfig{Transport: mock})
	defer bus.Close()

	ctx := context.Background()

	// Test invalid IDs
	_, err := bus.Ping(ctx, -1)
	if err == nil {
		t.Error("expected error for negative ID")
	}

	_, err = bus.Ping(ctx, 255)
	if err == nil {
		t.Error("expected error for ID > MaxServoID")
	}
}

func TestBus_Close(t *testing.T) {
	mock := &transports.MockTransport{}
	bus, _ := NewBus(BusConfig{Transport: mock})

	err := bus.Close()
	if err != nil {
		t.Errorf("Close failed: %v", err)
	}
	if !mock.Closed {
		t.Error("transport not closed")
	}

	// Closing again should be safe
	err = bus.Close()
	if err != nil {
		t.Errorf("second Close failed: %v", err)
	}
}

func TestBus_ClosedOperations(t *testing.T) {
	mock := &transports.MockTransport{}
	bus, _ := NewBus(BusConfig{Transport: mock})
	bus.Close()

	ctx := context.Background()

	_, err := bus.Ping(ctx, 1)
	if err != ErrBusClosed {
		t.Errorf("expected ErrBusClosed, got %v", err)
	}
}

func TestServo_Position(t *testing.T) {
	mock := &transports.MockTransport{
		ReadData: []byte{0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2},
	}

	bus, _ := NewBus(BusConfig{
		Transport: mock,
		Timeout:   100 * time.Millisecond,
	})
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	ctx := context.Background()

	pos, err := servo.Position(ctx)
	if err != nil {
		t.Fatalf("Position failed: %v", err)
	}

	if pos != 2048 {
		t.Errorf("position: got %d, want 2048", pos)
	}
}

func TestServo_SetPosition(t *testing.T) {
	mock := &transports.MockTransport{
		ReadData: []byte{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC},
	}

	bus, _ := NewBus(BusConfig{
		Transport: mock,
		Timeout:   100 * time.Millisecond,
	})
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	ctx := context.Background()

	err := servo.SetPosition(ctx, 2048)
	if err != nil {
		t.Fatalf("SetPosition failed: %v", err)
	}

	// Verify position data in packet
	// Position 2048 = 0x0800, little-endian = [0x00, 0x08]
	posData := mock.WriteData[6:8]
	if !bytes.Equal(posData, []byte{0x00, 0x08}) {
		t.Errorf("position data: got %X, want [00 08]", posData)
	}
}

func TestServo_TorqueEnable(t *testing.T) {
	mock := &transports.MockTransport{
		ReadData: []byte{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC},
	}

	bus, _ := NewBus(BusConfig{
		Transport: mock,
		Timeout:   100 * time.Millisecond,
	})
	defer bus.Close()

	servo := NewServo(bus, 1, nil)
	ctx := context.Background()

	err := servo.Enable(ctx)
	if err != nil {
		t.Fatalf("Enable failed: %v", err)
	}

	// Verify torque enable address and value
	if mock.WriteData[5] != RegTorqueEnable.Address {
		t.Errorf("wrong address: got %02X, want %02X", mock.WriteData[5], RegTorqueEnable.Address)
	}
	if mock.WriteData[6] != 1 {
		t.Errorf("wrong value: got %d, want 1", mock.WriteData[6])
	}
}

func TestBus_ContextCancellation(t *testing.T) {
	// Simulate slow transport
	mock := &transports.MockTransport{
		ReadFunc: func(p []byte) (int, error) {
			time.Sleep(500 * time.Millisecond)
			return 0, nil
		},
	}

	bus, _ := NewBus(BusConfig{
		Transport: mock,
		Timeout:   time.Second,
	})
	defer bus.Close()

	ctx, cancel := context.WithTimeout(context.Background(), 50*time.Millisecond)
	defer cancel()

	_, err := bus.Ping(ctx, 1)
	if err == nil {
		t.Error("expected context cancellation error")
	}
}
