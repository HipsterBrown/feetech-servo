package feetech

import (
	"bytes"
	"context"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/transports"
)

// SetGoal writes 7 bytes starting at the acceleration register (addr 41):
// [acc, pos_lo, pos_hi, time_lo, time_hi, speed_lo, speed_hi], matching the
// Python SDK's WritePosEx field order.
func TestServo_SetGoal_WritesSevenBytesFromAccel_PythonParity(t *testing.T) {
	mock, bus := writeMock(t)
	defer bus.Close()

	err := NewServo(bus, 1, nil).SetGoal(context.Background(), GoalRequest{
		Position: 2048, Speed: 1000, Acc: 50,
	})
	if err != nil {
		t.Fatalf("SetGoal: %v", err)
	}

	// acc=50(0x32), pos=2048(0x0800->00 08), time=0, speed=1000(0x03E8->E8 03).
	payload := []byte{0x32, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03}
	want := bus.protocol.WritePacket(1, RegAcceleration.Address, payload)
	if !bytes.Equal(mock.WriteData, want) {
		t.Errorf("packet:\n  got:  %X\n  want: %X", mock.WriteData, want)
	}
}

func TestServo_SetGoal_WithTime(t *testing.T) {
	mock, bus := writeMock(t)
	defer bus.Close()

	if err := NewServo(bus, 1, nil).SetGoal(context.Background(), GoalRequest{
		Position: 2048, Time: 1000, Acc: 10,
	}); err != nil {
		t.Fatalf("SetGoal: %v", err)
	}
	// acc=10, pos=2048, time=1000(E8 03), speed=0.
	payload := []byte{0x0A, 0x00, 0x08, 0xE8, 0x03, 0x00, 0x00}
	want := bus.protocol.WritePacket(1, RegAcceleration.Address, payload)
	if !bytes.Equal(mock.WriteData, want) {
		t.Errorf("packet:\n  got:  %X\n  want: %X", mock.WriteData, want)
	}
}

func TestServo_SetGoal_NegativePositionSignMagnitude(t *testing.T) {
	mock, bus := writeMock(t)
	defer bus.Close()

	if err := NewServo(bus, 1, nil).SetGoal(context.Background(), GoalRequest{
		Position: -100,
	}); err != nil {
		t.Fatalf("SetGoal: %v", err)
	}
	// pos -100 -> sign-magnitude 0x8064 -> [64 80] at data[1:3].
	if got := mock.WriteData[7:9]; !bytes.Equal(got, []byte{0x64, 0x80}) {
		t.Errorf("position bytes: got %X, want [64 80]", got)
	}
}

func TestServo_SetGoal_RejectsOutOfRange(t *testing.T) {
	cases := map[string]GoalRequest{
		"acc>255":      {Acc: 256},
		"acc<0":        {Acc: -1},
		"speed>65535":  {Speed: 70000},
		"time>65535":   {Time: 70000},
		"bad position": {Position: 40000},
	}
	for name, g := range cases {
		mock, bus := writeMock(t)
		if err := NewServo(bus, 1, nil).SetGoal(context.Background(), g); err == nil {
			t.Errorf("%s: expected error", name)
		} else if len(mock.WriteData) != 0 {
			t.Errorf("%s: wrote %X, want no write on rejection", name, mock.WriteData)
		}
		bus.Close()
	}
}

func TestServoGroup_SetGoals_WritesSyncWriteFromAccel(t *testing.T) {
	mock := &transports.MockTransport{}
	bus, _ := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	defer bus.Close()

	g := NewServoGroupByIDs(bus, 1)
	if err := g.SetGoals(context.Background(), map[int]GoalRequest{
		1: {Position: 2048, Speed: 1000, Acc: 50},
	}); err != nil {
		t.Fatalf("SetGoals: %v", err)
	}

	if len(mock.WriteData) < 8 {
		t.Fatalf("write too short: %X", mock.WriteData)
	}
	if mock.WriteData[2] != BroadcastID {
		t.Errorf("not broadcast: %02X", mock.WriteData[2])
	}
	if mock.WriteData[4] != InstSyncWrite {
		t.Errorf("instruction: %02X want sync write", mock.WriteData[4])
	}
	if mock.WriteData[5] != RegAcceleration.Address {
		t.Errorf("address: %02X want %02X (accel)", mock.WriteData[5], RegAcceleration.Address)
	}
	if mock.WriteData[6] != 7 {
		t.Errorf("dataLen: %d want 7", mock.WriteData[6])
	}
	if !bytes.Contains(mock.WriteData, []byte{0x32, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03}) {
		t.Errorf("missing 7-byte goal payload in %X", mock.WriteData)
	}
}

func TestServoGroup_SetGoals_RejectsUnknownID(t *testing.T) {
	mock := &transports.MockTransport{}
	bus, _ := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	defer bus.Close()

	g := NewServoGroupByIDs(bus, 1)
	if err := g.SetGoals(context.Background(), map[int]GoalRequest{99: {Position: 100}}); err == nil {
		t.Error("expected error for unknown ID")
	}
}
