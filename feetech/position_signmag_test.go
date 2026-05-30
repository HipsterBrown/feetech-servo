package feetech

import (
	"bytes"
	"context"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/transports"
)

// statusOK is a valid write-acknowledgement status packet for servo ID 1.
const statusOKID1 = "FF FF 01 02 00 FC"

func writeMock(t *testing.T) (*transports.MockTransport, *Bus) {
	t.Helper()
	mock := &transports.MockTransport{ReadData: mustHex(t, statusOKID1)}
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	return mock, bus
}

func TestRegisters_PositionRegistersAreSignMagnitude(t *testing.T) {
	if RegGoalPosition.SignBit != 15 {
		t.Errorf("RegGoalPosition.SignBit = %d, want 15", RegGoalPosition.SignBit)
	}
	if RegPresentPosition.SignBit != 15 {
		t.Errorf("RegPresentPosition.SignBit = %d, want 15", RegPresentPosition.SignBit)
	}
}

func TestServo_SetPosition_NegativeUsesSignMagnitude(t *testing.T) {
	mock, bus := writeMock(t)
	defer bus.Close()

	if err := NewServo(bus, 1, nil).SetPosition(context.Background(), -100); err != nil {
		t.Fatalf("SetPosition(-100): %v", err)
	}

	// -100 -> sign-magnitude bit 15 -> 0x8064 -> little-endian [0x64, 0x80].
	// (NOT two's-complement 0xFF9C / [0x9C, 0xFF].)
	want := bus.protocol.WritePacket(1, RegGoalPosition.Address, []byte{0x64, 0x80})
	if !bytes.Equal(mock.WriteData, want) {
		t.Errorf("packet:\n  got:  %X\n  want: %X", mock.WriteData, want)
	}
}

func TestServo_SetPosition_PositiveUnchanged(t *testing.T) {
	mock, bus := writeMock(t)
	defer bus.Close()

	if err := NewServo(bus, 1, nil).SetPosition(context.Background(), 2048); err != nil {
		t.Fatalf("SetPosition(2048): %v", err)
	}
	if got := mock.WriteData[6:8]; !bytes.Equal(got, []byte{0x00, 0x08}) {
		t.Errorf("position data: got %X, want [00 08]", got)
	}
}

func TestServo_SetPosition_RejectsUnencodable(t *testing.T) {
	for _, pos := range []int{32768, -32768, 70000, -70000} {
		mock, bus := writeMock(t)
		err := NewServo(bus, 1, nil).SetPosition(context.Background(), pos)
		if err == nil {
			t.Errorf("SetPosition(%d): expected out-of-range error", pos)
		}
		if len(mock.WriteData) != 0 {
			t.Errorf("SetPosition(%d): wrote %X, want no write on rejection", pos, mock.WriteData)
		}
		bus.Close()
	}
}

func TestServo_Position_DecodesNegative(t *testing.T) {
	// present_position raw 0x8064 -> sign-magnitude -100.
	mock := &transports.MockTransport{ReadData: mustHex(t, "FF FF 01 04 00 64 80 16")}
	bus, _ := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	defer bus.Close()

	pos, err := NewServo(bus, 1, nil).Position(context.Background())
	if err != nil {
		t.Fatalf("Position: %v", err)
	}
	if pos != -100 {
		t.Errorf("Position = %d, want -100", pos)
	}
}

func TestServo_Position_PositiveUnchanged(t *testing.T) {
	// present_position raw 0x07CF (1999), bit 15 clear.
	mock := &transports.MockTransport{ReadData: mustHex(t, "FF FF 01 04 00 CF 07 24")}
	bus, _ := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	defer bus.Close()

	pos, err := NewServo(bus, 1, nil).Position(context.Background())
	if err != nil {
		t.Fatalf("Position: %v", err)
	}
	if pos != 1999 {
		t.Errorf("Position = %d, want 1999", pos)
	}
}

func TestServo_SetPositionWithSpeed_NegativePosition(t *testing.T) {
	mock, bus := writeMock(t)
	defer bus.Close()

	if err := NewServo(bus, 1, nil).SetPositionWithSpeed(context.Background(), -100, 500); err != nil {
		t.Fatalf("SetPositionWithSpeed: %v", err)
	}
	// Position is the first 2 bytes of the 6-byte payload (data starts at index 6).
	if got := mock.WriteData[6:8]; !bytes.Equal(got, []byte{0x64, 0x80}) {
		t.Errorf("position component: got %X, want [64 80]", got)
	}
}

func TestServoGroup_SetPositions_NegativeUsesSignMagnitude(t *testing.T) {
	mock := &transports.MockTransport{}
	bus, _ := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	defer bus.Close()

	g := NewServoGroupByIDs(bus, 1)
	if err := g.SetPositions(context.Background(), PositionMap{1: -100}); err != nil {
		t.Fatalf("SetPositions: %v", err)
	}
	if !bytes.Contains(mock.WriteData, []byte{0x64, 0x80}) {
		t.Errorf("sync write %X missing sign-magnitude bytes [64 80]", mock.WriteData)
	}
	if bytes.Contains(mock.WriteData, []byte{0x9C, 0xFF}) {
		t.Error("sync write used two's-complement encoding [9C FF] instead of sign-magnitude")
	}
}

func TestServoGroup_Positions_DecodesNegative(t *testing.T) {
	mock := &transports.MockTransport{ReadData: mustHex(t, "FF FF 01 04 00 64 80 16")}
	bus, _ := NewBus(BusConfig{Transport: mock, Protocol: ProtocolSTS, Timeout: 100 * time.Millisecond})
	defer bus.Close()

	g := NewServoGroupByIDs(bus, 1)
	positions, err := g.Positions(context.Background())
	if err != nil {
		t.Fatalf("Positions: %v", err)
	}
	if positions[1] != -100 {
		t.Errorf("positions[1] = %d, want -100", positions[1])
	}
}

func TestPosition_SignMagnitude_RoundTrip(t *testing.T) {
	proto := NewProtocol(ProtocolSTS)
	for _, v := range []int{-4095, -2048, -1, 0, 1, 2048, 4095, 32767, -32767} {
		enc := encodeSignMagnitude(v, RegGoalPosition.SignBit)
		word := proto.EncodeWord(uint16(enc))
		dec := decodeSignMagnitude(int(proto.DecodeWord(word)), RegPresentPosition.SignBit)
		if dec != v {
			t.Errorf("round-trip %d -> 0x%04X -> %d", v, proto.DecodeWord(word), dec)
		}
	}
}
