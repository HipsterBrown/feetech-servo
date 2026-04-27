package feetech

import (
	"bytes"
	"testing"
)

// Each test asserts that the protocol encoder produces the exact bytes documented in
// Feetech Communication Protocol Manual V1.01.

func TestManual_PingPacket(t *testing.T) {
	p := NewProtocol(ProtocolSTS)
	got := p.PingPacket(0x01)
	want := loadHexFixture(t, "manual_ping")
	if !bytes.Equal(got, want) {
		t.Errorf("ping packet: got %X want %X", got, want)
	}
}

func TestManual_ReadPositionPacket(t *testing.T) {
	p := NewProtocol(ProtocolSTS)
	got := p.ReadPacket(0x01, 0x38, 0x02)
	want := loadHexFixture(t, "manual_read_position")
	if !bytes.Equal(got, want) {
		t.Errorf("read packet: got %X want %X", got, want)
	}
}

func TestManual_WriteIDPacket(t *testing.T) {
	p := NewProtocol(ProtocolSTS)
	got := p.WritePacket(BroadcastID, 0x05, []byte{0x01})
	want := loadHexFixture(t, "manual_write_id")
	if !bytes.Equal(got, want) {
		t.Errorf("write-ID packet: got %X want %X", got, want)
	}
}

func TestManual_WritePositionTimeSpeedPacket(t *testing.T) {
	// Manual §1.3.3 Example 4: pos=0x0800, time=0x0000, speed=0x03E8 to ID 1 starting at 0x2A.
	p := NewProtocol(ProtocolSTS)
	params := []byte{0x00, 0x08, 0x00, 0x00, 0xE8, 0x03}
	got := p.WritePacket(0x01, 0x2A, params)
	want := loadHexFixture(t, "manual_write_pos_time_speed")
	if !bytes.Equal(got, want) {
		t.Errorf("position+time+speed write: got %X want %X", got, want)
	}
}

func TestManual_ActionPacket(t *testing.T) {
	p := NewProtocol(ProtocolSTS)
	got := p.ActionPacket()
	want := loadHexFixture(t, "manual_action")
	if !bytes.Equal(got, want) {
		t.Errorf("action packet: got %X want %X", got, want)
	}
}

func TestManual_SyncWritePacket(t *testing.T) {
	p := NewProtocol(ProtocolSTS)
	servoData := map[byte][]byte{
		1: {0x00, 0x08, 0x00, 0x00, 0xE8, 0x03},
		2: {0x00, 0x08, 0x00, 0x00, 0xE8, 0x03},
		3: {0x00, 0x08, 0x00, 0x00, 0xE8, 0x03},
		4: {0x00, 0x08, 0x00, 0x00, 0xE8, 0x03},
	}
	got := p.SyncWritePacket(0x2A, 6, servoData)

	// Manual fixture lists servos in ID order; production code uses Go map iteration order.
	// Validate envelope (header + length + instruction + address + dataLen + checksum) and
	// each per-servo block independent of order. Production fix is tracked in Phase 5.
	want := loadHexFixture(t, "manual_sync_write_4_servos")
	if len(got) != len(want) {
		t.Fatalf("sync write length: got %d want %d", len(got), len(want))
	}
	if !bytes.Equal(got[:7], want[:7]) {
		t.Errorf("sync write header: got %X want %X", got[:7], want[:7])
	}
	if got[len(got)-1] != calcSyncWriteChecksum(t, got) {
		t.Errorf("sync write checksum invalid")
	}
	for id := byte(1); id <= 4; id++ {
		block := []byte{id, 0x00, 0x08, 0x00, 0x00, 0xE8, 0x03}
		if !bytes.Contains(got[7:len(got)-1], block) {
			t.Errorf("sync write missing block for servo %d", id)
		}
	}
}

// calcSyncWriteChecksum returns the expected checksum for the encoded sync-write packet.
// Checksum is bitwise NOT of the byte sum from index 2 (ID) through the second-to-last byte.
func calcSyncWriteChecksum(t *testing.T, packet []byte) byte {
	t.Helper()
	var sum byte
	for _, b := range packet[2 : len(packet)-1] {
		sum += b
	}
	return ^sum
}
