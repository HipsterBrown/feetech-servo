package feetech

import (
	"bytes"
	"context"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/transports"
)

// queuedReader returns the given responses one per Read call, then (0, nil)
// (the "no data" signal) once exhausted — mimicking servos that respond to the
// first pings and silence for absent IDs.
func queuedReader(responses [][]byte) func([]byte) (int, error) {
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

// TestBus_Discover_PingsSequentially_NotBroadcast asserts the fixed Discover
// finds every responding servo by pinging IDs individually, never via a single
// broadcast ping (which collides on a multi-servo bus).
func TestBus_Discover_PingsSequentially_NotBroadcast(t *testing.T) {
	mock := &transports.MockTransport{}
	// Servos at IDs 1 and 2 respond (ping status + model read each); rest silent.
	mock.ReadFunc = queuedReader([][]byte{
		mustHex(t, "FF FF 01 02 00 FC"),       // ping resp id 1
		mustHex(t, "FF FF 01 04 00 09 03 EE"), // model 777 id 1
		mustHex(t, "FF FF 02 02 00 FB"),       // ping resp id 2
		mustHex(t, "FF FF 02 04 00 09 03 ED"), // model 777 id 2
	})

	bus, err := NewBus(BusConfig{
		Transport:     mock,
		Timeout:       50 * time.Millisecond,
		MinCommandGap: time.Microsecond,     // don't pad the per-ping round trips
		PingTimeout:   3 * time.Millisecond, // keep the 1..253 sweep fast in tests
	})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	found, err := bus.Discover(context.Background())
	if err != nil {
		t.Fatalf("Discover: %v", err)
	}

	gotIDs := map[int]bool{}
	for _, f := range found {
		gotIDs[f.ID] = true
	}
	if len(found) != 2 || !gotIDs[1] || !gotIDs[2] {
		t.Errorf("Discover found %d servos %v, want IDs {1,2}", len(found), gotIDs)
	}

	// Must have pinged ID 1 individually and NOT broadcast to 0xFE.
	if !bytes.Contains(mock.WriteData, bus.protocol.PingPacket(1)) {
		t.Error("expected an individual ping to ID 1 (sequential discovery)")
	}
	if bytes.Contains(mock.WriteData, bus.protocol.PingPacket(BroadcastID)) {
		t.Error("Discover sent a broadcast ping (0xFE) — must use sequential pings")
	}
}

// TestBus_BroadcastPing_SendsBroadcast keeps the legacy fast-but-lossy path
// available explicitly, STS-only.
func TestBus_BroadcastPing_SendsBroadcast(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.ReadFunc = queuedReader([][]byte{
		mustHex(t, "FF FF 01 02 00 FC"),       // one servo answers the broadcast
		mustHex(t, "FF FF 01 04 00 09 03 EE"), // its model
	})
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 30 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	found, err := bus.BroadcastPing(context.Background())
	if err != nil {
		t.Fatalf("BroadcastPing: %v", err)
	}
	if len(found) != 1 || found[0].ID != 1 {
		t.Errorf("BroadcastPing found %v, want [ID 1]", found)
	}
	if !bytes.Contains(mock.WriteData, bus.protocol.PingPacket(BroadcastID)) {
		t.Error("BroadcastPing must send a broadcast ping to 0xFE")
	}
}

func TestBus_BroadcastPing_SCSUnsupported(t *testing.T) {
	mock := &transports.MockTransport{}
	bus, err := NewBus(BusConfig{Transport: mock, Protocol: ProtocolSCS})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	if _, err := bus.BroadcastPing(context.Background()); err == nil {
		t.Error("BroadcastPing should error on SCS/Protocol 1 (no broadcast ping)")
	}
}

func TestBus_PingTimeout_Default(t *testing.T) {
	bus, err := NewBus(BusConfig{Transport: &transports.MockTransport{}})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()
	if bus.pingTimeout <= 0 {
		t.Errorf("default pingTimeout = %v, want > 0", bus.pingTimeout)
	}

	custom, err := NewBus(BusConfig{Transport: &transports.MockTransport{}, PingTimeout: 7 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer custom.Close()
	if custom.pingTimeout != 7*time.Millisecond {
		t.Errorf("pingTimeout = %v, want 7ms", custom.pingTimeout)
	}
}

// TestBus_ReadHonorsContextDeadline guards the change that makes per-ping
// timeouts effective on a real (blocking) transport: a short context deadline
// must bound the per-read transport timeout, not the much larger bus timeout.
func TestBus_ReadHonorsContextDeadline(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.ReadFunc = func(p []byte) (int, error) { return 0, nil } // never responds
	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 5 * time.Second})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Millisecond)
	defer cancel()
	_, _ = bus.Ping(ctx, 1) // expected to fail (no response)

	if mock.ReadTimeout >= time.Second {
		t.Errorf("transport read timeout = %v; a 30ms ctx deadline should bound it well under 5s", mock.ReadTimeout)
	}
}

func TestScan_IncludesOverloadedServo(t *testing.T) {
	mock := &transports.MockTransport{}
	mock.Script = &transports.Script{
		Steps: []transports.Step{
			// ID 1: ping answers with the overload flag set...
			{Reply: errPacket(1, byte(ErrOverload))},
			// ...and the follow-up model-number read carries 777 (0x0309) with the flag.
			{Reply: readReplyPacket(1, byte(ErrOverload), 0x09, 0x03)},
		},
	}

	bus, err := NewBus(BusConfig{Transport: mock, Timeout: 100 * time.Millisecond})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	found, err := bus.Scan(context.Background(), 1, 1)
	if err != nil {
		t.Fatalf("Scan: %v", err)
	}
	if len(found) != 1 {
		t.Fatalf("an overloaded servo must still be discovered: got %d servos", len(found))
	}
	if found[0].ModelNumber != 777 {
		t.Errorf("ModelNumber: got %d, want 777", found[0].ModelNumber)
	}
	if found[0].Status != ErrOverload {
		t.Errorf("Status: got %v, want ErrOverload", found[0].Status)
	}
}
