package feetech

import (
	"context"
	"sync"
	"testing"
	"time"

	"github.com/hipsterbrown/feetech-servo/transports"
)

// TestBus_ConcurrentReads exercises Bus serialization under -race. Eight goroutines
// race to call Ping concurrently against a deterministic mock; the bus must serialize
// them through its mutex without data races.
func TestBus_ConcurrentReads(t *testing.T) {
	mock := &transports.MockTransport{
		ReadFunc: func(p []byte) (int, error) {
			// Always reply with a ping ack. We don't care whether subsequent stages
			// (model-number read) succeed — the goal is to exercise serialization,
			// not protocol correctness.
			return copy(p, []byte{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC}), nil
		},
	}

	bus, err := NewBus(BusConfig{
		Transport: mock,
		Timeout:   200 * time.Millisecond,
	})
	if err != nil {
		t.Fatalf("NewBus: %v", err)
	}
	defer bus.Close()

	const goroutines = 8
	const iterations = 5

	var wg sync.WaitGroup
	wg.Add(goroutines)
	ctx := context.Background()

	for i := 0; i < goroutines; i++ {
		go func() {
			defer wg.Done()
			for j := 0; j < iterations; j++ {
				// Errors are acceptable as long as nothing data-races. Mock will
				// likely error on the second-stage model read, which is expected
				// since we don't script multi-step replies here.
				_, _ = bus.Ping(ctx, 1)
			}
		}()
	}
	wg.Wait()
}
