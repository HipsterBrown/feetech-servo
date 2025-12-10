package main

import (
	"context"
	"fmt"
	"log"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
	// For testing without hardware
	mock := &feetech.MockTransport{
		// Pre-load a ping response
		ReadData: []byte{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC},
	}

	bus, _ := feetech.NewBus(feetech.BusConfig{
		Transport: mock,
	})
	defer bus.Close()

	ctx := context.Background()

	// This will use the mock data
	_, err := bus.Ping(ctx, 1)
	if err != nil {
		log.Printf("Ping failed: %v", err)
	}

	// Check what was written
	fmt.Printf("Sent: %X\n", mock.WriteData)
}
