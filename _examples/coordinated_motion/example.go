package main

import (
	"context"
	"log"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
	ctx := context.Background()

	bus, err := feetech.NewBus(feetech.BusConfig{
		Port: "/dev/cu.usbmodem5A7A0594021",
	})
	if err != nil {
		log.Fatal(err)
	}
	defer bus.Close()

	group := feetech.NewServoGroupByIDs(bus, 1, 2, 3)
	group.EnableAll(ctx)

	// Use reg write + action for perfectly synchronized motion
	positions := feetech.PositionMap{1: 1000, 2: 2000, 3: 3000}

	// Buffer writes to each servo
	group.RegWritePositions(ctx, positions)

	// Trigger all moves simultaneously
	bus.Action(ctx)
}
