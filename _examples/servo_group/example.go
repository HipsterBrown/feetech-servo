package main

import (
	"context"
	"fmt"
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

	// Create a group of servos
	group := feetech.NewServoGroupByIDs(bus, 1, 2, 3, 4, 5)

	// Enable all servos
	group.EnableAll(ctx)

	// Move servos to positions using map
	positions := feetech.PositionMap{1: 2000, 2: 2000, 3: 2000, 4: 2000, 5: 2000}
	speeds := feetech.PositionMap{1: 200, 2: 200, 3: 200, 4: 200, 5: 200}
	group.SetPositionsWithSpeed(ctx, positions, speeds)

	// Read all positions efficiently using sync read
	currentPos, _ := group.Positions(ctx)
	fmt.Printf("Positions: %v\n", currentPos)
}
