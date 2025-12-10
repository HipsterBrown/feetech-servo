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

	// Move all servos to positions
	// positions := []int{2048, 2048, 2048, 2048, 2048}
	positions := []int{2000, 2000, 2000, 2000, 2000}
	// group.SetPositions(ctx, positions)
	group.SetPositionsWithSpeed(ctx, positions, []int{200, 200, 200, 200, 200})

	// Read all positions efficiently using sync read
	currentPos, _ := group.Positions(ctx)
	fmt.Printf("Positions: %v\n", currentPos)

	// Move with speed control
	// group.SetPositionsWithSpeed(ctx,
	// 	[]int{1024, 1024, 1024, 1024, 1024},
	// 	[]int{100, 100, 100, 100, 100},
	// )
}
