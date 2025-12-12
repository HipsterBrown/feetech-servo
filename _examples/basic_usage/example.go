package main

import (
	"context"
	"fmt"
	"log"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
	ctx := context.Background()

	// Open the servo bus
	bus, err := feetech.NewBus(feetech.BusConfig{
		Port:     "/dev/cu.usbmodem5A7A0594021",
		BaudRate: 1000000,
	})
	if err != nil {
		log.Fatal(err)
	}
	defer bus.Close()

	// Create a servo instance
	servo := feetech.NewServo(bus, 1, nil) // nil = default STS3215

	// Ping and detect model
	if err := servo.DetectModel(ctx); err != nil {
		log.Fatal(err)
	}
	fmt.Printf("Connected to: %s\n", servo.Model().Name)

	// Read current position
	pos, _ := servo.Position(ctx)
	fmt.Printf("Current Position: %d\n", pos)

	// Enable torque and move
	servo.Enable(ctx)
	servo.SetPosition(ctx, 2048) // Center position

	// Read current position again
	pos, _ = servo.Position(ctx)
	fmt.Printf("New Position: %d\n", pos)
}
