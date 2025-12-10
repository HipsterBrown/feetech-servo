package main

import (
	"context"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
	ctx := context.Background()

	bus, _ := feetech.NewBus(feetech.BusConfig{
		Port: "/dev/ttyUSB0",
	})
	defer bus.Close()

	servo := feetech.NewServo(bus, 1, nil)

	// Disable torque to change mode
	servo.Disable(ctx)

	// Set to velocity (wheel) mode
	servo.SetOperatingMode(ctx, feetech.ModeVelocity)

	// Enable and spin
	servo.Enable(ctx)
	servo.SetVelocity(ctx, 500) // Positive = clockwise

	time.Sleep(2 * time.Second)

	servo.SetVelocity(ctx, -500) // Negative = counter-clockwise

	time.Sleep(2 * time.Second)

	servo.SetVelocity(ctx, 0) // Stop
}
