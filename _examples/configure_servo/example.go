package main

import (
	"context"
	"fmt"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
	ctx := context.Background()

	// Connect at factory default baud rate
	bus, _ := feetech.NewBus(feetech.BusConfig{
		Port:     "/dev/cu.usbmodem5A7A0594021",
		BaudRate: 1000000,
	})
	defer bus.Close()

	// Servo at default ID 1
	servo := feetech.NewServo(bus, 1, nil)

	// Change ID (requires torque disabled, which SetID does automatically)
	servo.SetID(ctx, 5)

	// Change baud rate
	servo.SetBaudRate(ctx, 500000)

	fmt.Println("Servo configured with new ID 5 at 500kbps")
}
