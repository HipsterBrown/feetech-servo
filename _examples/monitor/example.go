package main

import (
	"context"
	"fmt"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
	ctx := context.Background()

	bus, err := feetech.NewBus(feetech.BusConfig{
		Port: "/dev/cu.usbmodem5A7A0594021",
	})
	if err != nil {
		fmt.Printf("Failed to create bus: %v\n", err)
		return
	}
	defer bus.Close()

	fmt.Printf("Created bus successfully\n")

	servo := feetech.NewServo(bus, 1, nil)

	// Read various status values
	voltage, err := servo.Voltage(ctx)
	if err != nil {
		fmt.Printf("Failed to read voltage: %v\n", err)
		return
	}

	temp, err := servo.Temperature(ctx)
	if err != nil {
		fmt.Printf("Failed to read temperature: %v\n", err)
		return
	}

	load, err := servo.Load(ctx)
	if err != nil {
		fmt.Printf("Failed to read load: %v\n", err)
		return
	}

	fmt.Printf("Voltage: %.1fV\n", float64(voltage)/10.0)
	fmt.Printf("Temperature: %d°C\n", temp)
	fmt.Printf("Load: %d\n", load)
}
