package main

import (
	"context"
	"fmt"
	"log"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()

	bus, err := feetech.NewBus(feetech.BusConfig{
		Port: "/dev/cu.usbmodem5A7A0594021",
	})
	defer bus.Close()
	if err != nil {
		log.Fatal(err)
	}

	// Scan for servos
	found, err := bus.Scan(ctx, 0, 20)
	if err != nil {
		log.Fatal(err)
	}

	for _, servo := range found {
		name := "unknown"
		if servo.Model != nil {
			name = servo.Model.Name
		}
		fmt.Printf("Found servo ID %d: %s (model number %d)\n",
			servo.ID, name, servo.ModelNumber)
	}
}
