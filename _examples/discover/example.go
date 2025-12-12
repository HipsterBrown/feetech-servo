package main

import (
	"context"
	"fmt"
	"log"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
	ctx := context.Background()

	// Create bus - adjust port for your system
	// Common ports: "/dev/ttyUSB0" (Linux), "/dev/cu.usbserial-*" (macOS)
	bus, err := feetech.NewBus(feetech.BusConfig{
		Port:     "/dev/cu.usbmodem5A7A0594021",
		BaudRate: 1000000,
		Protocol: feetech.ProtocolSTS, // Broadcast discovery requires STS protocol
		Timeout:  time.Second,
	})
	if err != nil {
		log.Fatalf("Failed to create bus: %v", err)
	}
	defer bus.Close()

	fmt.Println("Discovering servos using broadcast ping...")
	fmt.Println("This is faster than sequential scanning but only works with STS protocol.")
	fmt.Println()

	// Use broadcast discovery (fast)
	servos, err := bus.Discover(ctx)
	if err != nil {
		log.Fatalf("Discovery failed: %v", err)
	}

	if len(servos) == 0 {
		fmt.Println("No servos found.")
		fmt.Println("\nTroubleshooting:")
		fmt.Println("1. Check that servos are powered on")
		fmt.Println("2. Verify correct serial port")
		fmt.Println("3. Ensure baudrate matches servo settings (default: 1000000)")
		fmt.Println("4. Try sequential scan with Scan() if broadcast doesn't work")
		return
	}

	fmt.Printf("Found %d servo(s):\n\n", len(servos))

	for i, servo := range servos {
		fmt.Printf("%d. Servo ID: %d\n", i+1, servo.ID)
		fmt.Printf("   Model Number: %d\n", servo.ModelNumber)
		if servo.Model != nil {
			fmt.Printf("   Model Name: %s\n", servo.Model.Name)
			fmt.Printf("   Protocol: %d\n", servo.Model.Protocol)
			fmt.Printf("   Resolution: %d steps\n", servo.Model.Resolution)
			fmt.Printf("   Max Position: %d\n", servo.Model.MaxPosition)
		} else {
			fmt.Printf("   Model Name: unknown (model %d not in registry)\n", servo.ModelNumber)
		}
		fmt.Println()
	}

	// Example: Read position from first discovered servo
	if len(servos) > 0 {
		firstServo := feetech.NewServo(bus, servos[0].ID, servos[0].Model)

		fmt.Printf("Reading position from servo %d...\n", servos[0].ID)
		pos, err := firstServo.Position(ctx)
		if err != nil {
			fmt.Printf("Failed to read position: %v\n", err)
		} else {
			fmt.Printf("Current position: %d\n", pos)
		}
	}
}
