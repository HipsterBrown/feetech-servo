package main

import (
	"context"
	"fmt"
	"log"
	"os"
	"time"

	"github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
	ctx := context.Background()

	// Port: $SERVO_PORT or a sensible default.
	// Common ports: "/dev/ttyUSB0" (Linux), "/dev/cu.usbserial-*" / "/dev/tty.usbmodem*" (macOS)
	port := os.Getenv("SERVO_PORT")
	if port == "" {
		port = "/dev/cu.usbmodem5A7A0594021"
	}

	bus, err := feetech.NewBus(feetech.BusConfig{
		Port:     port,
		BaudRate: 1000000,
		Protocol: feetech.ProtocolSTS,
		Timeout:  time.Second,
	})
	if err != nil {
		log.Fatalf("Failed to create bus: %v", err)
	}
	defer bus.Close()

	// Discover pings each ID in turn — it reliably finds every servo on the bus
	// (works for STS and SCS). Each absent ID costs one PingTimeout; tune
	// BusConfig.PingTimeout to trade coverage for speed.
	fmt.Printf("Discovering servos on %s ...\n\n", port)
	servos, err := bus.Discover(ctx)
	if err != nil {
		log.Fatalf("Discovery failed: %v", err)
	}

	if len(servos) == 0 {
		fmt.Println("No servos found.")
		fmt.Println("\nTroubleshooting:")
		fmt.Println("1. Check that servos are powered on")
		fmt.Println("2. Verify the serial port ($SERVO_PORT)")
		fmt.Println("3. Ensure baudrate matches servo settings (default: 1000000)")
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

	// To scan only part of the ID space (faster), use Scan directly:
	//   servos, _ := bus.Scan(ctx, 1, 12)
	//
	// BroadcastPing is a single fast 0xFE ping (STS only). WARNING: with more
	// than one servo on the bus the simultaneous replies collide and it usually
	// returns just one servo — only use it when a single servo is attached.

	// Read position from the first discovered servo.
	first := feetech.NewServo(bus, servos[0].ID, servos[0].Model)
	fmt.Printf("Reading position from servo %d...\n", servos[0].ID)
	if pos, err := first.Position(ctx); err != nil {
		fmt.Printf("Failed to read position: %v\n", err)
	} else {
		fmt.Printf("Current position: %d\n", pos)
	}
}
