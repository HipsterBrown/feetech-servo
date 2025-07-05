# Feetech Servo Package

[![Go Reference](https://pkg.go.dev/badge/github.com/hipsterbrown/feetech-servo.svg)](https://pkg.go.dev/github.com/hipsterbrown/feetech-servo)
[![Go Report Card](https://goreportcard.com/badge/github.com/hipsterbrown/feetech-servo)](https://goreportcard.com/report/github.com/hipsterbrown/feetech-servo)

A Go package for communicating with Feetech servo motors using serial communication protocols. This package provides a clean, idiomatic Go interface for controlling Feetech servos, particularly the STS3215 series used in robotics applications.

## Features

- **Multi-protocol support**: Compatible with both Protocol 0 and Protocol 1
- **Thread-safe operations**: All operations are mutex-protected for concurrent use
- **Structured error handling**: Custom error types with proper error wrapping
- **Model-based register access**: Type-safe register operations using servo specifications
- **Bulk operations**: Efficient synchronized control of multiple servos
- **Resource management**: Proper cleanup with explicit Close() methods
- **Comprehensive testing**: Unit tests and benchmarks included

## Supported Hardware

Currently supports:
- **STS3215**: 12-bit resolution servo with 0-4095 position range
- Protocol 0 and Protocol 1 communication

Easy to extend for additional Feetech servo models.

## Installation

```bash
go get github.com/hipsterbrown/feetech-servo
```

## Quick Start

```go
package main

import (
    "log"
    "time"
    
    "github.com/hipsterbrown/feetech-servo"
)

func main() {
    // Create a new servo bus
    bus, err := feetech.NewBus(feetech.BusConfig{
        Port:     "/dev/ttyUSB0",
        Baudrate: 1000000,
        Protocol: feetech.ProtocolV0,
        Timeout:  time.Second,
    })
    if err != nil {
        log.Fatal("Failed to create bus:", err)
    }
    defer bus.Close()

    // Create a servo instance
    servo := bus.Servo(1) // Servo with ID 1

    // Ping the servo to verify connection
    modelNum, err := servo.Ping()
    if err != nil {
        log.Fatal("Failed to ping servo:", err)
    }
    log.Printf("Connected to servo model: %d", modelNum)

    // Enable torque and move to center position
    if err := servo.SetTorqueEnable(true); err != nil {
        log.Fatal("Failed to enable torque:", err)
    }

    if err := servo.WritePosition(2048); err != nil {
        log.Fatal("Failed to write position:", err)
    }

    log.Println("Servo moved to center position")
}
```

## Core Interfaces

### Bus

The `Bus` type manages serial communication with multiple servos:

```go
type Bus struct {
    // Internal fields for serial communication, protocol handling, and thread safety
}

// Create a new bus
func NewBus(config BusConfig) (*Bus, error)

// Create a servo instance for the given ID  
func (b *Bus) Servo(id int) *Servo

// Synchronously write positions to multiple servos
func (b *Bus) SyncWritePositions(servos []*Servo, positions []int) error

// Close the bus and release resources
func (b *Bus) Close() error
```

### Servo

The `Servo` type represents an individual servo motor:

```go
type Servo struct {
    ID    int
    Model string
    // Internal reference to bus
}

// Basic operations
func (s *Servo) Ping() (int, error)
func (s *Servo) ReadPosition() (int, error)
func (s *Servo) WritePosition(position int) error
func (s *Servo) ReadVelocity() (int, error)
func (s *Servo) WriteVelocity(velocity int) error

// Control operations
func (s *Servo) SetTorqueEnable(enable bool) error
func (s *Servo) IsMoving() (bool, error)
func (s *Servo) SetOperatingMode(mode byte) error

// Model-based register access
func (s *Servo) ReadRegisterByName(name string) ([]byte, error)
func (s *Servo) WriteRegisterByName(name string, data []byte) error
```

### Configuration

```go
type BusConfig struct {
    Port     string        // Serial port path (e.g., "/dev/ttyUSB0")
    Baudrate int           // Communication speed (default: 1000000)
    Protocol int           // Protocol version: ProtocolV0 or ProtocolV1
    Timeout  time.Duration // Communication timeout (default: 1 second)
}
```

## Examples

### Single Servo Control

```go
func singleServoExample() {
    bus, err := feetech.NewBus(feetech.BusConfig{
        Port:     "/dev/ttyUSB0",
        Baudrate: 1000000,
        Protocol: feetech.ProtocolV0,
    })
    if err != nil {
        log.Fatal(err)
    }
    defer bus.Close()

    servo := bus.Servo(1)
    
    // Set to position control mode
    servo.SetOperatingMode(feetech.OperatingModePosition)
    servo.SetTorqueEnable(true)
    
    // Read current position
    pos, err := servo.ReadPosition()
    if err != nil {
        log.Fatal(err)
    }
    log.Printf("Current position: %d", pos)
    
    // Move to new position
    servo.WritePosition(3000)
    
    // Wait for movement to complete
    for {
        moving, _ := servo.IsMoving()
        if !moving {
            break
        }
        time.Sleep(50 * time.Millisecond)
    }
}
```

### Multi-Servo Robot Arm

```go
type RobotArm struct {
    bus    *feetech.Bus
    servos []*feetech.Servo
}

func NewRobotArm(port string, servoIDs []int) (*RobotArm, error) {
    bus, err := feetech.NewBus(feetech.BusConfig{
        Port:     port,
        Baudrate: 1000000,
        Protocol: feetech.ProtocolV0,
    })
    if err != nil {
        return nil, err
    }

    servos := make([]*feetech.Servo, len(servoIDs))
    for i, id := range servoIDs {
        servos[i] = bus.Servo(id)
        
        // Initialize each servo
        servos[i].SetOperatingMode(feetech.OperatingModePosition)
        servos[i].SetTorqueEnable(true)
    }

    return &RobotArm{bus: bus, servos: servos}, nil
}

func (r *RobotArm) MoveToPositions(positions []int) error {
    // Move all joints simultaneously using sync write
    return r.bus.SyncWritePositions(r.servos, positions)
}

func (r *RobotArm) GetJointPositions() ([]int, error) {
    positions := make([]int, len(r.servos))
    for i, servo := range r.servos {
        pos, err := servo.ReadPosition()
        if err != nil {
            return nil, err
        }
        positions[i] = pos
    }
    return positions, nil
}

func (r *RobotArm) Close() error {
    // Disable torque for safety
    for _, servo := range r.servos {
        servo.SetTorqueEnable(false)
    }
    return r.bus.Close()
}

// Usage
func robotArmExample() {
    arm, err := NewRobotArm("/dev/ttyUSB0", []int{1, 2, 3, 4, 5})
    if err != nil {
        log.Fatal(err)
    }
    defer arm.Close()

    // Move to target configuration
    targetPositions := []int{2048, 1800, 2300, 2048, 2048}
    if err := arm.MoveToPositions(targetPositions); err != nil {
        log.Fatal(err)
    }
}
```

### Advanced Register Access

```go
func advancedExample() {
    bus, _ := feetech.NewBus(feetech.BusConfig{
        Port:     "/dev/ttyUSB0",
        Baudrate: 1000000,
        Protocol: feetech.ProtocolV0,
    })
    defer bus.Close()

    servo := bus.Servo(1)
    
    // Read servo status using named registers
    voltage, err := servo.ReadRegisterByName("present_voltage")
    if err != nil {
        log.Fatal(err)
    }
    log.Printf("Voltage: %dV", voltage[0])
    
    temperature, err := servo.ReadRegisterByName("present_temperature")
    if err != nil {
        log.Fatal(err)
    }
    log.Printf("Temperature: %d°C", temperature[0])
    
    // Set position limits
    minLimit := []byte{0x00, 0x02} // 512 in little-endian
    maxLimit := []byte{0x00, 0x0E} // 3584 in little-endian
    
    servo.WriteRegisterByName("min_position_limit", minLimit)
    servo.WriteRegisterByName("max_position_limit", maxLimit)
}
```

## Error Handling

The package provides structured error types for better error handling:

```go
func handleErrors() {
    bus, _ := feetech.NewBus(feetech.BusConfig{Port: "/dev/ttyUSB0"})
    servo := bus.Servo(1)
    
    _, err := servo.ReadPosition()
    if err != nil {
        // Check for specific error types
        switch e := err.(type) {
        case *feetech.ServoError:
            log.Printf("Servo %d error during %s: %v", e.ServoID, e.Op, e.Err)
        case *feetech.CommunicationError:
            log.Printf("Communication error (code %d): %s", e.Code, e.Msg)
        default:
            log.Printf("Unknown error: %v", err)
        }
    }
}
```

## Operating Modes

The STS3215 supports multiple operating modes:

```go
const (
    OperatingModePosition = 0  // Position control (default)
    OperatingModeVelocity = 1  // Velocity control
    OperatingModePWM      = 2  // PWM control
    OperatingModeStep     = 3  // Step control
)

// Set operating mode
servo.SetOperatingMode(feetech.OperatingModeVelocity)
```

## Thread Safety

All operations are thread-safe and can be called from multiple goroutines:

```go
func concurrentExample() {
    bus, _ := feetech.NewBus(feetech.BusConfig{Port: "/dev/ttyUSB0"})
    defer bus.Close()
    
    servo1 := bus.Servo(1)
    servo2 := bus.Servo(2)
    
    // Safe to call from multiple goroutines
    go func() {
        servo1.WritePosition(2048)
    }()
    
    go func() {
        servo2.WritePosition(1024)
    }()
}
```

## Testing

Run the test suite:

```bash
go test ./...
```

Run benchmarks:

```bash
go test -bench=.
```

Run tests with coverage:

```bash
go test -cover ./...
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Add tests for new functionality
5. Ensure all tests pass (`go test ./...`)
6. Commit your changes (`git commit -am 'Add amazing feature'`)
7. Push to the branch (`git push origin feature/amazing-feature`)
8. Open a Pull Request

### Development Guidelines

- Follow standard Go formatting (`go fmt`)
- Add tests for new functionality
- Update documentation for API changes
- Ensure thread safety for concurrent operations
- Use structured error types for error conditions

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Inspired by the Python [lerobot](https://github.com/huggingface/lerobot) FeetechMotorsBus implementation
- Feetech for their servo motor documentation and protocol specifications
