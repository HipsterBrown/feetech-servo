# Feetech Servo Package

[![Go Reference](https://pkg.go.dev/badge/github.com/hipsterbrown/feetech-servo.svg)](https://pkg.go.dev/github.com/hipsterbrown/feetech-servo)
[![Go Report Card](https://goreportcard.com/badge/github.com/hipsterbrown/feetech-servo)](https://goreportcard.com/report/github.com/hipsterbrown/feetech-servo)

A Go package for communicating with Feetech servo motors using serial communication protocols. This package provides a clean, idiomatic Go interface for controlling Feetech servos, particularly the STS and SCS series used in robotics applications.

## Features

- **Context-aware operations**: Proper cancellation and timeout support
- **Multi-protocol support**: Compatible with both STS (Protocol 0) and SCS (Protocol 1)
- **Thread-safe operations**: All operations are mutex-protected for concurrent use
- **ServoGroup**: Coordinated control of multiple servos with map-based positioning
- **Model-based register access**: Type-safe register operations using servo specifications
- **Bulk operations**: Efficient synchronized control using SyncRead/SyncWrite
- **Multiple normalization modes**: Work with raw values or application-defined normalization
- **Testing support**: MockTransport for unit testing without hardware
- **Resource management**: Proper cleanup with explicit Close() methods

## Supported Hardware

Currently supports:
- **STS3215**: 12-bit resolution servo (0-4095) - STS/Protocol 0
- **STS3250**: Enhanced version of STS3215 - STS/Protocol 0
- **SCS0009**: 10-bit resolution servo (0-1023) - SCS/Protocol 1
- **SM8512BL**: 16-bit resolution servo (0-65535) - STS/Protocol 0

Easy to extend for additional Feetech servo models.

## Installation

```bash
go get github.com/hipsterbrown/feetech-servo/feetech
```

## Quick Start

```go
package main

import (
    "context"
    "log"

    "github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
    ctx := context.Background()

    // Create a new servo bus
    bus, err := feetech.NewBus(feetech.BusConfig{
        Port:     "/dev/ttyUSB0",
        BaudRate: 1000000,
        Protocol: feetech.ProtocolSTS,
    })
    if err != nil {
        log.Fatal("Failed to create bus:", err)
    }
    defer bus.Close()

    // Create a servo instance (defaults to STS3215)
    servo := feetech.NewServo(bus, 1, nil)

    // Detect model
    if err := servo.DetectModel(ctx); err != nil {
        log.Fatal("Failed to detect model:", err)
    }
    log.Printf("Connected to: %s", servo.Model().Name)

    // Enable torque and move to center position
    servo.Enable(ctx)
    servo.SetPosition(ctx, 2048) // Center position for 12-bit servo

    // Read current position
    pos, _ := servo.Position(ctx)
    log.Printf("Current position: %d", pos)
}
```

## Core Interfaces

### Bus

The `Bus` type manages serial communication with servos:

```go
// Create a new bus
func NewBus(config BusConfig) (*Bus, error)

// Discovery
func (b *Bus) Discover(ctx context.Context) ([]FoundServo, error)
func (b *Bus) Scan(ctx context.Context, startID, endID int) ([]FoundServo, error)

// Broadcast action (execute buffered reg writes)
func (b *Bus) Action(ctx context.Context) error

// Low-level operations
func (b *Bus) Read(ctx context.Context, id int, address byte, length byte) ([]byte, error)
func (b *Bus) Write(ctx context.Context, id int, address byte, data []byte) error
func (b *Bus) SyncRead(ctx context.Context, address, size byte, ids []int) (map[int][]byte, error)
func (b *Bus) SyncWrite(ctx context.Context, address, size byte, data map[int][]byte) error
func (b *Bus) RegWrite(ctx context.Context, id int, address byte, data []byte) error

// Close the bus and release resources
func (b *Bus) Close() error
```

### Servo

The `Servo` type represents an individual servo motor:

```go
// Create a new servo
func NewServo(bus *Bus, id int, model *ServoModel) *Servo

// Basic operations
func (s *Servo) Ping(ctx context.Context) (int, error)
func (s *Servo) DetectModel(ctx context.Context) error
func (s *Servo) Position(ctx context.Context) (int, error)
func (s *Servo) SetPosition(ctx context.Context, position int) error
func (s *Servo) SetPositionWithSpeed(ctx context.Context, position, speed int) error
func (s *Servo) SetPositionWithTime(ctx context.Context, position, timeMs int) error
func (s *Servo) Velocity(ctx context.Context) (int, error)
func (s *Servo) SetVelocity(ctx context.Context, velocity int) error

// Torque control
func (s *Servo) Enable(ctx context.Context) error
func (s *Servo) Disable(ctx context.Context) error
func (s *Servo) SetTorqueEnabled(ctx context.Context, enable bool) error
func (s *Servo) TorqueEnabled(ctx context.Context) (bool, error)

// Status operations
func (s *Servo) Moving(ctx context.Context) (bool, error)
func (s *Servo) Load(ctx context.Context) (int, error)
func (s *Servo) Voltage(ctx context.Context) (int, error)
func (s *Servo) Temperature(ctx context.Context) (int, error)

// Operating mode
func (s *Servo) OperatingMode(ctx context.Context) (byte, error)
func (s *Servo) SetOperatingMode(ctx context.Context, mode byte) error

// Model information
func (s *Servo) Model() *ServoModel
func (s *Servo) ID() int

// Configuration (EEPROM writes)
func (s *Servo) SetID(ctx context.Context, newID int) error
func (s *Servo) SetBaudRate(ctx context.Context, baudrate int) error
func (s *Servo) PositionLimits(ctx context.Context) (min, max int, err error)
func (s *Servo) SetPositionLimits(ctx context.Context, min, max int) error

// Register access
func (s *Servo) ReadRegister(ctx context.Context, name string) ([]byte, error)
func (s *Servo) WriteRegister(ctx context.Context, name string, data []byte) error
```

### ServoGroup

The `ServoGroup` type enables coordinated multi-servo operations:

```go
// Create a servo group
func NewServoGroup(bus *Bus, servos ...*Servo) *ServoGroup
func NewServoGroupByIDs(bus *Bus, ids ...int) *ServoGroup

// Position control (map-based)
func (g *ServoGroup) Positions(ctx context.Context) (PositionMap, error)
func (g *ServoGroup) SetPositions(ctx context.Context, positions PositionMap) error
func (g *ServoGroup) SetPositionsWithSpeed(ctx context.Context, positions, speeds PositionMap) error
func (g *ServoGroup) SetPositionsWithTime(ctx context.Context, positions, times PositionMap) error

// Coordinated motion
func (g *ServoGroup) MoveTo(ctx context.Context, positions PositionMap, timeoutMs int) (PositionMap, error)
func (g *ServoGroup) RegWritePositions(ctx context.Context, positions PositionMap) error

// Group control
func (g *ServoGroup) EnableAll(ctx context.Context) error
func (g *ServoGroup) DisableAll(ctx context.Context) error
func (g *ServoGroup) WaitForStop(ctx context.Context, timeoutMs int) (PositionMap, error)

// Register operations
func (g *ServoGroup) ReadRegister(ctx context.Context, registerName string) (map[int][]byte, error)
func (g *ServoGroup) WriteRegister(ctx context.Context, registerName string, data map[int][]byte) error

// Introspection
func (g *ServoGroup) Servos() []*Servo
func (g *ServoGroup) IDs() []int
func (g *ServoGroup) Servo(index int) *Servo
func (g *ServoGroup) ServoByID(id int) *Servo
```

### Configuration

```go
type BusConfig struct {
    Port      string               // Serial port path (e.g., "/dev/ttyUSB0")
    BaudRate  int                  // Communication speed (default: 1000000)
    Protocol  int                  // Protocol: ProtocolSTS or ProtocolSCS
    Timeout   time.Duration        // Communication timeout (default: 1 second)
    Transport Transport            // Optional: custom transport (for testing)
}

type FoundServo struct {
    ID          int          // Servo ID found on bus
    ModelNumber int          // Hardware model number
    Model       *ServoModel  // Model specification (nil if unknown)
}

// PositionMap is used for map-based servo control
type PositionMap map[int]int
```

## Examples

### Basic Servo Control

```go
package main

import (
    "context"
    "log"
    "time"

    "github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
    ctx := context.Background()

    bus, err := feetech.NewBus(feetech.BusConfig{
        Port:     "/dev/ttyUSB0",
        BaudRate: 1000000,
        Protocol: feetech.ProtocolSTS,
    })
    if err != nil {
        log.Fatal(err)
    }
    defer bus.Close()

    // Create servo with specific model
    servo := feetech.NewServo(bus, 1, &feetech.ModelSTS3215)

    // Enable and move with speed control
    servo.Enable(ctx)
    servo.SetPositionWithSpeed(ctx, 2048, 200)

    // Wait for movement to complete
    for {
        moving, _ := servo.Moving(ctx)
        if !moving {
            break
        }
        time.Sleep(50 * time.Millisecond)
    }

    // Read final position
    pos, _ := servo.Position(ctx)
    log.Printf("Final position: %d", pos)
}
```

### Servo Discovery

```go
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
        Port:     "/dev/ttyUSB0",
        BaudRate: 1000000,
        Protocol: feetech.ProtocolSTS,
    })
    if err != nil {
        log.Fatal(err)
    }
    defer bus.Close()

    // Method 1: Broadcast discovery (fast, STS protocol only)
    servos, err := bus.Discover(ctx)
    if err != nil {
        log.Fatal(err)
    }

    fmt.Printf("Found %d servos:\n", len(servos))
    for _, s := range servos {
        fmt.Printf("  ID: %d, Model: %s (number: %d)\n",
            s.ID, s.Model.Name, s.ModelNumber)
    }

    // Method 2: Sequential scanning (works with both protocols)
    servos, err = bus.Scan(ctx, 0, 10) // Scan IDs 0-10
    if err != nil {
        log.Fatal(err)
    }
}
```

### Multi-Servo Control with ServoGroup

```go
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
        Port: "/dev/ttyUSB0",
    })
    if err != nil {
        log.Fatal(err)
    }
    defer bus.Close()

    // Create a group of servos by IDs
    group := feetech.NewServoGroupByIDs(bus, 1, 2, 3, 4, 5)

    // Enable all servos in the group
    group.EnableAll(ctx)

    // Map-based position control
    positions := feetech.PositionMap{
        1: 2000,
        2: 2000,
        3: 2000,
        4: 2000,
        5: 2000,
    }

    speeds := feetech.PositionMap{
        1: 200,
        2: 200,
        3: 200,
        4: 200,
        5: 200,
    }

    // Move with speed control
    group.SetPositionsWithSpeed(ctx, positions, speeds)

    // Read all positions efficiently (single sync read)
    currentPos, _ := group.Positions(ctx)
    fmt.Printf("Current positions: %v\n", currentPos)
}
```

### Partial Updates with PositionMap

```go
package main

import (
    "context"
    "log"

    "github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
    ctx := context.Background()

    bus, _ := feetech.NewBus(feetech.BusConfig{Port: "/dev/ttyUSB0"})
    defer bus.Close()

    // Group has 5 servos
    group := feetech.NewServoGroupByIDs(bus, 1, 2, 3, 4, 5)

    // Only move servos 1 and 3 (partial update)
    positions := feetech.PositionMap{
        1: 1000,
        3: 2000,
    }
    group.SetPositions(ctx, positions)

    // Read only specific servos
    allPos, _ := group.Positions(ctx)
    log.Printf("Servo 1: %d, Servo 3: %d", allPos[1], allPos[3])
}
```

### Coordinated Motion (RegWrite + Action)

```go
package main

import (
    "context"
    "log"

    "github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
    ctx := context.Background()

    bus, _ := feetech.NewBus(feetech.BusConfig{Port: "/dev/ttyUSB0"})
    defer bus.Close()

    group := feetech.NewServoGroupByIDs(bus, 1, 2, 3)
    group.EnableAll(ctx)

    // Buffer position writes (doesn't execute yet)
    positions := feetech.PositionMap{1: 1000, 2: 2000, 3: 3000}
    group.RegWritePositions(ctx, positions)

    // Execute all buffered writes simultaneously
    bus.Action(ctx)

    // Wait for movement to complete
    group.WaitForStop(ctx, 5000) // 5 second timeout
}
```

### Move and Wait Pattern

```go
package main

import (
    "context"
    "fmt"
    "log"

    "github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
    ctx := context.Background()

    bus, _ := feetech.NewBus(feetech.BusConfig{Port: "/dev/ttyUSB0"})
    defer bus.Close()

    group := feetech.NewServoGroupByIDs(bus, 1, 2, 3)
    group.EnableAll(ctx)

    // Move and wait for completion
    targetPos := feetech.PositionMap{1: 1000, 2: 2000, 3: 3000}
    finalPos, err := group.MoveTo(ctx, targetPos, 5000) // 5s timeout
    if err != nil {
        log.Fatal(err)
    }

    fmt.Printf("Final positions: %v\n", finalPos)
}
```

### Servo Configuration

```go
package main

import (
    "context"
    "fmt"
    "log"

    "github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
    ctx := context.Background()

    // Connect at current baudrate
    bus, err := feetech.NewBus(feetech.BusConfig{
        Port:     "/dev/ttyUSB0",
        BaudRate: 57600,
    })
    if err != nil {
        log.Fatal(err)
    }
    defer bus.Close()

    // Discover servo
    servos, err := bus.Discover(ctx)
    if err != nil || len(servos) != 1 {
        log.Fatal("Expected exactly one servo")
    }

    servo := feetech.NewServo(bus, servos[0].ID, servos[0].Model)

    // Change ID and baudrate (writes to EEPROM)
    if err := servo.SetID(ctx, 5); err != nil {
        log.Fatal(err)
    }
    fmt.Println("✓ Servo ID changed to 5")

    if err := servo.SetBaudRate(ctx, 1000000); err != nil {
        log.Fatal(err)
    }
    fmt.Println("✓ Baudrate set to 1000000")

    // Reconnect at new settings
    bus.Close()
    bus, _ = feetech.NewBus(feetech.BusConfig{
        Port:     "/dev/ttyUSB0",
        BaudRate: 1000000,
    })
    defer bus.Close()

    // Verify
    newServo := feetech.NewServo(bus, 5, nil)
    _, err = newServo.Ping(ctx)
    if err != nil {
        log.Fatal("Failed to ping at new settings")
    }
    fmt.Println("✓ Setup complete")
}
```

### Context Usage Patterns

```go
package main

import (
    "context"
    "fmt"
    "time"

    "github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
    bus, _ := feetech.NewBus(feetech.BusConfig{Port: "/dev/ttyUSB0"})
    defer bus.Close()
    servo := feetech.NewServo(bus, 1, nil)

    // Pattern 1: Basic context
    ctx := context.Background()
    servo.SetPosition(ctx, 2048)

    // Pattern 2: Context with timeout
    ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
    defer cancel()
    pos, err := servo.Position(ctx)
    if err != nil {
        fmt.Println("Operation timed out or failed:", err)
    }

    // Pattern 3: Context with cancellation
    ctx, cancel = context.WithCancel(context.Background())
    go func() {
        time.Sleep(1 * time.Second)
        cancel() // Cancel after 1 second
    }()

    _, err = servo.Position(ctx)
    if err == context.Canceled {
        fmt.Println("Operation was cancelled")
    }
}
```

### TinyGo Support through MCU UART Transport

```go
//go:build baremetal

package main

import (
    "context"
    "log"

    "github.com/hipsterbrown/feetech-servo/feetech"
)

func main() {
    ctx := context.Background()

    // Create a new servo bus
    bus, err := feetech.NewBus(feetech.BusConfig{
        Port:     "0",
        BaudRate: 1000000,
        Protocol: feetech.ProtocolSTS,
    })
    if err != nil {
        log.Fatal("Failed to create bus:", err)
    }
    defer bus.Close()

    // Create a servo instance (defaults to STS3215)
    servo := feetech.NewServo(bus, 1, nil)

    // Detect model
    if err := servo.DetectModel(ctx); err != nil {
        log.Fatal("Failed to detect model:", err)
    }
    log.Printf("Connected to: %s", servo.Model().Name)

    // Enable torque and move to center position
    servo.Enable(ctx)
    servo.SetPosition(ctx, 2048) // Center position for 12-bit servo

    // Read current position
    pos, _ := servo.Position(ctx)
    log.Printf("Current position: %d", pos)
}
```

### Testing with MockTransport

```go
package main

import (
    "context"
    "testing"

    "github.com/hipsterbrown/feetech-servo/feetech"
)

func TestServoPosition(t *testing.T) {
    // Create mock transport with pre-loaded response
    mock := &feetech.MockTransport{
        // Response for position read: [header, header, id, len, err, pos_low, pos_high, checksum]
        ReadData: []byte{0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2},
    }

    bus, err := feetech.NewBus(feetech.BusConfig{
        Transport: mock,
    })
    if err != nil {
        t.Fatal(err)
    }
    defer bus.Close()

    servo := feetech.NewServo(bus, 1, nil)
    ctx := context.Background()

    pos, err := servo.Position(ctx)
    if err != nil {
        t.Fatal(err)
    }

    if pos != 2048 {
        t.Errorf("expected position 2048, got %d", pos)
    }

    // Verify write data
    if len(mock.WriteData) == 0 {
        t.Error("no data was written to mock transport")
    }
}
```

## Working with Different Servo Models

The package supports multiple servo models with different capabilities:

### Supported Models

- **STS3215**: 12-bit resolution (0-4095), STS/Protocol 0, most common
- **STS3250**: Enhanced version of STS3215, same register layout
- **SCS0009**: 10-bit resolution (0-1023), SCS/Protocol 1, different register layout
- **SM8512BL**: 16-bit resolution (0-65535), STS/Protocol 0, high precision

### Creating Servos with Specific Models

```go
ctx := context.Background()
bus, _ := feetech.NewBus(feetech.BusConfig{Port: "/dev/ttyUSB0"})

// Method 1: Default model (STS3215)
servo := feetech.NewServo(bus, 1, nil)

// Method 2: Specify model explicitly
servo = feetech.NewServo(bus, 1, &feetech.ModelSTS3215)
servo = feetech.NewServo(bus, 2, &feetech.ModelSCS0009)
servo = feetech.NewServo(bus, 3, &feetech.ModelSM8512BL)

// Method 3: Auto-detect from hardware
servo = feetech.NewServo(bus, 1, nil)
if err := servo.DetectModel(ctx); err != nil {
    log.Fatal(err)
}
fmt.Printf("Detected: %s\n", servo.Model().Name)
```

### Model Information

```go
servo := feetech.NewServo(bus, 1, &feetech.ModelSTS3215)
model := servo.Model()

fmt.Printf("Name: %s\n", model.Name)
fmt.Printf("Protocol: %d\n", model.Protocol)
fmt.Printf("Resolution: %d bits\n", model.Resolution)
fmt.Printf("Max Position: %d\n", model.MaxPosition)
fmt.Printf("Model Number: %d\n", model.Number)
```

## Operating Modes

Servos support multiple operating modes:

```go
const (
    ModePosition = 0  // Position control (default)
    ModeVelocity = 1  // Velocity control (continuous rotation)
    ModePWM      = 2  // PWM control (direct motor power)
    ModeStep     = 3  // Step control (depends on model)
)

// Set operating mode
servo.SetOperatingMode(ctx, feetech.ModeVelocity)

// Get current mode
mode, _ := servo.OperatingMode(ctx)
```

## Error Handling

The package provides structured error handling:

```go
import "errors"

pos, err := servo.Position(ctx)
if err != nil {
    // Check for context errors
    if errors.Is(err, context.Canceled) {
        log.Println("Operation was cancelled")
        return
    }
    if errors.Is(err, context.DeadlineExceeded) {
        log.Println("Operation timed out")
        return
    }

    // Other errors
    log.Printf("Failed to read position: %v", err)
}
```

## Thread Safety

All operations are thread-safe and can be called from multiple goroutines:

```go
func concurrentControl() {
    ctx := context.Background()
    bus, _ := feetech.NewBus(feetech.BusConfig{Port: "/dev/ttyUSB0"})
    defer bus.Close()

    servo1 := feetech.NewServo(bus, 1, nil)
    servo2 := feetech.NewServo(bus, 2, nil)

    // Safe to call from multiple goroutines
    go servo1.SetPosition(ctx, 1000)
    go servo2.SetPosition(ctx, 2000)
}
```

## Best Practices

### 1. Use Context Properly

Always pass a context and consider using timeouts for hardware operations:

```go
ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
defer cancel()

pos, err := servo.Position(ctx)
```

### 2. Use ServoGroup for Multi-Servo Operations

ServoGroup is more efficient than individual operations:

```go
// Good: Uses single SyncRead
group := feetech.NewServoGroupByIDs(bus, 1, 2, 3, 4, 5)
positions, _ := group.Positions(ctx)

// Less efficient: Multiple individual reads
pos1, _ := servo1.Position(ctx)
pos2, _ := servo2.Position(ctx)
pos3, _ := servo3.Position(ctx)
```

### 3. Use RegWrite + Action for Synchronized Motion

For perfectly synchronized movements:

```go
positions := feetech.PositionMap{1: 1000, 2: 2000, 3: 3000}
group.RegWritePositions(ctx, positions)
bus.Action(ctx) // All servos start moving simultaneously
```

### 4. Handle Errors Appropriately

Always check errors, especially for hardware operations:

```go
if err := servo.SetPosition(ctx, 2048); err != nil {
    log.Printf("Failed to set position: %v", err)
    return
}
```

### 5. Close Resources

Always close the bus when done:

```go
bus, err := feetech.NewBus(config)
if err != nil {
    return err
}
defer bus.Close() // Ensures cleanup even on error
```

## Testing

Run the test suite:

```bash
go test ./...
```

Run with coverage:

```bash
go test -cover ./...
```

Run specific tests:

```bash
go test -v -run TestServoPosition ./feetech
```

## Examples Directory

See the [_examples/](_examples/) directory for complete working examples:

- **basic_usage**: Simple servo control
- **discover**: Servo discovery and scanning
- **servo_group**: Multi-servo coordinated control
- **coordinated_motion**: Synchronized movement with RegWrite + Action
- **configure_servo**: ID and baudrate configuration
- **mock_transport**: Testing with MockTransport
- **monitor**: Reading servo status (voltage, temperature, load)
- **scanning**: Sequential ID scanning
- **wheel_mode**: Continuous rotation mode

## Migration from v0.3.0

If you're upgrading from v0.3.0, please see the comprehensive [MIGRATION.md](MIGRATION.md) guide which covers:

- Import path changes
- Method renames and signature changes
- Context usage patterns
- Calibration migration strategies
- Complete before/after examples

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
- Use context for cancellation support
- See [CLAUDE.md](CLAUDE.md) for architecture details

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Inspired by the Python [lerobot](https://github.com/huggingface/lerobot) FeetechMotorsBus implementation
- Feetech for their servo motor documentation and protocol specifications
