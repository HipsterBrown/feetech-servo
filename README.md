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
- **Motor calibration system**: Position normalization with homing offsets and drive mode inversion
- **Multiple normalization modes**: Raw values, percentages, and degrees
- **Resource management**: Proper cleanup with explicit Close() methods

## Supported Hardware

Currently supports:
- **STS3215**: 12-bit resolution servo with 0-4095 position range
- **STS3250**: Enhanced version of STS3215
- **SCS0009**: 10-bit resolution servo for Protocol 1
- **SM8512BL**: 16-bit resolution servo
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
    servo := bus.Servo(1) // Servo with ID 1 (defaults to STS3215)

    // Ping the servo and auto-detect model
    modelNum, err := servo.PingAndDetect()
    if err != nil {
        log.Fatal("Failed to ping servo:", err)
    }
    log.Printf("Connected to servo model: %s (number: %d)", servo.Model, modelNum)

    // Enable torque and move to center position
    if err := servo.SetTorqueEnable(true); err != nil {
        log.Fatal("Failed to enable torque:", err)
    }

    if err := servo.WritePosition(2048.0, false); err != nil { // Raw position
        log.Fatal("Failed to write position:", err)
    }

    log.Println("Servo moved to center position")
}
```

## Motor Calibration

The package includes a comprehensive calibration system for robotics applications:

### Calibration File Format

Calibrations are stored in JSON format with motor names as keys:

```json
{
    "shoulder_pan": {
        "id": 1,
        "drive_mode": 0,
        "homing_offset": -1470,
        "range_min": 758,
        "range_max": 3292
    },
    "gripper": {
        "id": 6,
        "drive_mode": 0,
        "homing_offset": 1407,
        "range_min": 2031,
        "range_max": 3476,
        "norm_mode": 1
    }
}
```

### Calibration Parameters

- **`id`**: Servo ID on the bus
- **`drive_mode`**: 0=normal direction, 1=inverted
- **`homing_offset`**: Firmware offset written to servo register
- **`range_min`**: Minimum usable position (after homing offset)
- **`range_max`**: Maximum usable position (after homing offset)
- **`norm_mode`**: Normalization mode (optional, defaults to degrees)

### Normalization Modes

- **0 (Raw)**: Raw servo values (0-4095)
- **1 (Range100)**: 0-100% range (good for grippers)
- **2 (RangeM100)**: -100 to +100 range
- **3 (Degrees)**: -180° to +180° range (default, good for joints)

### Using Calibrations

```go
package main

import (
    "log"
    "github.com/hipsterbrown/feetech-servo"
)

func main() {
    // Load calibrations from file
    calibrations, err := feetech.LoadCalibrations("robot_cal.json")
    if err != nil {
        log.Fatal("Failed to load calibrations:", err)
    }

    // Create bus with calibrations
    bus, err := feetech.NewBus(feetech.BusConfig{
        Port:         "/dev/ttyUSB0",
        Baudrate:     1000000,
        Protocol:     feetech.ProtocolV0,
        Calibrations: calibrations,
    })
    if err != nil {
        log.Fatal("Failed to create bus:", err)
    }
    defer bus.Close()

    // Initialize servos with homing offsets
    for id, cal := range calibrations {
        servo := bus.Servo(id)
        
        // Apply homing offset to servo firmware
        if err := cal.ApplyHomingOffset(servo); err != nil {
            log.Printf("Warning: Failed to apply homing offset for servo %d: %v", id, err)
        }
        
        // Enable torque
        servo.SetTorqueEnable(true)
    }

    // Use normalized positioning
    shoulderPan := bus.Servo(1)
    shoulderPan.WritePosition(45.0, true) // Move to 45 degrees (normalized)
    
    gripper := bus.Servo(6)
    gripper.WritePosition(75.0, true) // Move to 75% open (normalized)
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

// Create a servo instance for the given ID (defaults to STS3215 model)
func (b *Bus) Servo(id int) *Servo

// Create a servo instance with a specific model
func (b *Bus) ServoWithModel(id int, model string) (*Servo, error)

// Synchronously write positions to multiple servos
func (b *Bus) SyncWritePositions(servos []*Servo, positions []float64, normalize bool) error

// Synchronously read positions from multiple servos
func (b *Bus) SyncReadPositions(servos []*Servo, normalize bool) (map[int]float64, error)

// Servo discovery and setup
func (b *Bus) DiscoverServos() ([]DiscoveredServo, error)
func (b *Bus) ScanServoIDs(startID, endID int) ([]DiscoveredServo, error)
func (b *Bus) DiscoverServoAtBaudrates(baudrates []int, expectedModel string) (*DiscoveredServo, int, error)

// Calibration management
func (b *Bus) SetCalibration(servoID int, calibration *MotorCalibration)
func (b *Bus) GetCalibration(servoID int) (*MotorCalibration, bool)
func (b *Bus) IsCalibrated(servoID int) bool

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
func (s *Servo) PingAndDetect() (int, error)  // Ping and auto-detect model
func (s *Servo) ReadPosition(normalize bool) (float64, error)
func (s *Servo) WritePosition(position float64, normalize bool) error
func (s *Servo) ReadVelocity(normalize bool) (float64, error)
func (s *Servo) WriteVelocity(velocity float64, normalize bool) error

// Control operations
func (s *Servo) SetTorqueEnable(enable bool) error
func (s *Servo) IsMoving() (bool, error)
func (s *Servo) SetOperatingMode(mode byte) error
func (s *Servo) GetOperatingMode() (byte, error)

// Model operations
func (s *Servo) DetectModel() error
func (s *Servo) GetModelInfo() (map[string]interface{}, error)

// Setup operations (writes to EEPROM)
func (s *Servo) SetServoID(newID int) error
func (s *Servo) SetBaudrate(baudrate int) error

// Model-based register access
func (s *Servo) ReadRegisterByName(name string) ([]byte, error)
func (s *Servo) WriteRegisterByName(name string, data []byte) error
```

### Configuration

```go
type BusConfig struct {
    Port         string                        // Serial port path (e.g., "/dev/ttyUSB0")
    Baudrate     int                           // Communication speed (default: 1000000)
    Protocol     int                           // Protocol version: ProtocolV0 or ProtocolV1
    Timeout      time.Duration                 // Communication timeout (default: 1 second)
    Calibrations map[int]*MotorCalibration     // Optional motor calibrations
}

type DiscoveredServo struct {
    ID          int    // Servo ID found on bus
    ModelNumber int    // Hardware model number
    ModelName   string // Model name (e.g., "sts3215")
}
```

## Working with Different Servo Models

The package supports multiple servo models with different capabilities:

### Supported Models

- **STS3215**: 12-bit resolution (0-4095), Protocol 0, most common
- **STS3250**: Enhanced version of STS3215, same register layout
- **SCS0009**: 10-bit resolution (0-1023), Protocol 1, different register layout  
- **SM8512BL**: 16-bit resolution (0-65535), Protocol 0, high precision

### Creating Servos with Specific Models

```go
// Method 1: Default model (STS3215)
servo := bus.Servo(1)
servo.Model // "sts3215"

// Method 2: Specify model explicitly
servo, err := bus.ServoWithModel(1, "scs0009")
if err != nil {
    log.Fatal("Invalid model:", err)
}

// Method 3: Auto-detect model from hardware
servo := bus.Servo(1)
modelNum, err := servo.PingAndDetect()
if err != nil {
    log.Fatal("Failed to detect model:", err)
}
fmt.Printf("Detected model: %s (number: %d)\n", servo.Model, modelNum)

// Method 4: Detect model separately
servo := bus.Servo(1)
err := servo.DetectModel()
if err != nil {
    log.Fatal("Failed to detect model:", err)
}
```

### Model Information

```go
// Get detailed model information
servo, _ := bus.ServoWithModel(1, "sts3215")
info, err := servo.GetModelInfo()
if err != nil {
    log.Fatal(err)
}

fmt.Printf("Model: %s\n", info["name"])
fmt.Printf("Resolution: %d\n", info["resolution"])
fmt.Printf("Max Position: %d\n", info["max_position"])
fmt.Printf("Protocol: %d\n", info["protocol"])
```

### Multi-Model Bus Example

```go
func multiModelExample() {
    bus, err := feetech.NewBus(feetech.BusConfig{
        Port:     "/dev/ttyUSB0",
        Protocol: feetech.ProtocolV0, // Both protocols supported
    })
    if err != nil {
        log.Fatal(err)
    }
    defer bus.Close()

    // Create servos with known models
    sts3215, err := bus.ServoWithModel(1, "sts3215")
    if err != nil {
        log.Fatal(err)
    }
    
    scs0009, err := bus.ServoWithModel(2, "scs0009")
    if err != nil {
        log.Fatal(err)
    }
    
    highRes, err := bus.ServoWithModel(3, "sm8512bl")
    if err != nil {
        log.Fatal(err)
    }

    // Auto-detect unknown servo
    unknownServo := bus.Servo(4)
    modelNum, err := unknownServo.PingAndDetect()
    if err != nil {
        log.Printf("Servo 4 not responding: %v", err)
    } else {
        fmt.Printf("Servo 4 is model: %s (number: %d)\n", unknownServo.Model, modelNum)
    }

    // Use model-specific features
    // STS3215: 12-bit precision
    sts3215.WritePosition(2048.0, false) // Middle position
    
    // SCS0009: 10-bit precision, different registers
    scs0009.WriteRegisterByName("running_time", []byte{0x00, 0x32}) // Uses SCS0009 register map
    
    // SM8512BL: 16-bit precision
    highRes.WritePosition(32768.0, false) // Middle position with high precision
}
```

### Important Notes

- **Register Access**: `ReadRegisterByName` and `WriteRegisterByName` require the correct model to be set
- **Protocol Compatibility**: SCS0009 uses Protocol 1, others use Protocol 0
- **Resolution Differences**: Position values scale with model resolution
- **Model Detection**: Use `PingAndDetect()` for unknown servos or mixed installations

## Examples

### Single Servo Control with Calibration

```go
func calibratedServoExample() {
    // Create calibration
    cal := feetech.NewMotorCalibration(1)
    cal.RangeMin = 758
    cal.RangeMax = 3292
    cal.HomingOffset = -1470
    cal.NormMode = feetech.NormModeDegrees

    bus, err := feetech.NewBus(feetech.BusConfig{
        Port:     "/dev/ttyUSB0",
        Baudrate: 1000000,
        Protocol: feetech.ProtocolV0,
        Calibrations: map[int]*feetech.MotorCalibration{1: cal},
    })
    if err != nil {
        log.Fatal(err)
    }
    defer bus.Close()

    servo := bus.Servo(1)
    
    // Apply homing offset to servo
    cal.ApplyHomingOffset(servo)
    
    // Set to position control mode
    servo.SetOperatingMode(feetech.OperatingModePosition)
    servo.SetTorqueEnable(true)
    
    // Read current position (normalized to degrees)
    pos, err := servo.ReadPosition(true)
    if err != nil {
        log.Fatal(err)
    }
    log.Printf("Current position: %.1f degrees", pos)
    
    // Move to new position using normalized coordinates
    servo.WritePosition(90.0, true) // 90 degrees
    
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

func NewRobotArm(port string, calibrationFile string) (*RobotArm, error) {
    // Load calibrations
    calibrations, err := feetech.LoadCalibrations(calibrationFile)
    if err != nil {
        return nil, err
    }

    bus, err := feetech.NewBus(feetech.BusConfig{
        Port:         port,
        Baudrate:     1000000,
        Protocol:     feetech.ProtocolV0,
        Calibrations: calibrations,
    })
    if err != nil {
        return nil, err
    }

    var servos []*feetech.Servo
    for id, cal := range calibrations {
        servo := bus.Servo(id)
        
        // Apply homing offset
        if err := cal.ApplyHomingOffset(servo); err != nil {
            log.Printf("Warning: Failed to apply homing offset for servo %d", id)
        }
        
        // Initialize servo
        servo.SetOperatingMode(feetech.OperatingModePosition)
        servo.SetTorqueEnable(true)
        
        servos = append(servos, servo)
    }

    return &RobotArm{bus: bus, servos: servos}, nil
}

func (r *RobotArm) MoveToAngles(angles map[int]float64) error {
    // Move specific joints to target angles (normalized)
    for servoID, angle := range angles {
        for _, servo := range r.servos {
            if servo.ID == servoID {
                if err := servo.WritePosition(angle, true); err != nil {
                    return err
                }
                break
            }
        }
    }
    return nil
}

func (r *RobotArm) GetJointAngles() (map[int]float64, error) {
    // Use sync read for efficient position reading
    return r.bus.SyncReadPositions(r.servos, true)
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
    arm, err := NewRobotArm("/dev/ttyUSB0", "robot_cal.json")
    if err != nil {
        log.Fatal(err)
    }
    defer arm.Close()

    // Move to specific joint configuration (in degrees)
    targetAngles := map[int]float64{
        1: 45.0,  // shoulder_pan to 45°
        2: -30.0, // shoulder_lift to -30°
        3: 90.0,  // elbow_flex to 90°
        6: 75.0,  // gripper to 75% (if using percentage mode)
    }
    
    if err := arm.MoveToAngles(targetAngles); err != nil {
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

    // Create servo and detect model
    servo := bus.Servo(1)
    modelNum, err := servo.PingAndDetect()
    if err != nil {
        log.Fatal(err)
    }
    log.Printf("Detected servo model: %s", servo.Model)
    
    // Read servo status using named registers (model-specific)
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
    
    // Model-specific register access
    switch servo.Model {
    case "sts3215", "sts3250":
        // STS series has goal_time register
        goalTime := []byte{0x00, 0x32} // 50ms in little-endian
        servo.WriteRegisterByName("goal_time", goalTime)
        
    case "scs0009":
        // SCS series has running_time register instead
        runningTime := []byte{0x00, 0x32}
        servo.WriteRegisterByName("running_time", runningTime)
    }
    
    // Set position limits (available on all models)
    minLimit := []byte{0x00, 0x02} // 512 in little-endian
    maxLimit := []byte{0x00, 0x0E} // 3584 in little-endian
    
    servo.WriteRegisterByName("min_position_limit", minLimit)
    servo.WriteRegisterByName("max_position_limit", maxLimit)
    
    // Get model capabilities
    info, err := servo.GetModelInfo()
    if err != nil {
        log.Fatal(err)
    }
    log.Printf("Model resolution: %d", info["resolution"])
    log.Printf("Max position: %d", info["max_position"])
}
```

### Servo Discovery and Setup

The package provides comprehensive servo discovery and setup functionality for initial motor configuration:

```go
func servoDiscoveryExample() {
    bus, err := feetech.NewBus(feetech.BusConfig{
        Port:     "/dev/ttyUSB0",
        Baudrate: 1000000,
        Protocol: feetech.ProtocolV0,
    })
    if err != nil {
        log.Fatal(err)
    }
    defer bus.Close()

    // Method 1: Broadcast discovery (Protocol 0 only)
    // Discovers all servos on the bus at current baudrate
    discovered, err := bus.DiscoverServos()
    if err != nil {
        log.Fatal(err)
    }

    fmt.Printf("Found %d servos:\n", len(discovered))
    for _, servo := range discovered {
        fmt.Printf("  ID: %d, Model: %s (number: %d)\n", 
            servo.ID, servo.ModelName, servo.ModelNumber)
    }

    // Method 2: Sequential ID scanning
    // Useful for Protocol 1 or when broadcast fails
    scanned, err := bus.ScanServoIDs(0, 10) // Scan IDs 0-10
    if err != nil {
        log.Fatal(err)
    }

    // Method 3: Multi-baudrate discovery
    // Scan multiple baudrates to find servos with unknown settings
    baudrates := []int{1000000, 500000, 250000, 115200, 57600, 38400, 19200, 9600}
    servo, baudrate, err := bus.DiscoverServoAtBaudrates(baudrates, "sts3215")
    if err != nil {
        log.Printf("No STS3215 servo found: %v", err)
    } else {
        fmt.Printf("Found STS3215 at ID %d, baudrate %d\n", servo.ID, baudrate)
    }
}
```

### Servo ID and Baudrate Configuration

```go
func servoSetupExample() {
    bus, err := feetech.NewBus(feetech.BusConfig{
        Port:     "/dev/ttyUSB0",
        Baudrate: 57600, // Connect at current servo baudrate
        Protocol: feetech.ProtocolV0,
    })
    if err != nil {
        log.Fatal(err)
    }
    defer bus.Close()

    // Step 1: Discover servo at current settings
    discovered, err := bus.DiscoverServos()
    if err != nil || len(discovered) != 1 {
        log.Fatal("Expected exactly one servo, found:", len(discovered))
    }

    servo := bus.Servo(discovered[0].ID)
    servo.Model = discovered[0].ModelName

    fmt.Printf("Found servo: ID %d, Model %s\n", servo.ID, servo.Model)

    // Step 2: Change servo ID (writes to EEPROM)
    newID := 5
    if err := servo.SetServoID(newID); err != nil {
        log.Fatal("Failed to set servo ID:", err)
    }
    fmt.Printf("✓ Servo ID changed to %d\n", newID)

    // Step 3: Change baudrate to standard rate (writes to EEPROM)
    if err := servo.SetBaudrate(1000000); err != nil {
        log.Fatal("Failed to set baudrate:", err)
    }
    fmt.Printf("✓ Servo baudrate set to 1000000\n")

    // Step 4: Reconnect at new baudrate to verify
    bus.Close()
    
    newBus, err := feetech.NewBus(feetech.BusConfig{
        Port:     "/dev/ttyUSB0",
        Baudrate: 1000000,
        Protocol: feetech.ProtocolV0,
    })
    if err != nil {
        log.Fatal(err)
    }
    defer newBus.Close()

    // Verify the servo responds at new settings
    newServo := newBus.Servo(newID)
    modelNum, err := newServo.Ping()
    if err != nil {
        log.Fatal("Servo not responding at new settings:", err)
    }
    
    fmt.Printf("✓ Servo setup complete - ID: %d, Model: %d\n", newID, modelNum)
}
```

### Systematic Motor Setup Process

For robotics applications requiring multiple servos with specific IDs:

```go
func systematicSetupExample() {
    // Define target motor configuration
    type MotorConfig struct {
        Name     string
        TargetID int
        Model    string
    }

    motors := []MotorConfig{
        {"gripper", 6, "sts3215"},
        {"wrist_roll", 5, "sts3215"},
        {"wrist_flex", 4, "sts3215"},
        {"elbow_flex", 3, "sts3215"},
        {"shoulder_lift", 2, "sts3215"},
        {"shoulder_pan", 1, "sts3215"},
    }

    // Process each motor individually
    for _, motor := range motors {
        fmt.Printf("\n=== Setting up %s (ID: %d) ===\n", motor.Name, motor.TargetID)
        fmt.Printf("Connect only the '%s' motor and press Enter...\n", motor.Name)
        
        // Wait for user to connect the specific motor
        fmt.Scanln()

        // Discover servo at various baudrates
        baudrates := []int{1000000, 500000, 250000, 115200, 57600, 38400, 19200, 9600}
        
        bus, err := feetech.NewBus(feetech.BusConfig{
            Port:     "/dev/ttyUSB0",
            Protocol: feetech.ProtocolV0,
        })
        if err != nil {
            log.Printf("Failed to create bus: %v", err)
            continue
        }

        servo, foundBaudrate, err := bus.DiscoverServoAtBaudrates(baudrates, motor.Model)
        if err != nil {
            log.Printf("❌ Motor %s not found: %v", motor.Name, err)
            bus.Close()
            continue
        }

        fmt.Printf("Found %s at ID %d, baudrate %d\n", motor.Name, servo.ID, foundBaudrate)

        // Reconnect at found baudrate
        bus.Close()
        bus, err = feetech.NewBus(feetech.BusConfig{
            Port:     "/dev/ttyUSB0",
            Baudrate: foundBaudrate,
            Protocol: feetech.ProtocolV0,
        })
        if err != nil {
            log.Printf("Failed to reconnect: %v", err)
            continue
        }

        // Configure the servo
        servoInstance := bus.Servo(servo.ID)
        servoInstance.Model = servo.ModelName

        // Set target ID
        if servo.ID != motor.TargetID {
            if err := servoInstance.SetServoID(motor.TargetID); err != nil {
                log.Printf("❌ Failed to set ID for %s: %v", motor.Name, err)
                bus.Close()
                continue
            }
        }

        // Set standard baudrate
        if err := servoInstance.SetBaudrate(1000000); err != nil {
            log.Printf("❌ Failed to set baudrate for %s: %v", motor.Name, err)
        } else {
            fmt.Printf("✅ %s configured successfully (ID: %d)\n", motor.Name, motor.TargetID)
        }

        bus.Close()
    }

    fmt.Println("\n=== Setup Complete ===")
    fmt.Println("All motors should now be configured with standard settings:")
    fmt.Println("- Baudrate: 1000000")
    fmt.Println("- IDs assigned as specified")
}
```

### Calibration Management

```go
func calibrationExample() {
    // Create calibrations programmatically
    calibrations := map[int]*feetech.MotorCalibration{
        1: {
            ID:           1,
            DriveMode:    0,
            HomingOffset: -1470,
            RangeMin:     758,
            RangeMax:     3292,
            NormMode:     feetech.NormModeDegrees,
        },
        6: {
            ID:           6,
            DriveMode:    0,
            HomingOffset: 1407,
            RangeMin:     2031,
            RangeMax:     3476,
            NormMode:     feetech.NormModeRange100, // 0-100% for gripper
        },
    }
    
    // Define motor names for JSON export
    motorNames := map[int]string{
        1: "shoulder_pan",
        6: "gripper",
    }
    
    // Save to file
    err := feetech.SaveCalibrations("my_robot.json", calibrations, motorNames)
    if err != nil {
        log.Fatal(err)
    }
    
    // Load from file
    loadedCals, err := feetech.LoadCalibrations("my_robot.json")
    if err != nil {
        log.Fatal(err)
    }
    
    // Use with bus
    bus, _ := feetech.NewBus(feetech.BusConfig{
        Port:         "/dev/ttyUSB0",
        Calibrations: loadedCals,
    })
    defer bus.Close()
}
```

## Error Handling

The package provides structured error types for better error handling:

```go
func handleErrors() {
    bus, _ := feetech.NewBus(feetech.BusConfig{Port: "/dev/ttyUSB0"})
    servo := bus.Servo(1)
    
    _, err := servo.ReadPosition(true)
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
        servo1.WritePosition(90.0, true)
    }()
    
    go func() {
        servo2.WritePosition(-45.0, true)
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
- Include calibration data when testing with physical hardware

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Inspired by the Python [lerobot](https://github.com/huggingface/lerobot) FeetechMotorsBus implementation
- Feetech for their servo motor documentation and protocol specifications
