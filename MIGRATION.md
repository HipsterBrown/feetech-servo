# Migration Guide: feetech v1 → v2

This guide helps you migrate from the old top-level `feetech` package to the new `feetech/feetech` subpackage API.

## Overview

The v2 rewrite provides significant improvements:
- **Context support** for proper cancellation and timeouts
- **Transport abstraction** for easier testing with MockTransport
- **Protocol abstraction** for cleaner byte ordering handling
- **Cleaner API** with consistent naming and better error handling
- **ServoGroup** for coordinated multi-servo operations

**Breaking Changes:**
- All servo operations now require `context.Context`
- Calibration system removed (migrate to application layer)
- Some method names changed for consistency
- API moved to `feetech` subpackage

---

## Quick Start

### Package Import Change

```go
// OLD
import "github.com/hipsterbrown/feetech-servo"

// NEW
import "github.com/hipsterbrown/feetech-servo/feetech"
```

### Basic Usage Comparison

```go
// OLD
bus, err := feetech.NewBus(feetech.BusConfig{
    Port:     "/dev/ttyUSB0",
    Baudrate: 1000000,
    Protocol: feetech.ProtocolV0,
})
servo := bus.Servo(1)
pos, err := servo.ReadPosition(false)

// NEW
ctx := context.Background()
bus, err := feetech.NewBus(feetech.BusConfig{
    Port:     "/dev/ttyUSB0",
    BaudRate: 1000000, // Note: BaudRate not Baudrate
    Protocol: feetech.ProtocolSTS,
})
servo := feetech.NewServo(bus, 1, nil)
pos, err := servo.Position(ctx)
```

---

## API Migration Reference

### Bus Creation

| Old API | New API | Notes |
|---------|---------|-------|
| `BusConfig.Baudrate` | `BusConfig.BaudRate` | Capital R |
| `ProtocolV0` | `ProtocolSTS` | Renamed for clarity |
| `ProtocolV1` | `ProtocolSCS` | Renamed for clarity |
| `NewBus(config)` | `NewBus(config)` | Same |

**Example:**

```go
// OLD
config := feetech.BusConfig{
    Port:         "/dev/ttyUSB0",
    Baudrate:     1000000,
    Protocol:     feetech.ProtocolV0,
    Timeout:      time.Second,
    Calibrations: calibMap, // REMOVED
}

// NEW
config := feetech.BusConfig{
    Port:     "/dev/ttyUSB0",
    BaudRate: 1000000,
    Protocol: feetech.ProtocolSTS,
    Timeout:  time.Second,
    // Calibrations removed - implement in your application
}
```

### Servo Creation

| Old API | New API | Notes |
|---------|---------|-------|
| `bus.Servo(id)` | `feetech.NewServo(bus, id, nil)` | Constructor function |
| `bus.ServoWithModel(id, "sts3215")` | `feetech.NewServo(bus, id, &feetech.ModelSTS3215)` | Use Model pointer |

**Example:**

```go
// OLD
servo := bus.Servo(1)
servo.Model = "sts3215"

// NEW
servo := feetech.NewServo(bus, 1, &feetech.ModelSTS3215)
// Or let it default to STS3215:
servo := feetech.NewServo(bus, 1, nil)
```

### Position Control

| Old API | New API | Notes |
|---------|---------|-------|
| `ReadPosition(normalize)` | `Position(ctx)` | Removed normalization |
| `WritePosition(pos, normalize)` | `SetPosition(ctx, pos)` | Removed normalization |
| N/A | `SetPositionWithSpeed(ctx, pos, speed)` | New feature |
| N/A | `SetPositionWithTime(ctx, pos, timeMs)` | New feature |

**Example:**

```go
// OLD
pos, err := servo.ReadPosition(false)
err = servo.WritePosition(2048, false)

// NEW
ctx := context.Background()
pos, err := servo.Position(ctx)
err = servo.SetPosition(ctx, 2048)

// NEW: Advanced control
err = servo.SetPositionWithSpeed(ctx, 2048, 100)
err = servo.SetPositionWithTime(ctx, 2048, 1000) // 1000ms
```

### Velocity Control

| Old API | New API | Notes |
|---------|---------|-------|
| `ReadVelocity(normalize)` | `Velocity(ctx)` | Removed normalization |
| `WriteVelocity(vel, normalize)` | `SetVelocity(ctx, vel)` | Removed normalization |

**Example:**

```go
// OLD
vel, err := servo.ReadVelocity(false)
err = servo.WriteVelocity(100, false)

// NEW
ctx := context.Background()
vel, err := servo.Velocity(ctx)
err = servo.SetVelocity(ctx, 100)
```

### Torque Control

| Old API | New API | Notes |
|---------|---------|-------|
| `SetTorqueEnable(enable)` | `SetTorqueEnabled(ctx, enable)` | Added 'd' |
| N/A | `Enable(ctx)` | Convenience method |
| N/A | `Disable(ctx)` | Convenience method |
| N/A | `TorqueEnabled(ctx)` | Read torque state |

**Example:**

```go
// OLD
err := servo.SetTorqueEnable(true)

// NEW
ctx := context.Background()
err := servo.Enable(ctx)
// Or:
err = servo.SetTorqueEnabled(ctx, true)

// NEW: Check state
enabled, err := servo.TorqueEnabled(ctx)
```

### Status Reading

| Old API | New API | Notes |
|---------|---------|-------|
| `IsMoving()` | `Moving(ctx)` | Renamed |
| Via registers | `Load(ctx)` | Dedicated method |
| Via registers | `Voltage(ctx)` | Dedicated method |
| Via registers | `Temperature(ctx)` | Dedicated method |

**Example:**

```go
// OLD
moving, err := servo.IsMoving()

// NEW
ctx := context.Background()
moving, err := servo.Moving(ctx)
voltage, err := servo.Voltage(ctx)    // Returns tenths of volt
temp, err := servo.Temperature(ctx)   // Returns degrees C
load, err := servo.Load(ctx)          // Returns signed load
```

### Operating Mode

| Old API | New API | Notes |
|---------|---------|-------|
| `SetOperatingMode(mode)` | `SetOperatingMode(ctx, mode)` | Added context |
| `GetOperatingMode()` | `OperatingMode(ctx)` | Renamed |
| `OperatingModePosition` | `feetech.ModePosition` | Package-level constant |
| `OperatingModeVelocity` | `feetech.ModeVelocity` | Package-level constant |

**Example:**

```go
// OLD
err := servo.SetOperatingMode(feetech.OperatingModeVelocity)
mode, err := servo.GetOperatingMode()

// NEW
ctx := context.Background()
err := servo.SetOperatingMode(ctx, feetech.ModeVelocity)
mode, err := servo.OperatingMode(ctx)
```

### Model Detection

| Old API | New API | Notes |
|---------|---------|-------|
| `Ping()` | `Ping(ctx)` | Added context |
| `PingAndDetect()` | `Ping(ctx)` + `DetectModel(ctx)` | Split into two methods |
| `DetectModel()` | `DetectModel(ctx)` | Added context |

**Example:**

```go
// OLD
modelNum, err := servo.PingAndDetect()
fmt.Printf("Model: %s\n", servo.Model)

// NEW
ctx := context.Background()
modelNum, err := servo.Ping(ctx)
err = servo.DetectModel(ctx)
fmt.Printf("Model: %s\n", servo.Model().Name)
```

### Discovery

| Old API | New API | Notes |
|---------|---------|-------|
| `DiscoverServos()` | `Discover(ctx)` | Renamed, added context |
| `ScanServoIDs(start, end)` | `Scan(ctx, start, end)` | Added context |
| `DiscoveredServo` | `FoundServo` | Renamed |

**Example:**

```go
// OLD
servos, err := bus.DiscoverServos()
for _, s := range servos {
    fmt.Printf("ID: %d, Model: %s\n", s.ID, s.ModelName)
}

// NEW
ctx := context.Background()
servos, err := bus.Discover(ctx)
for _, s := range servos {
    fmt.Printf("ID: %d, Model: %s\n", s.ID, s.Model.Name)
}
```

### Register Access

| Old API | New API | Notes |
|---------|---------|-------|
| `ReadRegisterByName(name)` | `ReadRegister(ctx, name)` | Simplified |
| `WriteRegisterByName(name, data)` | `WriteRegister(ctx, name, data)` | Simplified |

**Example:**

```go
// OLD
data, err := servo.ReadRegisterByName("present_voltage")

// NEW
ctx := context.Background()
data, err := servo.ReadRegister(ctx, "present_voltage")
```

### Configuration (EEPROM)

| Old API | New API | Notes |
|---------|---------|-------|
| `SetServoID(newID)` | `SetID(ctx, newID)` | Renamed |
| `SetBaudrate(baudrate)` | `SetBaudRate(ctx, baudrate)` | Capital R |
| N/A | `SetPositionLimits(ctx, min, max)` | New method |
| N/A | `PositionLimits(ctx)` | New method |

**Example:**

```go
// OLD
err := servo.SetServoID(5)
err = servo.SetBaudrate(1000000)

// NEW
ctx := context.Background()
err := servo.SetID(ctx, 5)
err = servo.SetBaudRate(ctx, 1000000)

// NEW: Position limits
min, max, err := servo.PositionLimits(ctx)
err = servo.SetPositionLimits(ctx, 512, 3584)
```

---

## Multi-Servo Operations

### Sync Read/Write

The new API provides enhanced sync operations through the `ServoGroup` type.

**OLD: Basic sync operations**

```go
servos := []*feetech.Servo{servo1, servo2, servo3}
positions := []float64{1000, 2000, 3000}
err := bus.SyncWritePositions(servos, positions, false)

// Sync read was complex
posMap, err := bus.SyncReadPositions(servos, false)
```

**NEW: ServoGroup**

```go
ctx := context.Background()
group := feetech.NewServoGroup(bus, servo1, servo2, servo3)

// Write positions
positions := []int{1000, 2000, 3000}
err := group.SetPositions(ctx, positions)

// Read positions (returns slice in group order)
positions, err := group.Positions(ctx)

// Or use map-based operations
posMap, err := group.PositionsMap(ctx)
```

**NEW: Advanced group operations**

```go
// Coordinated move with speed
positions := []int{1000, 2000, 3000}
speeds := []int{100, 150, 200}
err := group.SetPositionsWithSpeed(ctx, positions, speeds)

// Coordinated move with time
err := group.SetPositionsWithTime(ctx, positions, 1000) // All reach in 1000ms

// Enable/disable all
err := group.EnableAll(ctx)
err := group.DisableAll(ctx)

// Wait for movement to complete
finalPositions, err := group.MoveTo(ctx, positions, 5000) // 5s timeout
```

---

## Calibration Migration

**The calibration system has been removed from the v2 API.** You should implement calibration in your application layer.

### Migration Strategy

**Option 1: Keep using old package for calibration**

```go
// Use old package just for loading calibration data
import oldFeetech "github.com/hipsterbrown/feetech-servo"

// Load calibrations
cals, err := oldFeetech.LoadCalibrations("robot.json")

// Implement normalization in your application
type CalibratedServo struct {
    servo *feetech.Servo
    cal   *oldFeetech.MotorCalibration
}

func (cs *CalibratedServo) SetNormalizedPosition(ctx context.Context, normalized float64) error {
    raw, err := cs.cal.Denormalize(normalized)
    if err != nil {
        return err
    }
    return cs.servo.SetPosition(ctx, raw)
}
```

**Option 2: Implement application-level calibration**

```go
type ServoCalibration struct {
    ID       int
    RangeMin int
    RangeMax int
    Invert   bool
}

func (c *ServoCalibration) Normalize(raw int) float64 {
    // Implement your normalization logic
    rangeSpan := float64(c.RangeMax - c.RangeMin)
    normalized := float64(raw-c.RangeMin) / rangeSpan
    if c.Invert {
        normalized = 1.0 - normalized
    }
    return normalized * 360.0 - 180.0 // Convert to degrees
}

func (c *ServoCalibration) Denormalize(degrees float64) int {
    // Implement your denormalization logic
    normalized := (degrees + 180.0) / 360.0
    if c.Invert {
        normalized = 1.0 - normalized
    }
    rangeSpan := float64(c.RangeMax - c.RangeMin)
    return c.RangeMin + int(normalized*rangeSpan)
}
```

**Option 3: Store raw values in your application**

The simplest approach is to work directly with raw servo values (0-4095) in your application and skip normalization entirely.

---

## Context Usage Patterns

All operations now require a `context.Context` for proper cancellation and timeout handling.

### Basic Context

```go
ctx := context.Background()
pos, err := servo.Position(ctx)
```

### Context with Timeout

```go
ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
defer cancel()

pos, err := servo.Position(ctx)
```

### Context with Cancellation

```go
ctx, cancel := context.WithCancel(context.Background())
defer cancel()

// In another goroutine
go func() {
    time.Sleep(5 * time.Second)
    cancel() // Cancel after 5 seconds
}()

pos, err := servo.Position(ctx)
if errors.Is(err, context.Canceled) {
    fmt.Println("Operation was cancelled")
}
```

---

## Testing with MockTransport

The new API makes testing much easier with the `MockTransport`.

```go
func TestServoControl(t *testing.T) {
    // Create mock transport with pre-loaded responses
    mock := &feetech.MockTransport{
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

    // Verify what was written
    if len(mock.WriteData) == 0 {
        t.Error("no data was written")
    }
}
```

---

## Common Migration Patterns

### Pattern 1: Simple Position Control

```go
// OLD
servo := bus.Servo(1)
servo.SetTorqueEnable(true)
servo.WritePosition(2048, false)
pos, _ := servo.ReadPosition(false)

// NEW
ctx := context.Background()
servo := feetech.NewServo(bus, 1, nil)
servo.Enable(ctx)
servo.SetPosition(ctx, 2048)
pos, _ := servo.Position(ctx)
```

### Pattern 2: Discover and Control

```go
// OLD
servos, err := bus.DiscoverServos()
for _, s := range servos {
    servo := bus.Servo(s.ID)
    servo.Model = s.ModelName
    servo.SetTorqueEnable(true)
}

// NEW
ctx := context.Background()
servos, err := bus.Discover(ctx)
for _, s := range servos {
    servo := feetech.NewServo(bus, s.ID, s.Model)
    servo.Enable(ctx)
}
```

### Pattern 3: Robot Arm Control

```go
// OLD
type Arm struct {
    bus    *feetech.Bus
    servos []*feetech.Servo
}

func (a *Arm) MoveJoints(positions []float64) error {
    return a.bus.SyncWritePositions(a.servos, positions, true)
}

// NEW
type Arm struct {
    group *feetech.ServoGroup
}

func (a *Arm) MoveJoints(ctx context.Context, positions []int) error {
    return a.group.SetPositions(ctx, positions)
}

func (a *Arm) MoveJointsAndWait(ctx context.Context, positions []int) error {
    _, err := a.group.MoveTo(ctx, positions, 5000)
    return err
}
```

---

## Migration Checklist

- [ ] Update import path to `feetech` subpackage
- [ ] Add `context.Context` to all servo operations
- [ ] Change `Baudrate` to `BaudRate` in BusConfig
- [ ] Change `ProtocolV0`/`ProtocolV1` to `ProtocolSTS`/`ProtocolSCS`
- [ ] Update servo creation from `bus.Servo()` to `feetech.NewServo()`
- [ ] Rename methods: `ReadPosition` → `Position`, `WritePosition` → `SetPosition`
- [ ] Rename methods: `IsMoving` → `Moving`, `SetTorqueEnable` → `SetTorqueEnabled`
- [ ] Remove `normalize` parameter from all position/velocity operations
- [ ] Update discovery: `DiscoverServos` → `Discover`, `ScanServoIDs` → `Scan`
- [ ] Update register access: `ReadRegisterByName` → `ReadRegister`
- [ ] Update EEPROM methods: `SetServoID` → `SetID`, `SetBaudrate` → `SetBaudRate`
- [ ] Migrate calibration to application layer
- [ ] Consider using `ServoGroup` for multi-servo operations
- [ ] Add proper error handling for context cancellation
- [ ] Update tests to use `MockTransport`

---

## Getting Help

If you encounter issues during migration:

1. Check the [examples directory](../_examples/) for updated usage patterns
2. Review the [CLAUDE.md](../CLAUDE.md) file for architecture details
3. See the [README.md](../README.md) for API documentation
4. Open an issue on GitHub with your migration question

---

## Appendix: Complete Example Migration

### Before (Old API)

```go
package main

import (
    "fmt"
    "log"
    "time"

    "github.com/hipsterbrown/feetech-servo"
)

func main() {
    // Create bus
    bus, err := feetech.NewBus(feetech.BusConfig{
        Port:     "/dev/ttyUSB0",
        Baudrate: 1000000,
        Protocol: feetech.ProtocolV0,
    })
    if err != nil {
        log.Fatal(err)
    }
    defer bus.Close()

    // Discover servos
    servos, err := bus.DiscoverServos()
    if err != nil {
        log.Fatal(err)
    }

    fmt.Printf("Found %d servos\n", len(servos))

    // Control first servo
    servo := bus.Servo(servos[0].ID)
    servo.Model = servos[0].ModelName

    servo.SetTorqueEnable(true)
    servo.WritePosition(2048, false)

    time.Sleep(time.Second)

    pos, _ := servo.ReadPosition(false)
    moving, _ := servo.IsMoving()

    fmt.Printf("Position: %.0f, Moving: %v\n", pos, moving)
}
```

### After (New API)

```go
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

    // Create bus
    bus, err := feetech.NewBus(feetech.BusConfig{
        Port:     "/dev/ttyUSB0",
        BaudRate: 1000000,
        Protocol: feetech.ProtocolSTS,
    })
    if err != nil {
        log.Fatal(err)
    }
    defer bus.Close()

    // Discover servos
    servos, err := bus.Discover(ctx)
    if err != nil {
        log.Fatal(err)
    }

    fmt.Printf("Found %d servos\n", len(servos))

    // Control first servo
    servo := feetech.NewServo(bus, servos[0].ID, servos[0].Model)

    servo.Enable(ctx)
    servo.SetPosition(ctx, 2048)

    time.Sleep(time.Second)

    pos, _ := servo.Position(ctx)
    moving, _ := servo.Moving(ctx)

    fmt.Printf("Position: %d, Moving: %v\n", pos, moving)
}
```

---

**Version:** 2.0.0
**Last Updated:** December 2025
