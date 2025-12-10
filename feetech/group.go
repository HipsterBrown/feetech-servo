package feetech

import (
	"context"
	"fmt"
	"time"
)

// ServoGroup manages coordinated operations across multiple servos.
type ServoGroup struct {
	bus    *Bus
	servos []*Servo
	ids    []int
}

// NewServoGroup creates a new group from the given servos.
func NewServoGroup(bus *Bus, servos ...*Servo) *ServoGroup {
	ids := make([]int, len(servos))
	for i, s := range servos {
		ids[i] = s.ID()
	}
	return &ServoGroup{
		bus:    bus,
		servos: servos,
		ids:    ids,
	}
}

// NewServoGroupByIDs creates servos with the given IDs and groups them.
// All servos default to the STS3215 model.
func NewServoGroupByIDs(bus *Bus, ids ...int) *ServoGroup {
	servos := make([]*Servo, len(ids))
	for i, id := range ids {
		servos[i] = NewServo(bus, id, nil)
	}
	return NewServoGroup(bus, servos...)
}

// Servos returns the servos in this group.
func (g *ServoGroup) Servos() []*Servo {
	return g.servos
}

// IDs returns the servo IDs in this group.
func (g *ServoGroup) IDs() []int {
	return g.ids
}

// Servo returns the servo at the given index.
func (g *ServoGroup) Servo(index int) *Servo {
	if index < 0 || index >= len(g.servos) {
		return nil
	}
	return g.servos[index]
}

// ServoByID returns the servo with the given ID, or nil if not found.
func (g *ServoGroup) ServoByID(id int) *Servo {
	for _, s := range g.servos {
		if s.ID() == id {
			return s
		}
	}
	return nil
}

// Positions reads positions from all servos using sync read.
// Returns a slice in the same order as the group's servos.
func (g *ServoGroup) Positions(ctx context.Context) ([]int, error) {
	data, err := g.bus.SyncRead(ctx, RegPresentPosition.Address, RegPresentPosition.Size, g.ids)
	if err != nil {
		return nil, err
	}

	proto := g.bus.Protocol()
	positions := make([]int, len(g.servos))
	for i, s := range g.servos {
		if d, ok := data[s.ID()]; ok {
			positions[i] = int(proto.DecodeWord(d))
		}
	}

	return positions, nil
}

// SetPositions writes positions to all servos using sync write.
// positions must have the same length as the group.
func (g *ServoGroup) SetPositions(ctx context.Context, positions []int) error {
	if len(positions) != len(g.servos) {
		return fmt.Errorf("position count mismatch: expected %d, got %d", len(g.servos), len(positions))
	}

	proto := g.bus.Protocol()
	servoData := make(map[int][]byte, len(g.servos))
	for i, s := range g.servos {
		servoData[s.ID()] = proto.EncodeWord(uint16(positions[i]))
	}

	return g.bus.SyncWrite(ctx, RegGoalPosition.Address, 2, servoData)
}

// SetPositionsWithSpeed writes positions with speed to all servos.
func (g *ServoGroup) SetPositionsWithSpeed(ctx context.Context, positions, speeds []int) error {
	if len(positions) != len(g.servos) || len(speeds) != len(g.servos) {
		return fmt.Errorf("array length mismatch")
	}

	proto := g.bus.Protocol()
	servoData := make(map[int][]byte, len(g.servos))

	for i, s := range g.servos {
		data := make([]byte, 6)
		copy(data[0:2], proto.EncodeWord(uint16(positions[i])))
		copy(data[2:4], proto.EncodeWord(0)) // Time = 0
		copy(data[4:6], proto.EncodeWord(uint16(speeds[i])))
		servoData[s.ID()] = data
	}

	return g.bus.SyncWrite(ctx, RegGoalPosition.Address, 6, servoData)
}

// SetPositionsWithTime writes positions with time to all servos.
func (g *ServoGroup) SetPositionsWithTime(ctx context.Context, positions []int, timeMs int) error {
	if len(positions) != len(g.servos) {
		return fmt.Errorf("position count mismatch: expected %d, got %d", len(g.servos), len(positions))
	}

	proto := g.bus.Protocol()
	servoData := make(map[int][]byte, len(g.servos))

	for i, s := range g.servos {
		data := make([]byte, 6)
		copy(data[0:2], proto.EncodeWord(uint16(positions[i])))
		copy(data[2:4], proto.EncodeWord(uint16(timeMs)))
		copy(data[4:6], proto.EncodeWord(0)) // Speed = 0
		servoData[s.ID()] = data
	}

	return g.bus.SyncWrite(ctx, RegGoalPosition.Address, 6, servoData)
}

// EnableAll enables torque on all servos.
func (g *ServoGroup) EnableAll(ctx context.Context) error {
	servoData := make(map[int][]byte, len(g.servos))
	for _, s := range g.servos {
		servoData[s.ID()] = []byte{1}
	}
	return g.bus.SyncWrite(ctx, RegTorqueEnable.Address, 1, servoData)
}

// DisableAll disables torque on all servos.
func (g *ServoGroup) DisableAll(ctx context.Context) error {
	servoData := make(map[int][]byte, len(g.servos))
	for _, s := range g.servos {
		servoData[s.ID()] = []byte{0}
	}
	return g.bus.SyncWrite(ctx, RegTorqueEnable.Address, 1, servoData)
}

// PositionMap is a map of servo ID to position value.
type PositionMap map[int]int

// PositionsMap reads positions and returns them as a map.
func (g *ServoGroup) PositionsMap(ctx context.Context) (PositionMap, error) {
	data, err := g.bus.SyncRead(ctx, RegPresentPosition.Address, RegPresentPosition.Size, g.ids)
	if err != nil {
		return nil, err
	}

	proto := g.bus.Protocol()
	result := make(PositionMap, len(data))
	for id, d := range data {
		result[id] = int(proto.DecodeWord(d))
	}

	return result, nil
}

// SetPositionsMap writes positions from a map.
// Only servos in the group with matching IDs in the map are written.
func (g *ServoGroup) SetPositionsMap(ctx context.Context, positions PositionMap) error {
	proto := g.bus.Protocol()
	servoData := make(map[int][]byte)

	for _, s := range g.servos {
		if pos, ok := positions[s.ID()]; ok {
			servoData[s.ID()] = proto.EncodeWord(uint16(pos))
		}
	}

	if len(servoData) == 0 {
		return nil
	}

	return g.bus.SyncWrite(ctx, RegGoalPosition.Address, 2, servoData)
}

// RegWritePositions buffers position writes to all servos.
// Call bus.Action() to execute them simultaneously.
func (g *ServoGroup) RegWritePositions(ctx context.Context, positions []int) error {
	if len(positions) != len(g.servos) {
		return fmt.Errorf("position count mismatch: expected %d, got %d", len(g.servos), len(positions))
	}

	proto := g.bus.Protocol()
	for i, s := range g.servos {
		data := proto.EncodeWord(uint16(positions[i]))
		if err := g.bus.RegWrite(ctx, s.ID(), RegGoalPosition.Address, data); err != nil {
			return fmt.Errorf("servo %d: %w", s.ID(), err)
		}
	}

	return nil
}

// MoveResult contains the result of a move operation.
type MoveResult struct {
	Positions []int
	Err       error
}

// MoveTo moves all servos to target positions and waits for completion.
// Returns the final positions. Timeout is in milliseconds.
func (g *ServoGroup) MoveTo(ctx context.Context, positions []int, timeoutMs int) ([]int, error) {
	if err := g.SetPositions(ctx, positions); err != nil {
		return nil, err
	}

	return g.WaitForStop(ctx, timeoutMs)
}

// WaitForStop waits for all servos to stop moving.
// Returns the final positions. Timeout is in milliseconds.
func (g *ServoGroup) WaitForStop(ctx context.Context, timeoutMs int) ([]int, error) {
	ticker := time.NewTicker(50 * time.Millisecond)
	defer ticker.Stop()

	timeout := time.After(time.Duration(timeoutMs) * time.Millisecond)

	for {
		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		case <-timeout:
			pos, _ := g.Positions(ctx)
			return pos, fmt.Errorf("move timeout after %dms", timeoutMs)
		case <-ticker.C:
			allStopped := true
			for _, s := range g.servos {
				moving, err := s.Moving(ctx)
				if err != nil {
					continue
				}
				if moving {
					allStopped = false
					break
				}
			}

			if allStopped {
				return g.Positions(ctx)
			}
		}
	}
}
