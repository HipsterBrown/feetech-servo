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
// Returns a map of servo ID to position value.
func (g *ServoGroup) Positions(ctx context.Context) (PositionMap, error) {
	data, err := g.bus.SyncRead(ctx, RegPresentPosition.Address, RegPresentPosition.Size, g.ids)
	if err != nil {
		return nil, err
	}

	proto := g.bus.Protocol()
	positions := make(PositionMap, len(data))
	for id, d := range data {
		positions[id] = int(proto.DecodeWord(d))
	}

	return positions, nil
}

// SetPositions writes positions to servos using sync write.
// Only servos with IDs present in the positions map are written.
func (g *ServoGroup) SetPositions(ctx context.Context, positions PositionMap) error {
	if len(positions) == 0 {
		return nil // No-op for empty map
	}

	proto := g.bus.Protocol()
	servoData := make(map[int][]byte, len(positions))

	for id, pos := range positions {
		// Validate servo ID exists in group
		if g.ServoByID(id) == nil {
			return fmt.Errorf("servo ID %d not in group", id)
		}
		servoData[id] = proto.EncodeWord(uint16(pos))
	}

	return g.bus.SyncWrite(ctx, RegGoalPosition.Address, 2, servoData)
}

// SetPositionsWithSpeed writes positions with speed to servos.
// Only servos present in BOTH positions and speeds maps are written (intersection).
func (g *ServoGroup) SetPositionsWithSpeed(ctx context.Context, positions, speeds PositionMap) error {
	if len(positions) == 0 || len(speeds) == 0 {
		return nil // No-op for empty maps
	}

	proto := g.bus.Protocol()
	servoData := make(map[int][]byte)

	// Use intersection: only write servos present in both maps
	for id, pos := range positions {
		speed, hasSpeed := speeds[id]
		if !hasSpeed {
			continue // Skip servos without speed
		}

		// Validate servo ID exists in group
		if g.ServoByID(id) == nil {
			return fmt.Errorf("servo ID %d not in group", id)
		}

		data := make([]byte, 6)
		copy(data[0:2], proto.EncodeWord(uint16(pos)))
		copy(data[2:4], proto.EncodeWord(0)) // Time = 0
		copy(data[4:6], proto.EncodeWord(uint16(speed)))
		servoData[id] = data
	}

	if len(servoData) == 0 {
		return nil // No servos in intersection
	}

	return g.bus.SyncWrite(ctx, RegGoalPosition.Address, 6, servoData)
}

// SetPositionsWithTime writes positions with time to servos.
// Only servos present in BOTH positions and times maps are written (intersection).
func (g *ServoGroup) SetPositionsWithTime(ctx context.Context, positions, times PositionMap) error {
	if len(positions) == 0 || len(times) == 0 {
		return nil // No-op for empty maps
	}

	proto := g.bus.Protocol()
	servoData := make(map[int][]byte)

	// Use intersection: only write servos present in both maps
	for id, pos := range positions {
		timeMs, hasTime := times[id]
		if !hasTime {
			continue // Skip servos without time
		}

		// Validate servo ID exists in group
		if g.ServoByID(id) == nil {
			return fmt.Errorf("servo ID %d not in group", id)
		}

		data := make([]byte, 6)
		copy(data[0:2], proto.EncodeWord(uint16(pos)))
		copy(data[2:4], proto.EncodeWord(uint16(timeMs)))
		copy(data[4:6], proto.EncodeWord(0)) // Speed = 0
		servoData[id] = data
	}

	if len(servoData) == 0 {
		return nil // No servos in intersection
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

// RegWritePositions buffers position writes to servos.
// Call bus.Action() to execute them simultaneously.
// Only servos with IDs present in the positions map are written.
func (g *ServoGroup) RegWritePositions(ctx context.Context, positions PositionMap) error {
	if len(positions) == 0 {
		return nil // No-op for empty map
	}

	proto := g.bus.Protocol()
	for id, pos := range positions {
		// Validate servo ID exists in group
		servo := g.ServoByID(id)
		if servo == nil {
			return fmt.Errorf("servo ID %d not in group", id)
		}

		data := proto.EncodeWord(uint16(pos))
		if err := g.bus.RegWrite(ctx, id, RegGoalPosition.Address, data); err != nil {
			return fmt.Errorf("servo %d: %w", id, err)
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
// TODO(Task 6): This will be updated to use PositionMap in Task 6
func (g *ServoGroup) MoveTo(ctx context.Context, positions []int, timeoutMs int) ([]int, error) {
	// Temporary conversion to PositionMap for compatibility
	posMap := make(PositionMap, len(positions))
	for i, pos := range positions {
		if i >= len(g.servos) {
			break
		}
		posMap[g.servos[i].ID()] = pos
	}

	if err := g.SetPositions(ctx, posMap); err != nil {
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
			posMap, _ := g.Positions(ctx)
			positions := make([]int, len(g.servos))
			for i, s := range g.servos {
				positions[i] = posMap[s.ID()]
			}
			return positions, fmt.Errorf("move timeout after %dms", timeoutMs)
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
				posMap, err := g.Positions(ctx)
				if err != nil {
					return nil, err
				}
				positions := make([]int, len(g.servos))
				for i, s := range g.servos {
					positions[i] = posMap[s.ID()]
				}
				return positions, nil
			}
		}
	}
}
