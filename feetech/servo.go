package feetech

import (
	"cmp"
	"context"
	"errors"
	"fmt"
)

// Servo provides a high-level interface for controlling a single servo.
type Servo struct {
	bus   *Bus
	id    int
	model *Model
}

// NewServo creates a new Servo instance.
// If model is nil, defaults to STS3215.
func NewServo(bus *Bus, id int, model *Model) *Servo {
	if model == nil {
		model = &ModelSTS3215
	}
	return &Servo{
		bus:   bus,
		id:    id,
		model: model,
	}
}

// ID returns the servo's ID.
func (s *Servo) ID() int {
	return s.id
}

// Model returns the servo's model specification.
func (s *Servo) Model() *Model {
	return s.model
}

// SetModel changes the servo's model.
func (s *Servo) SetModel(model *Model) {
	s.model = model
}

// Ping verifies communication with the servo and returns the model number.
func (s *Servo) Ping(ctx context.Context) (int, error) {
	return s.bus.Ping(ctx, s.id)
}

// DetectModel pings the servo and sets the model based on the returned model number.
func (s *Servo) DetectModel(ctx context.Context) error {
	modelNum, err := s.bus.Ping(ctx, s.id)
	if err != nil {
		return err
	}

	if model, ok := GetModelByNumber(modelNum); ok {
		s.model = model
	} else {
		return fmt.Errorf("unknown model number: %d", modelNum)
	}

	return nil
}

// Position Control

// Position reads the current position.
func (s *Servo) Position(ctx context.Context) (int, error) {
	data, err := s.bus.ReadRegister(ctx, s.id, RegPresentPosition.Address, int(RegPresentPosition.Size))
	if len(data) == 0 {
		return 0, err
	}
	// err may be a non-nil condition flag here; the value is still valid.
	return decodePositionWord(s.bus.Protocol(), data), err
}

// SetPosition commands the servo to move to the specified position.
// Negative positions are encoded as sign-magnitude (STS multi-turn).
func (s *Servo) SetPosition(ctx context.Context, position int) error {
	data, err := encodePositionWord(s.bus.Protocol(), position)
	if err != nil {
		return err
	}
	return s.bus.WriteRegister(ctx, s.id, RegGoalPosition.Address, data)
}

// SetPositionWithSpeed commands the servo to move to position at the specified speed.
// Speed is in steps per second.
func (s *Servo) SetPositionWithSpeed(ctx context.Context, position, speed int) error {
	proto := s.bus.Protocol()

	// Write position and velocity together (6 bytes starting at goal position)
	// Format: position(2) + time(2) + velocity(2)
	posBytes, err := encodePositionWord(proto, position)
	if err != nil {
		return err
	}
	data := make([]byte, 6)
	copy(data[0:2], posBytes)
	copy(data[2:4], proto.EncodeWord(0)) // Time = 0 (use speed instead)
	copy(data[4:6], proto.EncodeWord(uint16(speed)))

	return s.bus.WriteRegister(ctx, s.id, RegGoalPosition.Address, data)
}

// SetPositionWithTime commands the servo to reach position in the specified time.
// Time is in milliseconds.
func (s *Servo) SetPositionWithTime(ctx context.Context, position, timeMs int) error {
	proto := s.bus.Protocol()

	posBytes, err := encodePositionWord(proto, position)
	if err != nil {
		return err
	}
	data := make([]byte, 6)
	copy(data[0:2], posBytes)
	copy(data[2:4], proto.EncodeWord(uint16(timeMs)))
	copy(data[4:6], proto.EncodeWord(0)) // Speed = 0 (use time instead)

	return s.bus.WriteRegister(ctx, s.id, RegGoalPosition.Address, data)
}

// GoalRequest is a full position-move command: target position plus the motion
// profile (speed, time, acceleration). Unlike SetPositionWithSpeed/Time it also
// sets the acceleration register, matching the Python SDK's WritePosEx.
//
// Speed and Time are alternatives — set one and leave the other 0; the firmware
// uses Time when non-zero, otherwise Speed. Acc of 0 means an unlimited ramp.
type GoalRequest struct {
	Position int // goal position (sign-magnitude on STS multi-turn)
	Speed    int // goal speed in steps/s (unsigned); 0 = use Time / max
	Time     int // move time in ms (unsigned); 0 = use Speed
	Acc      int // acceleration, ~100 steps/s^2 per unit, 0-255; 0 = unlimited
}

// encodeGoal builds the 7-byte payload written from the acceleration register:
// [acc, pos_lo, pos_hi, time_lo, time_hi, speed_lo, speed_hi].
func encodeGoal(proto *Protocol, g GoalRequest) ([]byte, error) {
	if g.Acc < 0 || g.Acc > 0xFF {
		return nil, fmt.Errorf("acceleration %d out of range [0, 255]", g.Acc)
	}
	if g.Speed < 0 || g.Speed > 0xFFFF {
		return nil, fmt.Errorf("speed %d out of range [0, 65535]", g.Speed)
	}
	if g.Time < 0 || g.Time > 0xFFFF {
		return nil, fmt.Errorf("time %d out of range [0, 65535]", g.Time)
	}
	posBytes, err := encodePositionWord(proto, g.Position)
	if err != nil {
		return nil, err
	}
	data := make([]byte, 7)
	data[0] = byte(g.Acc)
	copy(data[1:3], posBytes)
	copy(data[3:5], proto.EncodeWord(uint16(g.Time)))
	copy(data[5:7], proto.EncodeWord(uint16(g.Speed)))
	return data, nil
}

// SetGoal commands a position move with full motion-profile control
// (acceleration, speed/time) in a single write starting at the acceleration
// register. For basic moves prefer SetPosition / SetPositionWithSpeed.
func (s *Servo) SetGoal(ctx context.Context, g GoalRequest) error {
	data, err := encodeGoal(s.bus.Protocol(), g)
	if err != nil {
		return err
	}
	return s.bus.WriteRegister(ctx, s.id, RegAcceleration.Address, data)
}

// Velocity Control

// Velocity reads the current velocity.
// Returns a signed value; negative indicates reverse direction.
func (s *Servo) Velocity(ctx context.Context) (int, error) {
	data, err := s.bus.ReadRegister(ctx, s.id, RegPresentVelocity.Address, int(RegPresentVelocity.Size))
	if len(data) == 0 {
		return 0, err
	}

	raw := int(s.bus.Protocol().DecodeWord(data))
	return decodeSignMagnitude(raw, RegPresentVelocity.SignBit), err
}

// SetVelocity sets the goal velocity (for wheel mode).
// Positive values rotate clockwise, negative counter-clockwise.
func (s *Servo) SetVelocity(ctx context.Context, velocity int) error {
	encoded := encodeSignMagnitude(velocity, RegGoalVelocity.SignBit)
	data := s.bus.Protocol().EncodeWord(uint16(encoded))
	return s.bus.WriteRegister(ctx, s.id, RegGoalVelocity.Address, data)
}

// Torque Control

// TorqueEnabled returns whether torque is enabled.
func (s *Servo) TorqueEnabled(ctx context.Context) (bool, error) {
	data, err := s.bus.ReadRegister(ctx, s.id, RegTorqueEnable.Address, 1)
	if len(data) == 0 {
		return false, err
	}
	return data[0] != 0, err
}

// SetTorqueEnabled enables or disables torque.
func (s *Servo) SetTorqueEnabled(ctx context.Context, enabled bool) error {
	var val byte
	if enabled {
		val = 1
	}
	return s.bus.WriteRegister(ctx, s.id, RegTorqueEnable.Address, []byte{val})
}

// Enable is a convenience alias for SetTorqueEnabled(true).
func (s *Servo) Enable(ctx context.Context) error {
	return s.SetTorqueEnabled(ctx, true)
}

// Disable is a convenience alias for SetTorqueEnabled(false).
func (s *Servo) Disable(ctx context.Context) error {
	return s.SetTorqueEnabled(ctx, false)
}

// Status

// Moving returns whether the servo is currently moving.
func (s *Servo) Moving(ctx context.Context) (bool, error) {
	data, err := s.bus.ReadRegister(ctx, s.id, RegMoving.Address, 1)
	if len(data) == 0 {
		return false, err
	}
	return data[0] != 0, err
}

// Load reads the current load.
// Returns a signed value; negative indicates load in reverse direction.
func (s *Servo) Load(ctx context.Context) (int, error) {
	data, err := s.bus.ReadRegister(ctx, s.id, RegPresentLoad.Address, int(RegPresentLoad.Size))
	if len(data) == 0 {
		return 0, err
	}

	raw := int(s.bus.Protocol().DecodeWord(data))
	return decodeSignMagnitude(raw, RegPresentLoad.SignBit), err
}

// Voltage reads the current supply voltage in tenths of a volt.
func (s *Servo) Voltage(ctx context.Context) (int, error) {
	data, err := s.bus.ReadRegister(ctx, s.id, RegPresentVoltage.Address, 1)
	if len(data) == 0 {
		return 0, err
	}
	return int(data[0]), err
}

// Temperature reads the current temperature in degrees Celsius.
func (s *Servo) Temperature(ctx context.Context) (int, error) {
	data, err := s.bus.ReadRegister(ctx, s.id, RegPresentTemp.Address, 1)
	if len(data) == 0 {
		return 0, err
	}
	return int(data[0]), err
}

// Configuration

// OperatingMode reads the current operating mode.
func (s *Servo) OperatingMode(ctx context.Context) (OperatingMode, error) {
	data, err := s.bus.ReadRegister(ctx, s.id, RegOperatingMode.Address, 1)
	if len(data) == 0 {
		return 0, err
	}
	return OperatingMode(data[0]), err
}

// SetOperatingMode sets the operating mode.
// Must disable torque first.
func (s *Servo) SetOperatingMode(ctx context.Context, mode OperatingMode) error {
	return s.writeRegister(ctx, RegOperatingMode, []byte{byte(mode)})
}

// PositionLimits reads the min and max position limits.
func (s *Servo) PositionLimits(ctx context.Context) (min, max int, err error) {
	minData, minErr := s.bus.ReadRegister(ctx, s.id, RegMinAngleLimit.Address, 2)
	if len(minData) == 0 {
		return 0, 0, minErr
	}

	maxData, maxErr := s.bus.ReadRegister(ctx, s.id, RegMaxAngleLimit.Address, 2)
	if len(maxData) == 0 {
		return 0, 0, maxErr
	}

	err = cmp.Or(minErr, maxErr)

	proto := s.bus.Protocol()
	return int(proto.DecodeWord(minData)), int(proto.DecodeWord(maxData)), err
}

// SetPositionLimits sets the min and max position limits.
func (s *Servo) SetPositionLimits(ctx context.Context, min, max int) error {
	proto := s.bus.Protocol()

	if err := s.writeRegister(ctx, RegMinAngleLimit, proto.EncodeWord(uint16(min))); err != nil {
		return err
	}
	return s.writeRegister(ctx, RegMaxAngleLimit, proto.EncodeWord(uint16(max)))
}

// EEPROM Configuration — unlock is handled automatically. SetID and SetBaudRate
// also disable torque first.

// SetID changes the servo's ID.
// The servo object is updated with the new ID on success.
func (s *Servo) SetID(ctx context.Context, newID int) error {
	if newID < 0 || newID > int(MaxServoID) {
		return fmt.Errorf("%w: %d", ErrInvalidID, newID)
	}

	// Safety: disable torque first
	if err := s.SetTorqueEnabled(ctx, false); err != nil {
		return fmt.Errorf("failed to disable torque: %w", err)
	}

	if err := s.writeRegister(ctx, RegID, []byte{byte(newID)}); err != nil {
		return err
	}

	s.id = newID
	return nil
}

// SetBaudRate changes the servo's baud rate.
// Takes the actual baud rate value (e.g., 1000000) not the index.
func (s *Servo) SetBaudRate(ctx context.Context, baudRate int) error {
	idx := s.model.BaudRateIndex(baudRate)
	if idx < 0 {
		return fmt.Errorf("baud rate %d not supported by model %s", baudRate, s.model.Name)
	}

	// Safety: disable torque first
	if err := s.SetTorqueEnabled(ctx, false); err != nil {
		return fmt.Errorf("failed to disable torque: %w", err)
	}

	return s.writeRegister(ctx, RegBaudRate, []byte{byte(idx)})
}

// ReadRegister reads a named register.
func (s *Servo) ReadRegister(ctx context.Context, name string) ([]byte, error) {
	reg, ok := s.model.GetRegister(name)
	if !ok {
		return nil, fmt.Errorf("unknown register: %s", name)
	}
	return s.bus.ReadRegister(ctx, s.id, reg.Address, int(reg.Size))
}

// WriteRegister writes to a named register.
func (s *Servo) WriteRegister(ctx context.Context, name string, data []byte) error {
	reg, ok := s.model.GetRegister(name)
	if !ok {
		return fmt.Errorf("unknown register: %s", name)
	}
	if reg.ReadOnly {
		return fmt.Errorf("register %s is read-only", name)
	}
	if len(data) != int(reg.Size) {
		return fmt.Errorf("data size mismatch: expected %d bytes, got %d", reg.Size, len(data))
	}
	return s.writeRegister(ctx, reg, data)
}

// Sign-magnitude encoding helpers

// maxEncodablePosition is the largest magnitude representable when bit 15 is the
// sign bit (bits 0-14 hold the magnitude). Beyond this, a value would overflow
// into the sign bit and silently flip sign.
const maxEncodablePosition = 0x7FFF

// encodePositionWord sign-magnitude-encodes a goal position into protocol bytes,
// rejecting values that cannot be represented without overflowing the sign bit.
func encodePositionWord(proto *Protocol, position int) ([]byte, error) {
	if position < -maxEncodablePosition || position > maxEncodablePosition {
		return nil, fmt.Errorf("position %d out of range [%d, %d]", position, -maxEncodablePosition, maxEncodablePosition)
	}
	return proto.EncodeWord(uint16(encodeSignMagnitude(position, RegGoalPosition.SignBit))), nil
}

// decodePositionWord sign-magnitude-decodes a present position from protocol bytes.
func decodePositionWord(proto *Protocol, data []byte) int {
	return decodeSignMagnitude(int(proto.DecodeWord(data)), RegPresentPosition.SignBit)
}

func decodeSignMagnitude(value, signBit int) int {
	if signBit == 0 {
		return value
	}

	signMask := 1 << signBit
	if value&signMask != 0 {
		return -(value & (signMask - 1))
	}
	return value
}

func encodeSignMagnitude(value, signBit int) int {
	if signBit == 0 {
		return value
	}

	if value < 0 {
		signMask := 1 << signBit
		return (-value) | signMask
	}
	return value
}

// writeRegister routes EEPROM-region writes through the lock dance and SRAM
// writes through the bus directly.
func (s *Servo) writeRegister(ctx context.Context, reg Register, data []byte) error {
	if reg.EEPROM {
		return s.writeEEPROM(ctx, reg.Address, data)
	}
	return s.bus.WriteRegister(ctx, s.id, reg.Address, data)
}

// writeEEPROM unlocks the lock register, performs the write, and re-locks. On
// a write failure, the re-lock is still attempted; errors are joined.
//
// If the model has no lock register (LockAddress == 0), this delegates to a
// plain bus write.
func (s *Servo) writeEEPROM(ctx context.Context, address byte, data []byte) error {
	if s.model.LockAddress == 0 {
		return s.bus.WriteRegister(ctx, s.id, address, data)
	}

	// Step 1: unlock. If this fails, abort — no further packets are sent and
	// the servo is left in its existing locked state.
	if err := s.bus.WriteRegister(ctx, s.id, s.model.LockAddress, []byte{0}); err != nil {
		return err
	}

	// Step 2: target write.
	writeErr := s.bus.WriteRegister(ctx, s.id, address, data)

	// Step 3: re-lock. Always attempted, even if step 2 failed.
	relockErr := s.bus.WriteRegister(ctx, s.id, s.model.LockAddress, []byte{1})

	switch {
	case writeErr != nil && relockErr != nil:
		return errors.Join(writeErr, relockErr)
	case writeErr != nil:
		return writeErr
	case relockErr != nil:
		return relockErr
	}
	return nil
}
