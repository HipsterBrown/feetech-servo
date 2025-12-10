package feetech

import (
	"context"
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
	data, err := s.bus.ReadRegister(ctx, s.id, RegPresentPosition.Address, RegPresentPosition.Size)
	if err != nil {
		return 0, err
	}
	return int(s.bus.Protocol().DecodeWord(data)), nil
}

// SetPosition commands the servo to move to the specified position.
func (s *Servo) SetPosition(ctx context.Context, position int) error {
	data := s.bus.Protocol().EncodeWord(uint16(position))
	return s.bus.WriteRegister(ctx, s.id, RegGoalPosition.Address, data)
}

// SetPositionWithSpeed commands the servo to move to position at the specified speed.
// Speed is in steps per second.
func (s *Servo) SetPositionWithSpeed(ctx context.Context, position, speed int) error {
	proto := s.bus.Protocol()

	// Write position and velocity together (6 bytes starting at goal position)
	// Format: position(2) + time(2) + velocity(2)
	data := make([]byte, 6)
	copy(data[0:2], proto.EncodeWord(uint16(position)))
	copy(data[2:4], proto.EncodeWord(0)) // Time = 0 (use speed instead)
	copy(data[4:6], proto.EncodeWord(uint16(speed)))

	return s.bus.WriteRegister(ctx, s.id, RegGoalPosition.Address, data)
}

// SetPositionWithTime commands the servo to reach position in the specified time.
// Time is in milliseconds.
func (s *Servo) SetPositionWithTime(ctx context.Context, position, timeMs int) error {
	proto := s.bus.Protocol()

	data := make([]byte, 6)
	copy(data[0:2], proto.EncodeWord(uint16(position)))
	copy(data[2:4], proto.EncodeWord(uint16(timeMs)))
	copy(data[4:6], proto.EncodeWord(0)) // Speed = 0 (use time instead)

	return s.bus.WriteRegister(ctx, s.id, RegGoalPosition.Address, data)
}

// Velocity Control

// Velocity reads the current velocity.
// Returns a signed value; negative indicates reverse direction.
func (s *Servo) Velocity(ctx context.Context) (int, error) {
	data, err := s.bus.ReadRegister(ctx, s.id, RegPresentVelocity.Address, RegPresentVelocity.Size)
	if err != nil {
		return 0, err
	}

	raw := int(s.bus.Protocol().DecodeWord(data))
	return decodeSignMagnitude(raw, RegPresentVelocity.SignBit), nil
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
	if err != nil {
		return false, err
	}
	return data[0] != 0, nil
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
	if err != nil {
		return false, err
	}
	return data[0] != 0, nil
}

// Load reads the current load.
// Returns a signed value; negative indicates load in reverse direction.
func (s *Servo) Load(ctx context.Context) (int, error) {
	data, err := s.bus.ReadRegister(ctx, s.id, RegPresentLoad.Address, RegPresentLoad.Size)
	if err != nil {
		return 0, err
	}

	raw := int(s.bus.Protocol().DecodeWord(data))
	return decodeSignMagnitude(raw, RegPresentLoad.SignBit), nil
}

// Voltage reads the current supply voltage in tenths of a volt.
func (s *Servo) Voltage(ctx context.Context) (int, error) {
	data, err := s.bus.ReadRegister(ctx, s.id, RegPresentVoltage.Address, 1)
	if err != nil {
		return 0, err
	}
	return int(data[0]), nil
}

// Temperature reads the current temperature in degrees Celsius.
func (s *Servo) Temperature(ctx context.Context) (int, error) {
	data, err := s.bus.ReadRegister(ctx, s.id, RegPresentTemp.Address, 1)
	if err != nil {
		return 0, err
	}
	return int(data[0]), nil
}

// Configuration

// OperatingMode reads the current operating mode.
func (s *Servo) OperatingMode(ctx context.Context) (int, error) {
	data, err := s.bus.ReadRegister(ctx, s.id, RegOperatingMode.Address, 1)
	if err != nil {
		return 0, err
	}
	return int(data[0]), nil
}

// SetOperatingMode sets the operating mode.
// Must disable torque first.
func (s *Servo) SetOperatingMode(ctx context.Context, mode int) error {
	return s.bus.WriteRegister(ctx, s.id, RegOperatingMode.Address, []byte{byte(mode)})
}

// PositionLimits reads the min and max position limits.
func (s *Servo) PositionLimits(ctx context.Context) (min, max int, err error) {
	minData, err := s.bus.ReadRegister(ctx, s.id, RegMinAngleLimit.Address, 2)
	if err != nil {
		return 0, 0, err
	}

	maxData, err := s.bus.ReadRegister(ctx, s.id, RegMaxAngleLimit.Address, 2)
	if err != nil {
		return 0, 0, err
	}

	proto := s.bus.Protocol()
	return int(proto.DecodeWord(minData)), int(proto.DecodeWord(maxData)), nil
}

// SetPositionLimits sets the min and max position limits.
func (s *Servo) SetPositionLimits(ctx context.Context, min, max int) error {
	proto := s.bus.Protocol()

	if err := s.bus.WriteRegister(ctx, s.id, RegMinAngleLimit.Address, proto.EncodeWord(uint16(min))); err != nil {
		return err
	}
	return s.bus.WriteRegister(ctx, s.id, RegMaxAngleLimit.Address, proto.EncodeWord(uint16(max)))
}

// EEPROM Configuration (requires torque disabled and lock disabled)

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

	if err := s.bus.WriteRegister(ctx, s.id, RegID.Address, []byte{byte(newID)}); err != nil {
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

	return s.bus.WriteRegister(ctx, s.id, RegBaudRate.Address, []byte{byte(idx)})
}

// ReadRegister reads a named register.
func (s *Servo) ReadRegister(ctx context.Context, name string) ([]byte, error) {
	reg, ok := s.model.GetRegister(name)
	if !ok {
		return nil, fmt.Errorf("unknown register: %s", name)
	}
	return s.bus.ReadRegister(ctx, s.id, reg.Address, reg.Size)
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
	if len(data) != reg.Size {
		return fmt.Errorf("data size mismatch: expected %d bytes, got %d", reg.Size, len(data))
	}
	return s.bus.WriteRegister(ctx, s.id, reg.Address, data)
}

// Sign-magnitude encoding helpers

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
