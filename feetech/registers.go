package feetech

// Register represents a servo control table register.
type Register struct {
	Address  byte
	Size     int // 1 or 2 bytes
	ReadOnly bool
	// SignBit indicates which bit is the sign bit for sign-magnitude encoding.
	// 0 means no sign-magnitude encoding (standard two's complement or unsigned).
	SignBit int
	// EEPROM is true when the register is in EEPROM space and writes must be
	// guarded by the lock-register dance (see Servo.writeEEPROM).
	EEPROM bool
}

// Common register addresses shared across most Feetech servos.
// These are for the STS series; SCS series may differ.
var (
	RegModelNumber              = Register{Address: 3, Size: 2, ReadOnly: true, EEPROM: true}
	RegFirmwareVersion          = Register{Address: 0, Size: 1, ReadOnly: true, EEPROM: true}
	RegID                       = Register{Address: 5, Size: 1, EEPROM: true}
	RegBaudRate                 = Register{Address: 6, Size: 1, EEPROM: true}
	RegResponseDelay            = Register{Address: 7, Size: 1, EEPROM: true}
	RegMinAngleLimit            = Register{Address: 9, Size: 2, EEPROM: true}
	RegMaxAngleLimit            = Register{Address: 11, Size: 2, EEPROM: true}
	RegMaxTemp                  = Register{Address: 13, Size: 1, EEPROM: true}
	RegMaxVoltage               = Register{Address: 14, Size: 1, EEPROM: true}
	RegMinVoltage               = Register{Address: 15, Size: 1, EEPROM: true}
	RegMaxTorque                = Register{Address: 16, Size: 2, EEPROM: true}
	RegPhase                    = Register{Address: 18, Size: 1, EEPROM: true}
	RegUnloadCondition          = Register{Address: 19, Size: 1, EEPROM: true}
	RegLEDAlarm                 = Register{Address: 20, Size: 1, EEPROM: true}
	RegPGain                    = Register{Address: 21, Size: 1, EEPROM: true}
	RegDGain                    = Register{Address: 22, Size: 1, EEPROM: true}
	RegIGain                    = Register{Address: 23, Size: 1, EEPROM: true}
	RegMinStartupForce          = Register{Address: 24, Size: 2, EEPROM: true}
	RegClockwiseDeadband        = Register{Address: 26, Size: 1, EEPROM: true}
	RegCounterClockwiseDeadband = Register{Address: 27, Size: 1, EEPROM: true}
	RegProtectionCurrent        = Register{Address: 28, Size: 2, EEPROM: true}
	RegAngularResolution        = Register{Address: 30, Size: 1, EEPROM: true}
	RegPositionOffset           = Register{Address: 31, Size: 2, SignBit: 11, EEPROM: true}
	RegOperatingMode            = Register{Address: 33, Size: 1, EEPROM: true}
	RegProtectionTorque         = Register{Address: 34, Size: 1, EEPROM: true}
	RegProtectionTime           = Register{Address: 35, Size: 1, EEPROM: true}
	RegOverloadTorque           = Register{Address: 36, Size: 1, EEPROM: true}
	RegSpeedClosedLoop          = Register{Address: 37, Size: 1, EEPROM: true}
	RegCurrentClosedLoop        = Register{Address: 38, Size: 1, EEPROM: true}

	// RAM registers (volatile)
	RegTorqueEnable = Register{Address: 40, Size: 1}
	RegAcceleration = Register{Address: 41, Size: 1}
	RegGoalPosition = Register{Address: 42, Size: 2}
	RegGoalTime     = Register{Address: 44, Size: 2}
	RegGoalVelocity = Register{Address: 46, Size: 2, SignBit: 15}
	RegTorqueLimit  = Register{Address: 48, Size: 2}
	RegLock         = Register{Address: 55, Size: 1}

	// Feedback registers (read-only)
	RegPresentPosition = Register{Address: 56, Size: 2, ReadOnly: true}
	RegPresentVelocity = Register{Address: 58, Size: 2, ReadOnly: true, SignBit: 15}
	RegPresentLoad     = Register{Address: 60, Size: 2, ReadOnly: true, SignBit: 9}
	RegPresentVoltage  = Register{Address: 62, Size: 1, ReadOnly: true}
	RegPresentTemp     = Register{Address: 63, Size: 1, ReadOnly: true}
	RegAsyncWriteFlag  = Register{Address: 64, Size: 1, ReadOnly: true}
	RegServoStatus     = Register{Address: 65, Size: 1, ReadOnly: true}
	RegMoving          = Register{Address: 66, Size: 1, ReadOnly: true}
	RegPresentCurrent  = Register{Address: 69, Size: 2, ReadOnly: true}

	// Factory (lives in EEPROM space per datasheet)
	RegMaxAcceleration = Register{Address: 85, Size: 1, EEPROM: true}
)

// Model represents a servo model specification.
type Model struct {
	Name        string
	Number      int // Model number returned by ping
	Protocol    int // ProtocolSTS or ProtocolSCS
	Resolution  int // Position resolution in steps (e.g., 4096 for 12-bit)
	MaxPosition int // Maximum position value

	// Registers maps register names to their definitions.
	// If nil, uses the default STS register map.
	Registers map[string]Register

	// BaudRates lists supported baud rates in index order.
	// Index 0 = first baud rate option, etc.
	BaudRates []int

	// LockAddress is the EEPROM lock register address. Servo writes to an
	// EEPROM-marked register on this model are wrapped in unlock (write 0)
	// and re-lock (write 1) packets at this address. Zero means the model
	// has no lock register and EEPROM writes proceed without an unlock dance.
	LockAddress byte
}

// DefaultBaudRates for most Feetech servos.
var DefaultBaudRates = []int{
	1000000, // 0
	500000,  // 1
	250000,  // 2
	128000,  // 3
	115200,  // 4
	76800,   // 5
	57600,   // 6
	38400,   // 7
}

// Predefined servo models.
var (
	ModelSTS3215 = Model{
		Name:        "sts3215",
		Number:      777,
		Protocol:    ProtocolSTS,
		Resolution:  4096,
		MaxPosition: 4095,
		BaudRates:   DefaultBaudRates,
		LockAddress: 55,
	}

	ModelSTS3250 = Model{
		Name:        "sts3250",
		Number:      1540,
		Protocol:    ProtocolSTS,
		Resolution:  4096,
		MaxPosition: 4095,
		BaudRates:   DefaultBaudRates,
		LockAddress: 55,
	}

	ModelSCS0009 = Model{
		Name:        "scs0009",
		Number:      9,
		Protocol:    ProtocolSCS,
		Resolution:  1024,
		MaxPosition: 1023,
		BaudRates:   DefaultBaudRates,
		Registers:   scs0009Registers,
		LockAddress: 48,
	}

	ModelSCS15 = Model{
		Name:        "scs15",
		Number:      15,
		Protocol:    ProtocolSCS,
		Resolution:  1024,
		MaxPosition: 1023,
		BaudRates:   DefaultBaudRates,
		Registers:   scs0009Registers, // Same register layout
		LockAddress: 48,
	}
)

// SCS series has different register addresses for some values.
var scs0009Registers = map[string]Register{
	"model_number":     {Address: 3, Size: 2, ReadOnly: true, EEPROM: true},
	"id":               {Address: 5, Size: 1, EEPROM: true},
	"baud_rate":        {Address: 6, Size: 1, EEPROM: true},
	"min_angle_limit":  {Address: 9, Size: 2, EEPROM: true},
	"max_angle_limit":  {Address: 11, Size: 2, EEPROM: true},
	"torque_enable":    {Address: 40, Size: 1},
	"goal_position":    {Address: 42, Size: 2},
	"running_time":     {Address: 44, Size: 2}, // Different name than STS
	"running_speed":    {Address: 46, Size: 2},
	"present_position": {Address: 56, Size: 2, ReadOnly: true},
	"present_speed":    {Address: 58, Size: 2, ReadOnly: true},
	"present_load":     {Address: 60, Size: 2, ReadOnly: true},
	"present_voltage":  {Address: 62, Size: 1, ReadOnly: true},
	"present_temp":     {Address: 63, Size: 1, ReadOnly: true},
	"moving":           {Address: 66, Size: 1, ReadOnly: true},
}

// modelRegistry holds all known models indexed by name and number.
var modelRegistry = struct {
	byName   map[string]*Model
	byNumber map[int]*Model
}{
	byName:   make(map[string]*Model),
	byNumber: make(map[int]*Model),
}

func init() {
	// Register built-in models
	RegisterModel(&ModelSTS3215)
	RegisterModel(&ModelSTS3250)
	RegisterModel(&ModelSCS0009)
	RegisterModel(&ModelSCS15)
}

// RegisterModel adds a model to the registry.
func RegisterModel(m *Model) {
	modelRegistry.byName[m.Name] = m
	modelRegistry.byNumber[m.Number] = m
}

// GetModel returns a model by name.
func GetModel(name string) (*Model, bool) {
	m, ok := modelRegistry.byName[name]
	return m, ok
}

// GetModelByNumber returns a model by its hardware model number.
func GetModelByNumber(number int) (*Model, bool) {
	m, ok := modelRegistry.byNumber[number]
	return m, ok
}

// ListModels returns all registered model names.
func ListModels() []string {
	names := make([]string, 0, len(modelRegistry.byName))
	for name := range modelRegistry.byName {
		names = append(names, name)
	}
	return names
}

// GetRegister returns the register definition for the given name.
// It checks model-specific registers first, then falls back to common registers.
func (m *Model) GetRegister(name string) (Register, bool) {
	if m.Registers != nil {
		if reg, ok := m.Registers[name]; ok {
			return reg, true
		}
	}

	// Fall back to common registers
	return getCommonRegister(name)
}

// BaudRateIndex returns the index for a baud rate, or -1 if not supported.
func (m *Model) BaudRateIndex(baudRate int) int {
	for i, rate := range m.BaudRates {
		if rate == baudRate {
			return i
		}
	}
	return -1
}

// getCommonRegister returns a common register by name.
func getCommonRegister(name string) (Register, bool) {
	commonRegisters := map[string]Register{
		"model_number":       RegModelNumber,
		"firmware_version":   RegFirmwareVersion,
		"id":                 RegID,
		"baud_rate":          RegBaudRate,
		"response_delay":     RegResponseDelay,
		"min_angle_limit":    RegMinAngleLimit,
		"max_angle_limit":    RegMaxAngleLimit,
		"max_temp":           RegMaxTemp,
		"max_voltage":        RegMaxVoltage,
		"min_voltage":        RegMinVoltage,
		"max_torque":         RegMaxTorque,
		"operating_mode":     RegOperatingMode,
		"torque_enable":      RegTorqueEnable,
		"acceleration":       RegAcceleration,
		"goal_position":      RegGoalPosition,
		"goal_time":          RegGoalTime,
		"goal_velocity":      RegGoalVelocity,
		"torque_limit":       RegTorqueLimit,
		"lock":               RegLock,
		"present_position":   RegPresentPosition,
		"present_velocity":   RegPresentVelocity,
		"present_load":       RegPresentLoad,
		"present_voltage":    RegPresentVoltage,
		"present_temp":       RegPresentTemp,
		"moving":             RegMoving,
		"present_current":    RegPresentCurrent,
		"position_offset":    RegPositionOffset,
		"protection_current": RegProtectionCurrent,
		"overload_torque":    RegOverloadTorque,
		"p_gain":             RegPGain,
		"d_gain":             RegDGain,
		"i_gain":             RegIGain,
		"max_acceleration":   RegMaxAcceleration,
	}

	reg, ok := commonRegisters[name]
	return reg, ok
}

// Operating modes.
const (
	ModePosition = 0
	ModeVelocity = 1 // Wheel mode
	ModePWM      = 2
	ModeStep     = 3
)
