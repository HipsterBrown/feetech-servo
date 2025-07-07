// calibration.go - Motor calibration definitions and functionality
package feetech

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
)

// Normalization modes
const (
	NormModeRaw       = 0 // Raw servo values (0-4095 for STS3215)
	NormModeRange100  = 1 // Normalized to 0-100 range
	NormModeRangeM100 = 2 // Normalized to -100 to +100 range
	NormModeDegrees   = 3 // Normalized to -180° to +180° range
)

// MotorCalibration defines calibration parameters for a servo motor
type MotorCalibration struct {
	ID           int `json:"id"`                  // Servo ID
	DriveMode    int `json:"drive_mode"`          // Drive direction (0=normal, 1=inverted)
	HomingOffset int `json:"homing_offset"`       // Firmware homing offset (written to servo register)
	RangeMin     int `json:"range_min"`           // Minimum usable position (after homing offset)
	RangeMax     int `json:"range_max"`           // Maximum usable position (after homing offset)
	NormMode     int `json:"norm_mode,omitempty"` // Normalization mode (optional, defaults to degrees)
}

// NewMotorCalibration creates a new motor calibration with default values
func NewMotorCalibration(id int) *MotorCalibration {
	return &MotorCalibration{
		ID:           id,
		DriveMode:    0,               // Normal direction
		HomingOffset: 0,               // No offset
		RangeMin:     0,               // Full range start
		RangeMax:     4095,            // Full range end (STS3215 default)
		NormMode:     NormModeDegrees, // Default to degrees
	}
}

// Validate checks if the calibration parameters are valid
func (c *MotorCalibration) Validate() error {
	if c.ID < 0 || c.ID > MaxID {
		return fmt.Errorf("invalid servo ID: %d (must be 0-%d)", c.ID, MaxID)
	}

	if c.RangeMin >= c.RangeMax {
		return fmt.Errorf("invalid range: min (%d) must be less than max (%d)", c.RangeMin, c.RangeMax)
	}

	if c.RangeMin < 0 || c.RangeMax > 4095 {
		return fmt.Errorf("range values must be between 0-4095, got min=%d max=%d", c.RangeMin, c.RangeMax)
	}

	if c.NormMode < NormModeRaw || c.NormMode > NormModeDegrees {
		return fmt.Errorf("invalid normalization mode: %d", c.NormMode)
	}

	return nil
}

// Clone creates a copy of the calibration
func (c *MotorCalibration) Clone() *MotorCalibration {
	return &MotorCalibration{
		ID:           c.ID,
		DriveMode:    c.DriveMode,
		HomingOffset: c.HomingOffset,
		RangeMin:     c.RangeMin,
		RangeMax:     c.RangeMax,
		NormMode:     c.NormMode,
	}
}

// GetRangeSize returns the usable range size
func (c *MotorCalibration) GetRangeSize() int {
	return c.RangeMax - c.RangeMin
}

// GetCenterPosition returns the center position of the calibrated range
func (c *MotorCalibration) GetCenterPosition() int {
	return (c.RangeMin + c.RangeMax) / 2
}

// NormalizationModeString returns a human-readable string for the normalization mode
func (c *MotorCalibration) NormalizationModeString() string {
	switch c.NormMode {
	case NormModeRaw:
		return "Raw"
	case NormModeRange100:
		return "0-100"
	case NormModeRangeM100:
		return "-100 to +100"
	case NormModeDegrees:
		return "Degrees (-180° to +180°)"
	default:
		return "Unknown"
	}
}

// String returns a string representation of the calibration
func (c *MotorCalibration) String() string {
	direction := "Normal"
	if c.DriveMode != 0 {
		direction = "Inverted"
	}

	return fmt.Sprintf("ID %d: Range[%d-%d] %s %s (offset: %d)",
		c.ID, c.RangeMin, c.RangeMax, c.NormalizationModeString(), direction, c.HomingOffset)
}

// LoadCalibrations loads calibration data from a JSON file
// Supports the flat format with motor names as keys
func LoadCalibrations(filename string) (map[int]*MotorCalibration, error) {
	data, err := os.ReadFile(filename)
	if err != nil {
		return nil, fmt.Errorf("failed to read calibration file: %w", err)
	}

	// Parse as map[string]*MotorCalibration (motor name -> calibration)
	var motorMap map[string]*MotorCalibration
	if err := json.Unmarshal(data, &motorMap); err != nil {
		return nil, fmt.Errorf("failed to parse calibration file: %w", err)
	}

	// Convert to map[int]*MotorCalibration (servo ID -> calibration)
	result := make(map[int]*MotorCalibration)
	for motorName, cal := range motorMap {
		// Set default normalization mode if not specified
		if cal.NormMode == 0 {
			cal.NormMode = NormModeDegrees
		}

		if err := cal.Validate(); err != nil {
			return nil, fmt.Errorf("invalid calibration for motor %s: %w", motorName, err)
		}

		if _, exists := result[cal.ID]; exists {
			return nil, fmt.Errorf("duplicate servo ID %d found in calibration file", cal.ID)
		}

		result[cal.ID] = cal
	}

	return result, nil
}

// SaveCalibrations saves calibration data to a JSON file
// Uses the flat format with motor names as keys
func SaveCalibrations(filename string, calibrations map[int]*MotorCalibration, motorNames map[int]string) error {
	// Convert from map[int]*MotorCalibration to map[string]*MotorCalibration
	motorMap := make(map[string]*MotorCalibration)

	for id, cal := range calibrations {
		motorName, exists := motorNames[id]
		if !exists {
			motorName = fmt.Sprintf("motor_%d", id)
		}
		motorMap[motorName] = cal
	}

	data, err := json.MarshalIndent(motorMap, "", "    ")
	if err != nil {
		return fmt.Errorf("failed to marshal calibrations: %w", err)
	}

	if err := os.WriteFile(filename, data, 0644); err != nil {
		return fmt.Errorf("failed to write calibration file: %w", err)
	}

	return nil
}

// CreateRobotArmCalibration creates a standard calibration set for a robot arm
func CreateRobotArmCalibration(servoIDs []int, motorNames []string) map[int]*MotorCalibration {
	calibrations := make(map[int]*MotorCalibration)

	for _, id := range servoIDs {
		cal := NewMotorCalibration(id)
		cal.RangeMin = 500 // Conservative range
		cal.RangeMax = 3500
		cal.NormMode = NormModeDegrees
		calibrations[id] = cal
	}

	return calibrations
}

// Normalize converts a raw servo position to normalized value
// Takes into account that the servo's present_position already includes the homing offset
func (c *MotorCalibration) Normalize(rawValue int) (float64, error) {
	// Note: The rawValue from the servo already includes the homing offset
	// So we normalize directly against the calibrated range
	var normalized float64

	// First normalize to the target range without considering drive mode
	switch c.NormMode {
	case NormModeRaw:
		normalized = float64(rawValue)

	case NormModeRange100:
		// Map to 0-100 range
		if c.RangeMax == c.RangeMin {
			return 0, fmt.Errorf("invalid calibration: min and max are equal")
		}
		normalized = float64(rawValue-c.RangeMin) / float64(c.RangeMax-c.RangeMin) * 100.0
		normalized = math.Max(0, math.Min(100, normalized))

	case NormModeRangeM100:
		// Map to -100 to +100 range
		if c.RangeMax == c.RangeMin {
			return 0, fmt.Errorf("invalid calibration: min and max are equal")
		}
		center := float64(c.RangeMin+c.RangeMax) / 2.0
		halfRange := float64(c.RangeMax-c.RangeMin) / 2.0
		normalized = (float64(rawValue) - center) / halfRange * 100.0
		normalized = math.Max(-100, math.Min(100, normalized))

	case NormModeDegrees:
		// Map to -180° to +180° range
		center := float64(c.RangeMin+c.RangeMax) / 2.0
		halfRange := float64(c.RangeMax-c.RangeMin) / 2.0
		normalized = (float64(rawValue) - center) / halfRange * 180.0
		normalized = math.Max(-180, math.Min(180, normalized))

	default:
		return 0, fmt.Errorf("unknown normalization mode: %d", c.NormMode)
	}

	// Apply drive mode inversion to the normalized value
	if c.DriveMode != 0 {
		switch c.NormMode {
		case NormModeRaw:
			// For raw mode, invert around the center of the range
			center := float64(c.RangeMin+c.RangeMax) / 2.0
			normalized = 2*center - normalized
		case NormModeRange100:
			// Invert 0-100 becomes 100-0
			normalized = 100.0 - normalized
		case NormModeRangeM100:
			// Invert -100 to +100 becomes +100 to -100
			normalized = -normalized
		case NormModeDegrees:
			// Invert -180° to +180° becomes +180° to -180°
			normalized = -normalized
		}
	}

	return normalized, nil
}

// Denormalize converts normalized value back to raw servo position
// The result should be sent to goal_position, which the servo will apply the homing offset to
func (c *MotorCalibration) Denormalize(normalizedValue float64) (int, error) {
	// Apply drive mode inversion to the normalized value first
	adjustedValue := normalizedValue
	if c.DriveMode != 0 {
		switch c.NormMode {
		case NormModeRaw:
			// For raw mode, invert around the center of the range
			center := float64(c.RangeMin+c.RangeMax) / 2.0
			adjustedValue = 2*center - normalizedValue
		case NormModeRange100:
			// Invert 0-100 becomes 100-0
			adjustedValue = 100.0 - normalizedValue
		case NormModeRangeM100:
			// Invert -100 to +100 becomes +100 to -100
			adjustedValue = -normalizedValue
		case NormModeDegrees:
			// Invert -180° to +180° becomes +180° to -180°
			adjustedValue = -normalizedValue
		}
	}

	var rawValue int

	// Convert from normalized value based on calibration mode
	switch c.NormMode {
	case NormModeRaw:
		rawValue = int(math.Round(adjustedValue))

	case NormModeRange100:
		// Map from 0-100 range
		if c.RangeMax == c.RangeMin {
			return 0, fmt.Errorf("invalid calibration: min and max are equal")
		}
		// Clamp to valid range
		clamped := math.Max(0, math.Min(100, adjustedValue))
		rawValue = int(math.Round(clamped/100.0*float64(c.RangeMax-c.RangeMin) + float64(c.RangeMin)))

	case NormModeRangeM100:
		// Map from -100 to +100 range
		if c.RangeMax == c.RangeMin {
			return 0, fmt.Errorf("invalid calibration: min and max are equal")
		}
		// Clamp to valid range
		clamped := math.Max(-100, math.Min(100, adjustedValue))
		center := float64(c.RangeMin+c.RangeMax) / 2.0
		halfRange := float64(c.RangeMax-c.RangeMin) / 2.0
		rawValue = int(math.Round(center + clamped/100.0*halfRange))

	case NormModeDegrees:
		// Map from -180° to +180° range
		// Clamp to valid range
		clamped := math.Max(-180, math.Min(180, adjustedValue))
		center := float64(c.RangeMin+c.RangeMax) / 2.0
		halfRange := float64(c.RangeMax-c.RangeMin) / 2.0
		rawValue = int(math.Round(center + clamped/180.0*halfRange))

	default:
		return 0, fmt.Errorf("unknown normalization mode: %d", c.NormMode)
	}

	// Clamp to servo limits
	if rawValue < c.RangeMin {
		rawValue = c.RangeMin
	}
	if rawValue > c.RangeMax {
		rawValue = c.RangeMax
	}

	return rawValue, nil
}

// ApplyHomingOffset applies the homing offset to the servo
// This should be called during servo initialization
func (c *MotorCalibration) ApplyHomingOffset(servo *Servo) error {
	// Convert homing offset to bytes (little-endian for STS3215)
	offset := c.HomingOffset
	data := []byte{
		byte(offset & 0xFF),
		byte((offset >> 8) & 0xFF),
	}

	return servo.WriteRegisterByName("homing_offset", data)
}

// GetHomingOffset returns the homing offset value
func (c *MotorCalibration) GetHomingOffset() int {
	return c.HomingOffset
}
