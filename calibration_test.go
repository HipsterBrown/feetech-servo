// calibration_test.go - Unit tests for motor calibration
package feetech

import (
	"fmt"
	"math"
	"os"
	"testing"
)

func TestMotorCalibrationValidation(t *testing.T) {
	tests := []struct {
		name        string
		calibration *MotorCalibration
		expectError bool
	}{
		{
			name: "valid calibration",
			calibration: &MotorCalibration{
				ID:       1,
				RangeMin: 500,
				RangeMax: 3500,
				NormMode: NormModeDegrees,
			},
			expectError: false,
		},
		{
			name: "invalid ID",
			calibration: &MotorCalibration{
				ID:       255, // Too high
				RangeMin: 500,
				RangeMax: 3500,
				NormMode: NormModeDegrees,
			},
			expectError: true,
		},
		{
			name: "invalid range - min >= max",
			calibration: &MotorCalibration{
				ID:       1,
				RangeMin: 3500,
				RangeMax: 500,
				NormMode: NormModeDegrees,
			},
			expectError: true,
		},
		{
			name: "invalid range - out of bounds",
			calibration: &MotorCalibration{
				ID:       1,
				RangeMin: -100,
				RangeMax: 3500,
				NormMode: NormModeDegrees,
			},
			expectError: true,
		},
		{
			name: "invalid norm mode",
			calibration: &MotorCalibration{
				ID:       1,
				RangeMin: 500,
				RangeMax: 3500,
				NormMode: 99, // Invalid mode
			},
			expectError: true,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			err := tt.calibration.Validate()
			if tt.expectError && err == nil {
				t.Error("Expected error but got none")
			}
			if !tt.expectError && err != nil {
				t.Errorf("Unexpected error: %v", err)
			}
		})
	}
}

func TestNormalization(t *testing.T) {
	cal := &MotorCalibration{
		ID:       1,
		RangeMin: 1000,
		RangeMax: 3000,
		NormMode: NormModeDegrees,
	}

	tests := []struct {
		name     string
		rawValue int
		expected float64
	}{
		{"center position", 2000, 0.0},
		{"max position", 3000, 180.0},
		{"min position", 1000, -180.0},
		{"quarter position", 1500, -90.0},
		{"three quarter position", 2500, 90.0},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result, err := cal.Normalize(tt.rawValue)
			if err != nil {
				t.Fatalf("Unexpected error: %v", err)
			}
			if math.Abs(result-tt.expected) > 0.01 {
				t.Errorf("Normalize(%d) = %.2f, want %.2f", tt.rawValue, result, tt.expected)
			}
		})
	}
}

func TestDenormalization(t *testing.T) {
	cal := &MotorCalibration{
		ID:       1,
		RangeMin: 1000,
		RangeMax: 3000,
		NormMode: NormModeDegrees,
	}

	tests := []struct {
		name            string
		normalizedValue float64
		expected        int
	}{
		{"center position", 0.0, 2000},
		{"max position", 180.0, 3000},
		{"min position", -180.0, 1000},
		{"quarter position", -90.0, 1500},
		{"three quarter position", 90.0, 2500},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result, err := cal.Denormalize(tt.normalizedValue)
			if err != nil {
				t.Fatalf("Unexpected error: %v", err)
			}
			if math.Abs(float64(result-tt.expected)) > 1 {
				t.Errorf("Denormalize(%.2f) = %d, want %d", tt.normalizedValue, result, tt.expected)
			}
		})
	}
}

func TestRoundTripNormalization(t *testing.T) {
	modes := []int{NormModeRaw, NormModeRange100, NormModeRangeM100, NormModeDegrees}

	for _, mode := range modes {
		t.Run(fmt.Sprintf("mode_%d", mode), func(t *testing.T) {
			cal := &MotorCalibration{
				ID:       1,
				RangeMin: 500,
				RangeMax: 3500,
				NormMode: mode,
			}

			testValues := []int{500, 1000, 2000, 3000, 3500}

			for _, rawValue := range testValues {
				// Normalize then denormalize
				normalized, err := cal.Normalize(rawValue)
				if err != nil {
					t.Fatalf("Normalize error: %v", err)
				}

				denormalized, err := cal.Denormalize(normalized)
				if err != nil {
					t.Fatalf("Denormalize error: %v", err)
				}

				// Should be close to original (within rounding error)
				if math.Abs(float64(denormalized-rawValue)) > 2 {
					t.Errorf("Round trip failed: %d -> %.2f -> %d", rawValue, normalized, denormalized)
				}
			}
		})
	}
}

func TestDriveModeInversion(t *testing.T) {
	cal := &MotorCalibration{
		ID:        1,
		DriveMode: 1, // Inverted
		RangeMin:  1000,
		RangeMax:  3000,
		NormMode:  NormModeDegrees,
	}

	tests := []struct {
		name     string
		rawValue int
		expected float64
	}{
		{"center position", 2000, 0.0},   // Center should still be 0°
		{"min position", 1000, 180.0},    // Min raw -> Max normalized
		{"max position", 3000, -180.0},   // Max raw -> Min normalized
		{"quarter position", 1500, 90.0}, // Quarter -> 3/4 normalized
		{"three quarter", 2500, -90.0},   // 3/4 -> Quarter normalized
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			normalized, err := cal.Normalize(tt.rawValue)
			if err != nil {
				t.Fatalf("Unexpected error: %v", err)
			}

			if math.Abs(normalized-tt.expected) > 0.01 {
				t.Errorf("Inverted normalize(%d) = %.2f°, want %.2f°", tt.rawValue, normalized, tt.expected)
			}

			// Test round trip
			denormalized, err := cal.Denormalize(normalized)
			if err != nil {
				t.Fatalf("Denormalize error: %v", err)
			}

			if math.Abs(float64(denormalized-tt.rawValue)) > 1 {
				t.Errorf("Round trip failed: %d -> %.2f° -> %d", tt.rawValue, normalized, denormalized)
			}
		})
	}
}

func TestDriveModeInversionAllModes(t *testing.T) {
	testCases := []struct {
		name     string
		normMode int
		testVal  int
		expected float64
	}{
		{"Range100 center", NormModeRange100, 2000, 50.0}, // Center -> 50%
		{"Range100 min", NormModeRange100, 1000, 100.0},   // Min -> 100% (inverted)
		{"Range100 max", NormModeRange100, 3000, 0.0},     // Max -> 0% (inverted)

		{"RangeM100 center", NormModeRangeM100, 2000, 0.0}, // Center -> 0
		{"RangeM100 min", NormModeRangeM100, 1000, 100.0},  // Min -> +100 (inverted)
		{"RangeM100 max", NormModeRangeM100, 3000, -100.0}, // Max -> -100 (inverted)

		{"Degrees center", NormModeDegrees, 2000, 0.0}, // Center -> 0°
		{"Degrees min", NormModeDegrees, 1000, 180.0},  // Min -> +180° (inverted)
		{"Degrees max", NormModeDegrees, 3000, -180.0}, // Max -> -180° (inverted)
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			cal := &MotorCalibration{
				ID:        1,
				DriveMode: 1, // Inverted
				RangeMin:  1000,
				RangeMax:  3000,
				NormMode:  tc.normMode,
			}

			normalized, err := cal.Normalize(tc.testVal)
			if err != nil {
				t.Fatalf("Unexpected error: %v", err)
			}

			if math.Abs(normalized-tc.expected) > 0.01 {
				t.Errorf("Mode %d: normalize(%d) = %.2f, want %.2f", tc.normMode, tc.testVal, normalized, tc.expected)
			}

			// Test round trip
			denormalized, err := cal.Denormalize(normalized)
			if err != nil {
				t.Fatalf("Denormalize error: %v", err)
			}

			if math.Abs(float64(denormalized-tc.testVal)) > 1 {
				t.Errorf("Round trip failed: %d -> %.2f -> %d", tc.testVal, normalized, denormalized)
			}
		})
	}
}

func TestCalibrationFileOperations(t *testing.T) {
	// Create test calibrations
	calibrations := map[int]*MotorCalibration{
		1: {
			ID:           1,
			DriveMode:    0,
			HomingOffset: -1470,
			RangeMin:     758,
			RangeMax:     3292,
			NormMode:     NormModeDegrees,
		},
		2: {
			ID:           2,
			DriveMode:    0,
			HomingOffset: -1177,
			RangeMin:     916,
			RangeMax:     2988,
			NormMode:     NormModeRange100,
		},
	}

	// Motor names
	motorNames := map[int]string{
		1: "shoulder_pan",
		2: "shoulder_lift",
	}

	// Save to temporary file
	filename := "test_calibrations.json"
	defer os.Remove(filename)

	err := SaveCalibrations(filename, calibrations, motorNames)
	if err != nil {
		t.Fatalf("Failed to save calibrations: %v", err)
	}

	// Load from file
	loadedCalibrations, err := LoadCalibrations(filename)
	if err != nil {
		t.Fatalf("Failed to load calibrations: %v", err)
	}

	// Verify loaded calibrations
	if len(loadedCalibrations) != len(calibrations) {
		t.Errorf("Expected %d calibrations, got %d", len(calibrations), len(loadedCalibrations))
	}

	for id, original := range calibrations {
		loaded, exists := loadedCalibrations[id]
		if !exists {
			t.Errorf("Missing calibration for servo %d", id)
			continue
		}

		if loaded.ID != original.ID ||
			loaded.RangeMin != original.RangeMin ||
			loaded.RangeMax != original.RangeMax ||
			loaded.NormMode != original.NormMode ||
			loaded.HomingOffset != original.HomingOffset ||
			loaded.DriveMode != original.DriveMode {
			t.Errorf("Calibration mismatch for servo %d", id)
		}
	}
}

func TestCreateRobotArmCalibration(t *testing.T) {
	servoIDs := []int{1, 2, 3, 4, 5, 6}
	motorNames := []string{"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"}
	calibrations := CreateRobotArmCalibration(servoIDs, motorNames)

	if len(calibrations) != len(servoIDs) {
		t.Errorf("Expected %d calibrations, got %d", len(servoIDs), len(calibrations))
	}

	for _, id := range servoIDs {
		cal, exists := calibrations[id]
		if !exists {
			t.Errorf("Missing calibration for servo %d", id)
			continue
		}

		if cal.ID != id {
			t.Errorf("Calibration ID mismatch: expected %d, got %d", id, cal.ID)
		}

		if cal.NormMode != NormModeDegrees {
			t.Errorf("Expected NormModeDegrees, got %d", cal.NormMode)
		}
	}
}

func TestCalibrationString(t *testing.T) {
	cal := &MotorCalibration{
		ID:           1,
		RangeMin:     500,
		RangeMax:     3500,
		NormMode:     NormModeDegrees,
		HomingOffset: -1470,
	}

	str := cal.String()
	expected := "ID 1: Range[500-3500] Degrees (-180° to +180°) Normal (offset: -1470)"

	if str != expected {
		t.Errorf("String() = %q, want %q", str, expected)
	}
}

func TestHomingOffsetHandling(t *testing.T) {
	cal := &MotorCalibration{
		ID:           1,
		DriveMode:    0,
		HomingOffset: -1470,
		RangeMin:     758,
		RangeMax:     3292,
		NormMode:     NormModeDegrees,
	}

	// Test that homing offset doesn't affect normalization
	// (because the servo's present_position already includes the offset)
	centerRaw := (cal.RangeMin + cal.RangeMax) / 2 // Should be ~2025

	normalized, err := cal.Normalize(centerRaw)
	if err != nil {
		t.Fatalf("Normalize failed: %v", err)
	}

	// Center position should normalize to 0 degrees regardless of homing offset
	if math.Abs(normalized) > 0.01 {
		t.Errorf("Center position should normalize to 0°, got %.2f°", normalized)
	}

	// Test denormalization
	denormalized, err := cal.Denormalize(0.0)
	if err != nil {
		t.Fatalf("Denormalize failed: %v", err)
	}

	// Should get back close to the center raw position
	if math.Abs(float64(denormalized-centerRaw)) > 1 {
		t.Errorf("Denormalize(0°) should be ~%d, got %d", centerRaw, denormalized)
	}
}

func TestDefaultNormalizationMode(t *testing.T) {
	// Create test calibration file without norm_mode field
	testData := `{
		"shoulder_pan": {
			"id": 1,
			"drive_mode": 0,
			"homing_offset": -1470,
			"range_min": 758,
			"range_max": 3292
		}
	}`

	filename := "test_default_norm.json"
	defer os.Remove(filename)

	err := os.WriteFile(filename, []byte(testData), 0644)
	if err != nil {
		t.Fatalf("Failed to write test file: %v", err)
	}

	calibrations, err := LoadCalibrations(filename)
	if err != nil {
		t.Fatalf("Failed to load calibrations: %v", err)
	}

	cal, exists := calibrations[1]
	if !exists {
		t.Fatal("Expected calibration for servo 1")
	}

	// Should default to degrees mode
	if cal.NormMode != NormModeDegrees {
		t.Errorf("Expected default NormMode to be %d (degrees), got %d", NormModeDegrees, cal.NormMode)
	}
}

func BenchmarkNormalize(b *testing.B) {
	cal := &MotorCalibration{
		ID:       1,
		RangeMin: 500,
		RangeMax: 3500,
		NormMode: NormModeDegrees,
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		cal.Normalize(2000)
	}
}

func BenchmarkDenormalize(b *testing.B) {
	cal := &MotorCalibration{
		ID:       1,
		RangeMin: 500,
		RangeMax: 3500,
		NormMode: NormModeDegrees,
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		cal.Denormalize(90.0)
	}
}
