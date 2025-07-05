package main

import (
	"testing"
	"time"
)

func TestPacketChecksumCalculation(t *testing.T) {
	bus := &Bus{protocol: ProtocolV0}

	tests := []struct {
		name     string
		packet   []byte
		expected byte
	}{
		{
			name:     "ping packet",
			packet:   []byte{0x01, 0x02, 0x01}, // ID=1, Length=2, Instruction=PING
			expected: ^byte(0x01 + 0x02 + 0x01),
		},
		{
			name:     "read packet",
			packet:   []byte{0x01, 0x04, 0x02, 0x38, 0x02}, // ID=1, Length=4, READ, Addr=56, Len=2
			expected: ^byte(0x01 + 0x04 + 0x02 + 0x38 + 0x02),
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := bus.calculateChecksum(tt.packet)
			if result != tt.expected {
				t.Errorf("calculateChecksum() = 0x%02X, want 0x%02X", result, tt.expected)
			}
		})
	}
}

func TestPacketChecksumVerification(t *testing.T) {
	bus := &Bus{protocol: ProtocolV0}

	tests := []struct {
		name     string
		packet   []byte
		expected bool
	}{
		{
			name:     "valid packet",
			packet:   []byte{0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC}, // Valid ping response
			expected: true,
		},
		{
			name:     "invalid checksum",
			packet:   []byte{0xFF, 0xFF, 0x01, 0x02, 0x00, 0x00}, // Wrong checksum
			expected: false,
		},
		{
			name:     "too short",
			packet:   []byte{0xFF, 0xFF, 0x01},
			expected: false,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := bus.verifyChecksum(tt.packet)
			if result != tt.expected {
				t.Errorf("verifyChecksum() = %v, want %v", result, tt.expected)
			}
		})
	}
}

func TestMakeWord(t *testing.T) {
	tests := []struct {
		name     string
		protocol int
		low      byte
		high     byte
		expected int
	}{
		{
			name:     "protocol 0 little endian",
			protocol: ProtocolV0,
			low:      0x34,
			high:     0x12,
			expected: 0x1234,
		},
		{
			name:     "protocol 1 big endian",
			protocol: ProtocolV1,
			low:      0x34,
			high:     0x12,
			expected: 0x3412,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			bus := &Bus{protocol: tt.protocol}
			result := bus.makeWord(tt.low, tt.high)
			if result != tt.expected {
				t.Errorf("makeWord() = 0x%04X, want 0x%04X", result, tt.expected)
			}
		})
	}
}

func TestServoModelLookup(t *testing.T) {
	// Test lookup by name
	model, ok := GetModel("sts3215")
	if !ok {
		t.Fatal("Expected to find STS3215 model")
	}
	if model.Name != "sts3215" {
		t.Errorf("Expected model name 'sts3215', got '%s'", model.Name)
	}
	if model.ModelNumber != 777 {
		t.Errorf("Expected model number 777, got %d", model.ModelNumber)
	}

	// Test lookup by model number
	model2, ok := GetModelByNumber(777)
	if !ok {
		t.Fatal("Expected to find model with number 777")
	}
	if model2.Name != "sts3215" {
		t.Errorf("Expected model name 'sts3215', got '%s'", model2.Name)
	}

	// Test unknown model
	_, ok = GetModel("unknown")
	if ok {
		t.Error("Expected not to find unknown model")
	}
}

func TestBaudRateValue(t *testing.T) {
	model, _ := GetModel("sts3215")

	tests := []struct {
		baudRate int
		expected byte
		found    bool
	}{
		{1000000, 0, true},
		{500000, 1, true},
		{115200, 4, true},
		{9600, 0, false}, // Not supported
	}

	for _, tt := range tests {
		t.Run(string(rune(tt.baudRate)), func(t *testing.T) {
			value, found := model.BaudRateValue(tt.baudRate)
			if found != tt.found {
				t.Errorf("BaudRateValue(%d) found = %v, want %v", tt.baudRate, found, tt.found)
			}
			if found && value != tt.expected {
				t.Errorf("BaudRateValue(%d) = %d, want %d", tt.baudRate, value, tt.expected)
			}
		})
	}
}

func TestBusConfiguration(t *testing.T) {
	tests := []struct {
		name        string
		config      BusConfig
		expectError bool
	}{
		{
			name: "valid config",
			config: BusConfig{
				Port:     "/dev/null", // Won't actually connect in test
				Baudrate: 1000000,
				Protocol: ProtocolV0,
				Timeout:  time.Second,
			},
			expectError: true, // Will fail to open /dev/null as serial port
		},
		{
			name: "invalid protocol",
			config: BusConfig{
				Port:     "/dev/ttyUSB0",
				Protocol: 5, // Invalid protocol
			},
			expectError: true,
		},
		{
			name: "default values",
			config: BusConfig{
				Port:     "/dev/null",
				Protocol: ProtocolV0,
				// Baudrate and Timeout will use defaults
			},
			expectError: true, // Will fail to open port
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			_, err := NewBus(tt.config)
			if tt.expectError {
				if err == nil {
					t.Error("Expected error but got none")
				}
			} else {
				if err != nil {
					t.Errorf("Unexpected error: %v", err)
				}
			}
		})
	}
}

func TestErrorTypes(t *testing.T) {
	// Test ServoError
	servoErr := &ServoError{
		ServoID: 1,
		Op:      "read position",
		Err:     NewCommunicationError(CommRxTimeout),
	}

	expected := "servo 1 read position: communication error (code -6): no status packet received"
	if servoErr.Error() != expected {
		t.Errorf("ServoError.Error() = %q, want %q", servoErr.Error(), expected)
	}

	// Test CommunicationError
	commErr := NewCommunicationError(CommTxFail)
	expected = "communication error (code -2): failed to transmit packet"
	if commErr.Error() != expected {
		t.Errorf("CommunicationError.Error() = %q, want %q", commErr.Error(), expected)
	}
}

// Benchmark tests
func BenchmarkChecksumCalculation(b *testing.B) {
	bus := &Bus{protocol: ProtocolV0}
	packet := []byte{0x01, 0x04, 0x02, 0x38, 0x02}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		bus.calculateChecksum(packet)
	}
}

func BenchmarkMakeWord(b *testing.B) {
	bus := &Bus{protocol: ProtocolV0}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		bus.makeWord(0x34, 0x12)
	}
}
