// models.go - Model definitions for supported servos
package main

import "fmt"

// ServoModel defines characteristics of a servo model
type ServoModel struct {
	Name         string
	ModelNumber  int
	Protocol     int
	Resolution   int
	BaudRates    []int
	AddressMap   map[string]AddressInfo
	EncodingBits map[string]int // For sign-magnitude encoding
}

// AddressInfo contains register address and size information
type AddressInfo struct {
	Address byte
	Size    int
}

// STS3215 model definition
var STS3215 = ServoModel{
	Name:        "sts3215",
	ModelNumber: 777,
	Protocol:    ProtocolV0,
	Resolution:  4096,
	BaudRates:   []int{1000000, 500000, 250000, 128000, 115200, 57600, 38400, 19200},
	AddressMap: map[string]AddressInfo{
		"model_number":        {Address: 3, Size: 2},
		"id":                  {Address: 5, Size: 1},
		"baud_rate":           {Address: 6, Size: 1},
		"return_delay_time":   {Address: 7, Size: 1},
		"min_position_limit":  {Address: 9, Size: 2},
		"max_position_limit":  {Address: 11, Size: 2},
		"max_temperature":     {Address: 13, Size: 1},
		"max_voltage":         {Address: 14, Size: 1},
		"min_voltage":         {Address: 15, Size: 1},
		"homing_offset":       {Address: 31, Size: 2},
		"operating_mode":      {Address: 33, Size: 1},
		"torque_enable":       {Address: 40, Size: 1},
		"acceleration":        {Address: 41, Size: 1},
		"goal_position":       {Address: 42, Size: 2},
		"goal_time":           {Address: 44, Size: 2},
		"goal_velocity":       {Address: 46, Size: 2},
		"present_position":    {Address: 56, Size: 2},
		"present_velocity":    {Address: 58, Size: 2},
		"present_load":        {Address: 60, Size: 2},
		"present_voltage":     {Address: 62, Size: 1},
		"present_temperature": {Address: 63, Size: 1},
		"moving":              {Address: 66, Size: 1},
	},
	EncodingBits: map[string]int{
		"homing_offset":    11,
		"goal_velocity":    15,
		"present_velocity": 15,
	},
}

// GetModel returns the servo model definition for a given model name
func GetModel(name string) (*ServoModel, bool) {
	switch name {
	case "sts3215":
		return &STS3215, true
	default:
		return nil, false
	}
}

// GetModelByNumber returns the servo model definition for a given model number
func GetModelByNumber(number int) (*ServoModel, bool) {
	switch number {
	case 777:
		return &STS3215, true
	default:
		return nil, false
	}
}

// BaudRateValue returns the register value for a given baud rate
func (m *ServoModel) BaudRateValue(baudRate int) (byte, bool) {
	for i, rate := range m.BaudRates {
		if rate == baudRate {
			return byte(i), true
		}
	}
	return 0, false
}

// Advanced servo operations using model information
func (s *Servo) ReadRegisterByName(name string) ([]byte, error) {
	model, ok := GetModel(s.Model)
	if !ok {
		return nil, fmt.Errorf("unknown servo model: %s", s.Model)
	}

	addrInfo, ok := model.AddressMap[name]
	if !ok {
		return nil, fmt.Errorf("unknown register: %s", name)
	}

	return s.readRegister(addrInfo.Address, addrInfo.Size)
}

func (s *Servo) WriteRegisterByName(name string, data []byte) error {
	model, ok := GetModel(s.Model)
	if !ok {
		return fmt.Errorf("unknown servo model: %s", s.Model)
	}

	addrInfo, ok := model.AddressMap[name]
	if !ok {
		return fmt.Errorf("unknown register: %s", name)
	}

	if len(data) != addrInfo.Size {
		return fmt.Errorf("data size mismatch: expected %d, got %d", addrInfo.Size, len(data))
	}

	return s.writeRegister(addrInfo.Address, data)
}

// Operating modes for STS3215
const (
	OperatingModePosition = 0
	OperatingModeVelocity = 1
	OperatingModePWM      = 2
	OperatingModeStep     = 3
)

// SetOperatingMode sets the operating mode of the servo
func (s *Servo) SetOperatingMode(mode byte) error {
	return s.WriteRegisterByName("operating_mode", []byte{mode})
}

// GetOperatingMode gets the current operating mode of the servo
func (s *Servo) GetOperatingMode() (byte, error) {
	data, err := s.ReadRegisterByName("operating_mode")
	if err != nil {
		return 0, err
	}
	return data[0], nil
}
