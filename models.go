// models.go - Servo model definitions and specifications
package feetech

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
var STS3215Model = ServoModel{
	Name:        "sts3215",
	ModelNumber: 777,
	Protocol:    ProtocolV0,
	Resolution:  4096,
	BaudRates:   []int{1000000, 500000, 250000, 128000, 115200, 57600, 38400, 19200},
	AddressMap: map[string]AddressInfo{
		// EPROM (Read-only)
		"firmware_major_version": {Address: 0, Size: 1},
		"firmware_minor_version": {Address: 1, Size: 1},
		"model_number":           {Address: 3, Size: 2},

		// EPROM (Read-write)
		"id":                           {Address: 5, Size: 1},
		"baud_rate":                    {Address: 6, Size: 1},
		"return_delay_time":            {Address: 7, Size: 1},
		"response_status_level":        {Address: 8, Size: 1},
		"min_position_limit":           {Address: 9, Size: 2},
		"max_position_limit":           {Address: 11, Size: 2},
		"max_temperature_limit":        {Address: 13, Size: 1},
		"max_voltage_limit":            {Address: 14, Size: 1},
		"min_voltage_limit":            {Address: 15, Size: 1},
		"max_torque_limit":             {Address: 16, Size: 2},
		"phase":                        {Address: 18, Size: 1},
		"unloading_condition":          {Address: 19, Size: 1},
		"led_alarm_condition":          {Address: 20, Size: 1},
		"p_coefficient":                {Address: 21, Size: 1},
		"d_coefficient":                {Address: 22, Size: 1},
		"i_coefficient":                {Address: 23, Size: 1},
		"minimum_startup_force":        {Address: 24, Size: 2},
		"cw_dead_zone":                 {Address: 26, Size: 1},
		"ccw_dead_zone":                {Address: 27, Size: 1},
		"protection_current":           {Address: 28, Size: 2},
		"angular_resolution":           {Address: 30, Size: 1},
		"homing_offset":                {Address: 31, Size: 2},
		"operating_mode":               {Address: 33, Size: 1},
		"protective_torque":            {Address: 34, Size: 1},
		"protection_time":              {Address: 35, Size: 1},
		"overload_torque":              {Address: 36, Size: 1},
		"velocity_p_coefficient":       {Address: 37, Size: 1},
		"over_current_protection_time": {Address: 38, Size: 1},
		"velocity_i_coefficient":       {Address: 39, Size: 1},

		// SRAM (Read-write)
		"torque_enable": {Address: 40, Size: 1},
		"acceleration":  {Address: 41, Size: 1},
		"goal_position": {Address: 42, Size: 2},
		"goal_time":     {Address: 44, Size: 2},
		"goal_velocity": {Address: 46, Size: 2},
		"torque_limit":  {Address: 48, Size: 2},
		"lock":          {Address: 55, Size: 1},

		// SRAM (Read-only)
		"present_position":    {Address: 56, Size: 2},
		"present_velocity":    {Address: 58, Size: 2},
		"present_load":        {Address: 60, Size: 2},
		"present_voltage":     {Address: 62, Size: 1},
		"present_temperature": {Address: 63, Size: 1},
		"status":              {Address: 65, Size: 1},
		"moving":              {Address: 66, Size: 1},
		"present_current":     {Address: 69, Size: 2},
		"goal_position_2":     {Address: 71, Size: 2},

		// Factory settings
		"moving_velocity":           {Address: 80, Size: 1},
		"moving_velocity_threshold": {Address: 80, Size: 1},
		"dts":                       {Address: 81, Size: 1},
		"velocity_unit_factor":      {Address: 82, Size: 1},
		"hts":                       {Address: 83, Size: 1},
		"maximum_velocity_limit":    {Address: 84, Size: 1},
		"maximum_acceleration":      {Address: 85, Size: 1},
		"acceleration_multiplier":   {Address: 86, Size: 1},
	},
	EncodingBits: map[string]int{
		"homing_offset":    11, // Sign-magnitude encoding with bit 11 as sign
		"goal_velocity":    15, // Sign-magnitude encoding with bit 15 as sign
		"present_velocity": 15, // Sign-magnitude encoding with bit 15 as sign
	},
}

// STS3250 model definition (similar to STS3215 but different model number)
var STS3250Model = ServoModel{
	Name:         "sts3250",
	ModelNumber:  2825,
	Protocol:     ProtocolV0,
	Resolution:   4096,
	BaudRates:    []int{1000000, 500000, 250000, 128000, 115200, 57600, 38400, 19200},
	AddressMap:   STS3215Model.AddressMap, // Same register layout as STS3215
	EncodingBits: STS3215Model.EncodingBits,
}

// SCS0009 model definition (Protocol 1)
var SCS0009Model = ServoModel{
	Name:        "scs0009",
	ModelNumber: 1284,
	Protocol:    ProtocolV1,
	Resolution:  1024,
	BaudRates:   []int{1000000, 500000, 250000, 128000, 115200, 57600, 38400, 19200},
	AddressMap: map[string]AddressInfo{
		// EPROM (Read-only)
		"firmware_major_version": {Address: 0, Size: 1},
		"firmware_minor_version": {Address: 1, Size: 1},
		"model_number":           {Address: 3, Size: 2},

		// EPROM (Read-write)
		"id":                    {Address: 5, Size: 1},
		"baud_rate":             {Address: 6, Size: 1},
		"return_delay_time":     {Address: 7, Size: 1},
		"response_status_level": {Address: 8, Size: 1},
		"min_position_limit":    {Address: 9, Size: 2},
		"max_position_limit":    {Address: 11, Size: 2},
		"max_temperature_limit": {Address: 13, Size: 1},
		"max_voltage_limit":     {Address: 14, Size: 1},
		"min_voltage_limit":     {Address: 15, Size: 1},
		"max_torque_limit":      {Address: 16, Size: 2},
		"phase":                 {Address: 18, Size: 1},
		"unloading_condition":   {Address: 19, Size: 1},
		"led_alarm_condition":   {Address: 20, Size: 1},
		"p_coefficient":         {Address: 21, Size: 1},
		"d_coefficient":         {Address: 22, Size: 1},
		"i_coefficient":         {Address: 23, Size: 1},
		"minimum_startup_force": {Address: 24, Size: 2},
		"cw_dead_zone":          {Address: 26, Size: 1},
		"ccw_dead_zone":         {Address: 27, Size: 1},
		"protective_torque":     {Address: 37, Size: 1},
		"protection_time":       {Address: 38, Size: 1},

		// SRAM (Read-write)
		"torque_enable": {Address: 40, Size: 1},
		"acceleration":  {Address: 41, Size: 1},
		"goal_position": {Address: 42, Size: 2},
		"running_time":  {Address: 44, Size: 2}, // Different from STS series
		"goal_velocity": {Address: 46, Size: 2},
		"lock":          {Address: 48, Size: 1},

		// SRAM (Read-only)
		"present_position":    {Address: 56, Size: 2},
		"present_velocity":    {Address: 58, Size: 2},
		"present_load":        {Address: 60, Size: 2},
		"present_voltage":     {Address: 62, Size: 1},
		"present_temperature": {Address: 63, Size: 1},
		"sync_write_flag":     {Address: 64, Size: 1},
		"status":              {Address: 65, Size: 1},
		"moving":              {Address: 66, Size: 1},

		// Factory settings
		"pwm_maximum_step":          {Address: 78, Size: 1},
		"moving_velocity_threshold": {Address: 79, Size: 1},
		"dts":                       {Address: 80, Size: 1},
		"minimum_velocity_limit":    {Address: 81, Size: 1},
		"maximum_velocity_limit":    {Address: 82, Size: 1},
		"acceleration_2":            {Address: 83, Size: 1},
	},
	EncodingBits: map[string]int{
		// SCS series doesn't use sign-magnitude encoding
	},
}

// SM8512BL model definition (16-bit resolution)
var SM8512BLModel = ServoModel{
	Name:         "sm8512bl",
	ModelNumber:  11272,
	Protocol:     ProtocolV0,
	Resolution:   65536, // 16-bit resolution
	BaudRates:    []int{1000000, 500000, 250000, 128000, 115200, 57600, 38400, 19200},
	AddressMap:   STS3215Model.AddressMap, // Same register layout as STS series
	EncodingBits: STS3215Model.EncodingBits,
}

// Model registry for lookup functions
var modelRegistry = map[string]*ServoModel{
	"sts3215":  &STS3215Model,
	"sts3250":  &STS3250Model,
	"scs0009":  &SCS0009Model,
	"sm8512bl": &SM8512BLModel,
}

var modelNumberRegistry = map[int]*ServoModel{
	777:   &STS3215Model,
	2825:  &STS3250Model,
	1284:  &SCS0009Model,
	11272: &SM8512BLModel,
}

// GetModel returns the servo model definition for a given model name
func GetModel(name string) (*ServoModel, bool) {
	model, exists := modelRegistry[name]
	return model, exists
}

// GetModelByNumber returns the servo model definition for a given model number
func GetModelByNumber(number int) (*ServoModel, bool) {
	model, exists := modelNumberRegistry[number]
	return model, exists
}

// GetSupportedModels returns a list of all supported model names
func GetSupportedModels() []string {
	models := make([]string, 0, len(modelRegistry))
	for name := range modelRegistry {
		models = append(models, name)
	}
	return models
}

// GetModelsByProtocol returns all models that use the specified protocol
func GetModelsByProtocol(protocol int) []*ServoModel {
	var models []*ServoModel
	for _, model := range modelRegistry {
		if model.Protocol == protocol {
			models = append(models, model)
		}
	}
	return models
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

// GetRegisterInfo returns address and size information for a register by name
func (m *ServoModel) GetRegisterInfo(registerName string) (AddressInfo, bool) {
	info, exists := m.AddressMap[registerName]
	return info, exists
}

// HasSignMagnitudeEncoding returns true if the register uses sign-magnitude encoding
func (m *ServoModel) HasSignMagnitudeEncoding(registerName string) bool {
	_, exists := m.EncodingBits[registerName]
	return exists
}

// GetSignBit returns the sign bit position for sign-magnitude encoded registers
func (m *ServoModel) GetSignBit(registerName string) (int, bool) {
	bit, exists := m.EncodingBits[registerName]
	return bit, exists
}

// SupportsProtocol returns true if the model supports the given protocol version
func (m *ServoModel) SupportsProtocol(protocol int) bool {
	return m.Protocol == protocol
}

// GetMaxPosition returns the maximum position value for this servo model
func (m *ServoModel) GetMaxPosition() int {
	return m.Resolution - 1
}

// GetCenterPosition returns the center position for this servo model
func (m *ServoModel) GetCenterPosition() int {
	return m.Resolution / 2
}

// ValidatePosition checks if a position value is valid for this servo model
func (m *ServoModel) ValidatePosition(position int) error {
	if position < 0 || position >= m.Resolution {
		return fmt.Errorf("position %d out of range [0, %d) for model %s",
			position, m.Resolution, m.Name)
	}
	return nil
}

// GetRegisterNames returns all available register names for this model
func (m *ServoModel) GetRegisterNames() []string {
	names := make([]string, 0, len(m.AddressMap))
	for name := range m.AddressMap {
		names = append(names, name)
	}
	return names
}

// IsReadOnlyRegister returns true if the register is read-only based on common patterns
func (m *ServoModel) IsReadOnlyRegister(registerName string) bool {
	// Read-only registers typically start with "present_", "firmware_", "model_", "status", "moving"
	readOnlyPrefixes := []string{"present_", "firmware_", "model_", "status", "moving", "sync_write_flag"}

	for _, prefix := range readOnlyPrefixes {
		if len(registerName) >= len(prefix) && registerName[:len(prefix)] == prefix {
			return true
		}
	}

	// Special cases
	readOnlyRegisters := map[string]bool{
		"moving": true,
		"status": true,
	}

	return readOnlyRegisters[registerName]
}

// GetModelInfo returns a summary of the model's capabilities
func (m *ServoModel) GetModelInfo() map[string]interface{} {
	return map[string]interface{}{
		"name":                m.Name,
		"model_number":        m.ModelNumber,
		"protocol":            m.Protocol,
		"resolution":          m.Resolution,
		"max_position":        m.GetMaxPosition(),
		"center_position":     m.GetCenterPosition(),
		"supported_baudrates": m.BaudRates,
		"register_count":      len(m.AddressMap),
		"has_sign_magnitude":  len(m.EncodingBits) > 0,
	}
}
