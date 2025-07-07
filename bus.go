// Communication with Feetech servo motors
package main

import (
	"errors"
	"fmt"
	"maps"
	"sync"
	"time"

	"go.bug.st/serial"
)

// Protocol constants
const (
	ProtocolV0 = 0
	ProtocolV1 = 1
)

// Packet structure constants
const (
	PktHeader1     = 0xFF
	PktHeader2     = 0xFF
	PktIDOffset    = 2
	PktLenOffset   = 3
	PktInstOffset  = 4
	PktErrorOffset = 4
	PktParam0      = 5
)

// Instructions
const (
	InstPing      = 0x01
	InstRead      = 0x02
	InstWrite     = 0x03
	InstRegWrite  = 0x04
	InstAction    = 0x05
	InstSyncWrite = 0x83
	InstSyncRead  = 0x82
)

// Common addresses (STS3215)
const (
	AddrModelNumber     = 3
	AddrID              = 5
	AddrBaudRate        = 6
	AddrTorqueEnable    = 40
	AddrGoalPosition    = 42
	AddrGoalVelocity    = 46
	AddrPresentPosition = 56
	AddrPresentVelocity = 58
	AddrMoving          = 66
)

// Special IDs
const (
	BroadcastID = 0xFE
	MaxID       = 0xFC
)

// Communication results
const (
	CommSuccess   = 0
	CommPortBusy  = -1
	CommTxFail    = -2
	CommRxFail    = -3
	CommTxError   = -4
	CommRxWaiting = -5
	CommRxTimeout = -6
	CommRxCorrupt = -7
	CommNotAvail  = -9
)

// Bus represents a Feetech servo bus
type Bus struct {
	port         serial.Port
	protocol     int
	timeout      time.Duration
	mu           sync.Mutex
	calibrations map[int]*MotorCalibration

	// Command timing
	lastCmdTime time.Time
	minCmdGap   time.Duration
}

// Servo represents an individual servo on the bus
type Servo struct {
	ID    int
	Model string
	bus   *Bus
}

// BusConfig holds configuration for creating a new bus
type BusConfig struct {
	Port         string
	Baudrate     int
	Protocol     int
	Timeout      time.Duration
	Calibrations map[int]*MotorCalibration
}

// NewBus creates a new Feetech servo bus
func NewBus(config BusConfig) (*Bus, error) {
	if config.Baudrate == 0 {
		config.Baudrate = 1000000
	}
	if config.Timeout == 0 {
		config.Timeout = time.Second
	}
	if config.Protocol != ProtocolV0 && config.Protocol != ProtocolV1 {
		return nil, fmt.Errorf("unsupported protocol version: %d", config.Protocol)
	}

	mode := &serial.Mode{
		BaudRate: config.Baudrate,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	}

	port, err := serial.Open(config.Port, mode)
	if err != nil {
		return nil, fmt.Errorf("failed to open port %s: %w", config.Port, err)
	}

	bus := &Bus{
		port:         port,
		protocol:     config.Protocol,
		timeout:      config.Timeout,
		calibrations: make(map[int]*MotorCalibration),
		minCmdGap:    5 * time.Millisecond,
		lastCmdTime:  time.Now(),
	}

	// Load calibrations if provided
	if config.Calibrations != nil {
		maps.Copy(bus.calibrations, config.Calibrations)
	}

	bus.clearBuffers()
	return bus, nil
}

// SetCalibration sets calibration data for a servo
func (b *Bus) SetCalibration(servoID int, calibration *MotorCalibration) {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.calibrations[servoID] = calibration
}

// GetCalibration gets calibration data for a servo
func (b *Bus) GetCalibration(servoID int) (*MotorCalibration, bool) {
	b.mu.Lock()
	defer b.mu.Unlock()
	cal, exists := b.calibrations[servoID]
	return cal, exists
}

// IsCalibrated returns true if the servo has calibration data
func (b *Bus) IsCalibrated(servoID int) bool {
	_, exists := b.GetCalibration(servoID)
	return exists
}

// normalize converts raw servo position to normalized value
func (b *Bus) normalize(servoID int, rawValue int) (float64, error) {
	cal, exists := b.GetCalibration(servoID)
	if !exists {
		return float64(rawValue), nil // Return raw value if no calibration
	}
	return cal.Normalize(rawValue)
}

// denormalize converts normalized value back to raw servo position
func (b *Bus) denormalize(servoID int, normalizedValue float64) (int, error) {
	cal, exists := b.GetCalibration(servoID)
	if !exists {
		return int(normalizedValue), nil // Return as raw value if no calibration
	}
	return cal.Denormalize(normalizedValue)
}

// Close closes the bus connection
func (b *Bus) Close() error {
	b.mu.Lock()
	defer b.mu.Unlock()

	if b.port != nil {
		return b.port.Close()
	}
	return nil
}

// Servo creates a new servo instance for the given ID
func (b *Bus) Servo(id int) *Servo {
	return &Servo{
		ID:    id,
		Model: "sts3215", // Default to STS3215
		bus:   b,
	}
}

// ServoWithModel creates a servo instance with a specific model
func (b *Bus) ServoWithModel(id int, model string) (*Servo, error) {
	// Validate that the model exists
	_, ok := GetModel(model)
	if !ok {
		return nil, fmt.Errorf("unknown servo model: %s", model)
	}

	return &Servo{
		ID:    id,
		Model: model,
		bus:   b,
	}, nil
}

// DetectModel automatically detects and sets the servo model by reading the model number
func (s *Servo) DetectModel() error {
	// Read model number from servo
	data, err := s.readRegister(AddrModelNumber, 2)
	if err != nil {
		return fmt.Errorf("failed to read model number: %w", err)
	}

	modelNumber := s.bus.makeWord(data[0], data[1])

	// Look up model by number
	model, ok := GetModelByNumber(modelNumber)
	if !ok {
		return fmt.Errorf("unknown model number: %d", modelNumber)
	}

	// Set the detected model
	s.Model = model.Name
	return nil
}

// Ping sends a ping to the servo and returns the model number
func (s *Servo) Ping() (int, error) {
	s.bus.mu.Lock()
	defer s.bus.mu.Unlock()

	s.bus.enforceCommandGap()

	packet := []byte{
		PktHeader1, PktHeader2,
		byte(s.ID),
		0x02,
		InstPing,
	}

	checksum := s.bus.calculateChecksum(packet[2:])
	packet = append(packet, checksum)

	if err := s.bus.writePacket(packet); err != nil {
		return 0, err
	}

	response, err := s.bus.readResponse(6)
	if err != nil {
		return 0, err
	}

	if len(response) < 6 || response[PktIDOffset] != byte(s.ID) {
		return 0, fmt.Errorf("invalid ping response from servo %d", s.ID)
	}

	// Read model number
	data, err := s.readRegister(AddrModelNumber, 2)
	if err != nil {
		return 0, err
	}

	return s.bus.makeWord(data[0], data[1]), nil
}

// PingAndDetect pings the servo and automatically detects its model
func (s *Servo) PingAndDetect() (int, error) {
	// First ping to verify communication
	modelNum, err := s.Ping()
	if err != nil {
		return 0, err
	}

	// Look up and set the model
	model, ok := GetModelByNumber(modelNum)
	if ok {
		s.Model = model.Name
	}

	return modelNum, nil
}

// GetModelInfo returns information about the servo's current model
func (s *Servo) GetModelInfo() (map[string]interface{}, error) {
	model, ok := GetModel(s.Model)
	if !ok {
		return nil, fmt.Errorf("unknown servo model: %s", s.Model)
	}

	return model.GetModelInfo(), nil
}

// ReadPosition reads the current position of the servo
func (s *Servo) ReadPosition(normalize bool) (float64, error) {
	data, err := s.readRegister(AddrPresentPosition, 2)
	if err != nil {
		return 0, err
	}
	rawPos := s.bus.makeWord(data[0], data[1])

	if normalize {
		return s.bus.normalize(s.ID, rawPos)
	}
	return float64(rawPos), nil
}

// WritePosition writes a goal position to the servo
func (s *Servo) WritePosition(position float64, normalize bool) error {
	var rawPos int
	var err error

	if normalize {
		rawPos, err = s.bus.denormalize(s.ID, position)
		if err != nil {
			return err
		}
	} else {
		rawPos = int(position)
	}

	data := []byte{
		byte(rawPos & 0xFF),
		byte((rawPos >> 8) & 0xFF),
	}
	return s.writeRegister(AddrGoalPosition, data)
}

// ReadVelocity reads the current velocity of the servo
func (s *Servo) ReadVelocity(normalize bool) (float64, error) {
	data, err := s.readRegister(AddrPresentVelocity, 2)
	if err != nil {
		return 0, err
	}
	rawVel := s.bus.makeWord(data[0], data[1])

	if normalize {
		// For velocity, we use a simple sign-magnitude conversion
		// TODO: Implement proper velocity normalization based on model
		return float64(rawVel), nil
	}
	return float64(rawVel), nil
}

// WriteVelocity writes a goal velocity to the servo
func (s *Servo) WriteVelocity(velocity float64, normalize bool) error {
	var rawVel int

	if normalize {
		// TODO: Implement proper velocity denormalization
		rawVel = int(velocity)
	} else {
		rawVel = int(velocity)
	}

	data := []byte{
		byte(rawVel & 0xFF),
		byte((rawVel >> 8) & 0xFF),
	}
	return s.writeRegister(AddrGoalVelocity, data)
}

// SetTorqueEnable enables or disables torque for the servo
func (s *Servo) SetTorqueEnable(enable bool) error {
	value := byte(0)
	if enable {
		value = 1
	}
	return s.writeRegister(AddrTorqueEnable, []byte{value})
}

// IsMoving returns whether the servo is currently moving
func (s *Servo) IsMoving() (bool, error) {
	data, err := s.readRegister(AddrMoving, 1)
	if err != nil {
		return false, err
	}
	return data[0] != 0, nil
}

// Operating modes for servos
const (
	OperatingModePosition = 0
	OperatingModeVelocity = 1
	OperatingModePWM      = 2
	OperatingModeStep     = 3
)

// SetOperatingMode sets the operating mode of the servo
func (s *Servo) SetOperatingMode(mode byte) error {
	return s.writeRegister(33, []byte{mode}) // Address 33 is Operating_Mode
}

// GetOperatingMode gets the current operating mode of the servo
func (s *Servo) GetOperatingMode() (byte, error) {
	data, err := s.readRegister(33, 1) // Address 33 is Operating_Mode
	if err != nil {
		return 0, err
	}
	return data[0], nil
}

// ReadRegisterByName reads a register using its name from the servo model
func (s *Servo) ReadRegisterByName(name string) ([]byte, error) {
	model, ok := GetModel(s.Model)
	if !ok {
		return nil, fmt.Errorf("unknown servo model: %s", s.Model)
	}

	addrInfo, ok := model.GetRegisterInfo(name)
	if !ok {
		return nil, fmt.Errorf("unknown register: %s", name)
	}

	return s.readRegister(addrInfo.Address, addrInfo.Size)
}

// WriteRegisterByName writes to a register using its name from the servo model
func (s *Servo) WriteRegisterByName(name string, data []byte) error {
	model, ok := GetModel(s.Model)
	if !ok {
		return fmt.Errorf("unknown servo model: %s", s.Model)
	}

	addrInfo, ok := model.GetRegisterInfo(name)
	if !ok {
		return fmt.Errorf("unknown register: %s", name)
	}

	if len(data) != addrInfo.Size {
		return fmt.Errorf("data size mismatch: expected %d, got %d", addrInfo.Size, len(data))
	}

	// Check if register is read-only
	if model.IsReadOnlyRegister(name) {
		return fmt.Errorf("register %s is read-only", name)
	}

	return s.writeRegister(addrInfo.Address, data)
}

// SyncWritePositions writes positions to multiple servos simultaneously
func (b *Bus) SyncWritePositions(servos []*Servo, positions []float64, normalize bool) error {
	if len(servos) != len(positions) {
		return errors.New("servo and position arrays must have same length")
	}

	b.mu.Lock()
	defer b.mu.Unlock()

	b.enforceCommandGap()

	// Convert positions to raw values
	rawPositions := make([]int, len(positions))
	for i, pos := range positions {
		if normalize {
			rawPos, err := b.denormalize(servos[i].ID, pos)
			if err != nil {
				return fmt.Errorf("failed to denormalize position for servo %d: %w", servos[i].ID, err)
			}
			rawPositions[i] = rawPos
		} else {
			rawPositions[i] = int(pos)
		}
	}

	dataLen := 2
	paramLen := len(servos) * (1 + dataLen)

	packet := []byte{
		PktHeader1, PktHeader2,
		BroadcastID,
		byte(4 + paramLen),
		InstSyncWrite,
		AddrGoalPosition,
		byte(dataLen),
	}

	for i, servo := range servos {
		position := rawPositions[i]
		packet = append(packet, byte(servo.ID))
		packet = append(packet, byte(position&0xFF))
		packet = append(packet, byte((position>>8)&0xFF))
	}

	checksum := b.calculateChecksum(packet[2:])
	packet = append(packet, checksum)

	return b.writePacket(packet)
}

// SyncReadPositions reads positions from multiple servos simultaneously
// Note: Protocol 1 doesn't support sync read, will fall back to individual reads
func (b *Bus) SyncReadPositions(servos []*Servo, normalize bool) (map[int]float64, error) {
	if b.protocol == ProtocolV1 {
		// Fallback to individual reads for Protocol 1
		return b.syncReadPositionsFallback(servos, normalize)
	}

	return b.syncReadPositionsProtocol0(servos, normalize)
}

// syncReadPositionsFallback reads positions individually for Protocol 1
func (b *Bus) syncReadPositionsFallback(servos []*Servo, normalize bool) (map[int]float64, error) {
	result := make(map[int]float64)

	for _, servo := range servos {
		pos, err := servo.ReadPosition(normalize)
		if err != nil {
			return nil, fmt.Errorf("failed to read position from servo %d: %w", servo.ID, err)
		}
		result[servo.ID] = pos
	}

	return result, nil
}

// syncReadPositionsProtocol0 implements true sync read for Protocol 0
func (b *Bus) syncReadPositionsProtocol0(servos []*Servo, normalize bool) (map[int]float64, error) {
	b.mu.Lock()
	defer b.mu.Unlock()

	b.enforceCommandGap()

	// Create sync read packet
	dataLen := 2            // Reading 2 bytes for position
	paramLen := len(servos) // One ID per servo

	packet := []byte{
		PktHeader1, PktHeader2,
		BroadcastID,
		byte(4 + paramLen),
		InstSyncRead,
		AddrPresentPosition,
		byte(dataLen),
	}

	// Add servo IDs
	for _, servo := range servos {
		packet = append(packet, byte(servo.ID))
	}

	checksum := b.calculateChecksum(packet[2:])
	packet = append(packet, checksum)

	if err := b.writePacket(packet); err != nil {
		return nil, err
	}

	// Read responses
	result := make(map[int]float64)
	expectedResponseLen := 6 + dataLen // Header + ID + Length + Error + Data + Checksum

	for i := range servos {
		response, err := b.readResponse(expectedResponseLen)
		if err != nil {
			return nil, fmt.Errorf("failed to read sync response %d: %w", i, err)
		}

		if len(response) < expectedResponseLen {
			return nil, fmt.Errorf("sync response %d too short: got %d, expected %d", i, len(response), expectedResponseLen)
		}

		servoID := int(response[PktIDOffset])
		if !b.verifyChecksum(response) {
			return nil, fmt.Errorf("checksum verification failed for servo %d", servoID)
		}

		// Extract position data
		rawPos := b.makeWord(response[PktParam0], response[PktParam0+1])

		var normalizedPos float64
		var err2 error
		if normalize {
			normalizedPos, err2 = b.normalize(servoID, rawPos)
			if err2 != nil {
				return nil, fmt.Errorf("failed to normalize position for servo %d: %w", servoID, err2)
			}
		} else {
			normalizedPos = float64(rawPos)
		}

		result[servoID] = normalizedPos
	}

	return result, nil
}

// SyncRead performs a generic sync read operation
func (b *Bus) SyncRead(address byte, dataLength int, servoIDs []int) (map[int][]byte, error) {
	if b.protocol == ProtocolV1 {
		return nil, fmt.Errorf("sync read not supported in Protocol 1")
	}

	b.mu.Lock()
	defer b.mu.Unlock()

	b.enforceCommandGap()

	paramLen := len(servoIDs)
	packet := []byte{
		PktHeader1, PktHeader2,
		BroadcastID,
		byte(4 + paramLen),
		InstSyncRead,
		address,
		byte(dataLength),
	}

	// Add servo IDs
	for _, id := range servoIDs {
		packet = append(packet, byte(id))
	}

	checksum := b.calculateChecksum(packet[2:])
	packet = append(packet, checksum)

	if err := b.writePacket(packet); err != nil {
		return nil, err
	}

	// Read responses
	result := make(map[int][]byte)
	expectedResponseLen := 6 + dataLength

	for i := range servoIDs {
		response, err := b.readResponse(expectedResponseLen)
		if err != nil {
			return nil, fmt.Errorf("failed to read sync response %d: %w", i, err)
		}

		if len(response) < expectedResponseLen {
			return nil, fmt.Errorf("sync response %d too short", i)
		}

		servoID := int(response[PktIDOffset])
		if !b.verifyChecksum(response) {
			return nil, fmt.Errorf("checksum verification failed for servo %d", servoID)
		}

		// Extract data
		data := make([]byte, dataLength)
		copy(data, response[PktParam0:PktParam0+dataLength])
		result[servoID] = data
	}

	return result, nil
}

// readRegister reads data from a servo register
func (s *Servo) readRegister(address byte, length int) ([]byte, error) {
	s.bus.mu.Lock()
	defer s.bus.mu.Unlock()

	s.bus.enforceCommandGap()

	packet := []byte{
		PktHeader1, PktHeader2,
		byte(s.ID),
		0x04,
		InstRead,
		address,
		byte(length),
	}

	checksum := s.bus.calculateChecksum(packet[2:])
	packet = append(packet, checksum)

	if err := s.bus.writePacket(packet); err != nil {
		return nil, err
	}

	expectedLen := 6 + length
	response, err := s.bus.readResponse(expectedLen)
	if err != nil {
		return nil, err
	}

	if len(response) < expectedLen {
		return nil, fmt.Errorf("response too short: got %d, expected %d", len(response), expectedLen)
	}

	if response[PktIDOffset] != byte(s.ID) {
		return nil, fmt.Errorf("wrong servo ID in response: got %d, expected %d", response[PktIDOffset], s.ID)
	}

	if !s.bus.verifyChecksum(response) {
		return nil, errors.New("checksum verification failed")
	}

	data := make([]byte, length)
	copy(data, response[PktParam0:PktParam0+length])
	return data, nil
}

// writeRegister writes data to a servo register
func (s *Servo) writeRegister(address byte, data []byte) error {
	s.bus.mu.Lock()
	defer s.bus.mu.Unlock()

	s.bus.enforceCommandGap()

	packet := []byte{
		PktHeader1, PktHeader2,
		byte(s.ID),
		byte(3 + len(data)),
		InstWrite,
		address,
	}

	packet = append(packet, data...)

	checksum := s.bus.calculateChecksum(packet[2:])
	packet = append(packet, checksum)

	if err := s.bus.writePacket(packet); err != nil {
		return err
	}

	// Read status packet for write operations
	response, err := s.bus.readResponse(6)
	if err != nil {
		return err
	}

	if response[PktIDOffset] != byte(s.ID) {
		return fmt.Errorf("wrong servo ID in response: got %d, expected %d", response[PktIDOffset], s.ID)
	}

	return nil
}

// Helper methods for the bus

func (b *Bus) clearBuffers() {
	if b.port == nil {
		return
	}

	// Set short timeout for clearing
	b.port.SetReadTimeout(10 * time.Millisecond)
	buffer := make([]byte, 256)

	for {
		n, err := b.port.Read(buffer)
		if err != nil || n == 0 {
			break
		}
	}

	// Restore normal timeout
	b.port.SetReadTimeout(b.timeout)
}

func (b *Bus) enforceCommandGap() {
	elapsed := time.Since(b.lastCmdTime)
	if elapsed < b.minCmdGap {
		time.Sleep(b.minCmdGap - elapsed)
	}
	b.lastCmdTime = time.Now()
}

func (b *Bus) writePacket(packet []byte) error {
	b.clearBuffers()

	n, err := b.port.Write(packet)
	if err != nil {
		return fmt.Errorf("failed to write packet: %w", err)
	}
	if n != len(packet) {
		return fmt.Errorf("incomplete write: wrote %d of %d bytes", n, len(packet))
	}

	time.Sleep(2 * time.Millisecond)
	return nil
}

func (b *Bus) readResponse(expectedLen int) ([]byte, error) {
	buffer := make([]byte, expectedLen*2)
	totalRead := 0
	startTime := time.Now()

	for totalRead < expectedLen {
		if time.Since(startTime) > b.timeout {
			return nil, fmt.Errorf("timeout reading response after %v", b.timeout)
		}

		n, err := b.port.Read(buffer[totalRead:])
		if err != nil {
			return nil, fmt.Errorf("failed to read response: %w", err)
		}

		if n == 0 {
			if time.Since(startTime) > 100*time.Millisecond {
				break
			}
			time.Sleep(5 * time.Millisecond)
			continue
		}

		totalRead += n

		// Look for packet header and validate packet
		if totalRead >= 6 {
			for i := 0; i <= totalRead-6; i++ {
				if buffer[i] == PktHeader1 && buffer[i+1] == PktHeader2 {
					if i+3 < totalRead {
						packetLength := int(buffer[i+3]) + 4
						if i+packetLength <= totalRead {
							response := make([]byte, packetLength)
							copy(response, buffer[i:i+packetLength])
							return response, nil
						}
					}
				}
			}
		}
	}

	if totalRead == 0 {
		return nil, errors.New("no response received")
	}

	response := make([]byte, totalRead)
	copy(response, buffer[:totalRead])
	return response, nil
}

func (b *Bus) calculateChecksum(packet []byte) byte {
	checksum := byte(0)
	for _, b := range packet {
		checksum += b
	}
	return ^checksum
}

func (b *Bus) verifyChecksum(packet []byte) bool {
	if len(packet) < 4 {
		return false
	}
	checksum := b.calculateChecksum(packet[2 : len(packet)-1])
	return checksum == packet[len(packet)-1]
}

func (b *Bus) makeWord(low, high byte) int {
	if b.protocol == ProtocolV0 {
		return int(low) | (int(high) << 8)
	}
	return int(high) | (int(low) << 8)
}
