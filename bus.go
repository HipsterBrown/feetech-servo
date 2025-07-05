// Provides communication with Feetech servo motgrs
package main

import (
	"errors"
	"fmt"
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
	port     serial.Port
	protocol int
	timeout  time.Duration
	mu       sync.Mutex

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
	Port     string
	Baudrate int
	Protocol int
	Timeout  time.Duration
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
		port:        port,
		protocol:    config.Protocol,
		timeout:     config.Timeout,
		minCmdGap:   5 * time.Millisecond,
		lastCmdTime: time.Now(),
	}

	bus.clearBuffers()
	return bus, nil
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

// ReadPosition reads the current position of the servo
func (s *Servo) ReadPosition() (int, error) {
	data, err := s.readRegister(AddrPresentPosition, 2)
	if err != nil {
		return 0, err
	}
	return s.bus.makeWord(data[0], data[1]), nil
}

// WritePosition writes a goal position to the servo
func (s *Servo) WritePosition(position int) error {
	data := []byte{
		byte(position & 0xFF),
		byte((position >> 8) & 0xFF),
	}
	return s.writeRegister(AddrGoalPosition, data)
}

// ReadVelocity reads the current velocity of the servo
func (s *Servo) ReadVelocity() (int, error) {
	data, err := s.readRegister(AddrPresentVelocity, 2)
	if err != nil {
		return 0, err
	}
	return s.bus.makeWord(data[0], data[1]), nil
}

// WriteVelocity writes a goal velocity to the servo
func (s *Servo) WriteVelocity(velocity int) error {
	data := []byte{
		byte(velocity & 0xFF),
		byte((velocity >> 8) & 0xFF),
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

// SyncWritePositions writes positions to multiple servos simultaneously
func (b *Bus) SyncWritePositions(servos []*Servo, positions []int) error {
	if len(servos) != len(positions) {
		return errors.New("servo and position arrays must have same length")
	}

	b.mu.Lock()
	defer b.mu.Unlock()

	b.enforceCommandGap()

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
		position := positions[i]
		packet = append(packet, byte(servo.ID))
		packet = append(packet, byte(position&0xFF))
		packet = append(packet, byte((position>>8)&0xFF))
	}

	checksum := b.calculateChecksum(packet[2:])
	packet = append(packet, checksum)

	return b.writePacket(packet)
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
