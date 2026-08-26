package feetech

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"time"

	"github.com/hipsterbrown/feetech-servo/transports"
)

// Bus manages communication with servos on a Feetech bus.
type Bus struct {
	transport Transport
	protocol  *Protocol
	timeout   time.Duration

	mu          sync.Mutex
	lastCmdTime time.Time
	minCmdGap   time.Duration
	pingTimeout time.Duration
	closed      bool
}

// BusConfig holds configuration for creating a new Bus.
type BusConfig struct {
	// Transport is the underlying communication transport.
	// If nil, Port must be specified to open a serial connection.
	Transport Transport

	// Port is the serial port path (e.g., "/dev/ttyUSB0").
	// Ignored if Transport is provided.
	Port string

	// BaudRate is the communication speed. Default is 1000000.
	BaudRate int

	// Protocol version: ProtocolSTS (default) or ProtocolSCS.
	Protocol ProtocolVersion

	// Timeout for communication operations. Default is 1 second.
	Timeout time.Duration

	// MinCommandGap is the minimum time between commands. Default is 1ms.
	MinCommandGap time.Duration

	// PingTimeout bounds how long Scan/Discover wait for each per-ID ping
	// before treating that ID as absent. Default is 30ms. Keep it short so
	// sweeping the full ID space stays fast on a mostly-empty bus.
	PingTimeout time.Duration
}

// NewBus creates a new servo bus with the given configuration.
func NewBus(cfg BusConfig) (*Bus, error) {
	// Set defaults
	if cfg.BaudRate == 0 {
		cfg.BaudRate = 1000000
	}
	if cfg.Timeout == 0 {
		cfg.Timeout = time.Second
	}
	if cfg.MinCommandGap == 0 {
		cfg.MinCommandGap = time.Millisecond
	}
	if cfg.PingTimeout == 0 {
		cfg.PingTimeout = 30 * time.Millisecond
	}

	// Get or create transport
	transport := cfg.Transport
	if transport == nil {
		if cfg.Port == "" {
			return nil, errors.New("either Transport or Port must be specified")
		}
		var err error
		transport, err = transports.OpenSerial(transports.SerialConfig{
			Port:     cfg.Port,
			BaudRate: cfg.BaudRate,
			Timeout:  cfg.Timeout,
		})
		if err != nil {
			return nil, fmt.Errorf("failed to open serial port: %w", err)
		}
	}

	return &Bus{
		transport:   transport,
		protocol:    NewProtocol(cfg.Protocol),
		timeout:     cfg.Timeout,
		minCmdGap:   cfg.MinCommandGap,
		pingTimeout: cfg.PingTimeout,
		lastCmdTime: time.Now(),
	}, nil
}

// Close closes the bus and releases resources.
func (b *Bus) Close() error {
	b.mu.Lock()
	defer b.mu.Unlock()

	if b.closed {
		return nil
	}
	b.closed = true

	return b.transport.Close()
}

// Protocol returns the protocol handler for this bus.
func (b *Bus) Protocol() *Protocol {
	return b.protocol
}

// Ping sends a ping to the specified servo and returns the model number.
func (b *Bus) Ping(ctx context.Context, id int) (int, error) {
	if err := b.validateID(id); err != nil {
		return 0, err
	}

	b.mu.Lock()
	defer b.mu.Unlock()

	if b.closed {
		return 0, ErrBusClosed
	}

	// Send ping packet
	packet := b.protocol.PingPacket(byte(id))
	if err := b.sendPacketLocked(packet); err != nil {
		return 0, &CommError{Op: "ping", Err: err}
	}

	// Read response
	resp, err := b.readResponseLocked(ctx, 6)
	if err != nil {
		return 0, &ServoError{ID: id, Op: "ping", Err: err}
	}

	if resp.Error.HasError() {
		return 0, &ServoError{ID: id, Op: "ping", Status: resp.Error}
	}

	// Now read model number
	modelData, err := b.readRegisterLocked(ctx, byte(id), RegModelNumber.Address, byte(RegModelNumber.Size))
	if err != nil {
		return 0, &ServoError{ID: id, Op: "read model", Err: err}
	}

	return int(b.protocol.DecodeWord(modelData)), nil
}

// ReadRegister reads bytes from a servo register.
//
// A servo condition flag (overload, overheat, voltage, angle limit) returns
// BOTH the payload and a non-nil error: the servo answered, and the flag
// describes the motor rather than the validity of the data. Callers that treat
// any error as fatal are unaffected; callers that want the reading anyway
// should check ConditionStatus(err). Request-rejection flags (checksum,
// instruction, range) return nil data.
func (b *Bus) ReadRegister(ctx context.Context, id int, address byte, length int) ([]byte, error) {
	if err := b.validateID(id); err != nil {
		return nil, err
	}

	b.mu.Lock()
	defer b.mu.Unlock()

	if b.closed {
		return nil, ErrBusClosed
	}

	return b.readRegisterLocked(ctx, byte(id), address, byte(length))
}

// WriteRegister writes bytes to a servo register.
func (b *Bus) WriteRegister(ctx context.Context, id int, address byte, data []byte) error {
	if err := b.validateID(id); err != nil {
		return err
	}

	b.mu.Lock()
	defer b.mu.Unlock()

	if b.closed {
		return ErrBusClosed
	}

	return b.writeRegisterLocked(ctx, byte(id), address, data)
}

// SyncWrite writes data to multiple servos simultaneously.
// servoData maps servo ID to the data to write.
func (b *Bus) SyncWrite(ctx context.Context, address byte, dataLen int, servoData map[int][]byte) error {
	b.mu.Lock()
	defer b.mu.Unlock()

	if b.closed {
		return ErrBusClosed
	}

	// Convert to byte keys
	byteData := make(map[byte][]byte, len(servoData))
	for id, data := range servoData {
		if err := b.validateID(id); err != nil {
			return err
		}
		if len(data) != dataLen {
			return fmt.Errorf("servo %d: data length mismatch: expected %d, got %d", id, dataLen, len(data))
		}
		byteData[byte(id)] = data
	}

	packet := b.protocol.SyncWritePacket(address, byte(dataLen), byteData)
	if err := b.sendPacketLocked(packet); err != nil {
		return &CommError{Op: "sync_write", Err: err}
	}

	// Sync write to broadcast ID gets no response
	return nil
}

// SyncRead reads data from multiple servos simultaneously.
// Returns a map of servo ID to the data read.
// Note: Only supported in ProtocolSTS.
func (b *Bus) SyncRead(ctx context.Context, address byte, dataLen int, ids []int) (map[int][]byte, error) {
	if b.protocol.Version() == ProtocolSCS {
		return nil, errors.New("sync read not supported in SCS protocol")
	}

	b.mu.Lock()
	defer b.mu.Unlock()

	if b.closed {
		return nil, ErrBusClosed
	}

	// Convert to byte IDs
	byteIDs := make([]byte, len(ids))
	for i, id := range ids {
		if err := b.validateID(id); err != nil {
			return nil, err
		}
		byteIDs[i] = byte(id)
	}

	// Send sync read packet
	packet := b.protocol.SyncReadPacket(address, byte(dataLen), byteIDs)
	if err := b.sendPacketLocked(packet); err != nil {
		return nil, &CommError{Op: "sync_read", Err: err}
	}

	// Read all responses
	expectedLen := len(ids) * b.protocol.ExpectedResponseLength(dataLen)
	rawData, err := b.readRawBytesLocked(ctx, expectedLen)
	if err != nil {
		return nil, &CommError{Op: "sync_read", Err: err}
	}

	// Parse responses
	packets, err := b.protocol.DecodeMultiple(rawData, len(ids))
	if err != nil {
		return nil, &CommError{Op: "sync_read", Err: err}
	}

	// Build result map
	result := make(map[int][]byte, len(packets))
	for _, pkt := range packets {
		if pkt.Error.HasError() {
			return nil, &ServoError{ID: int(pkt.ID), Op: "sync_read", Status: pkt.Error}
		}
		result[int(pkt.ID)] = pkt.Parameters
	}

	// Check for missing responses
	for _, id := range ids {
		if _, ok := result[id]; !ok {
			return result, &ServoError{ID: id, Op: "sync_read", Err: ErrNoResponse}
		}
	}

	return result, nil
}

// RegWrite writes data to a servo's buffer without immediate execution.
// Call Action() to execute all buffered writes.
func (b *Bus) RegWrite(ctx context.Context, id int, address byte, data []byte) error {
	if err := b.validateID(id); err != nil {
		return err
	}

	b.mu.Lock()
	defer b.mu.Unlock()

	if b.closed {
		return ErrBusClosed
	}

	packet := b.protocol.RegWritePacket(byte(id), address, data)
	if err := b.sendPacketLocked(packet); err != nil {
		return &CommError{Op: "reg_write", Err: err}
	}

	// Read response
	resp, err := b.readResponseLocked(ctx, 6)
	if err != nil {
		return &ServoError{ID: id, Op: "reg_write", Err: err}
	}

	if resp.Error.HasError() {
		return &ServoError{ID: id, Op: "reg_write", Status: resp.Error}
	}

	return nil
}

// Action triggers execution of all buffered RegWrite commands.
func (b *Bus) Action(ctx context.Context) error {
	b.mu.Lock()
	defer b.mu.Unlock()

	if b.closed {
		return ErrBusClosed
	}

	packet := b.protocol.ActionPacket()
	if err := b.sendPacketLocked(packet); err != nil {
		return &CommError{Op: "action", Err: err}
	}

	// Broadcast, no response expected
	return nil
}

// Scan searches for servos by pinging each ID in the range.
func (b *Bus) Scan(ctx context.Context, startID, endID int) ([]FoundServo, error) {
	if startID < 0 || endID > int(MaxServoID) || startID > endID {
		return nil, fmt.Errorf("invalid ID range: %d to %d", startID, endID)
	}

	var found []FoundServo

	for id := startID; id <= endID; id++ {
		select {
		case <-ctx.Done():
			return found, ctx.Err()
		default:
		}

		modelNum, err := b.pingWithTimeout(ctx, id)
		if err != nil {
			continue // No response at this ID
		}

		f := FoundServo{
			ID:          id,
			ModelNumber: modelNum,
		}

		if model, ok := GetModelByNumber(modelNum); ok {
			f.Model = model
		}

		found = append(found, f)
	}

	return found, nil
}

// pingWithTimeout pings id, bounding the wait to pingTimeout so absent IDs are
// skipped quickly during a sweep.
func (b *Bus) pingWithTimeout(ctx context.Context, id int) (int, error) {
	if b.pingTimeout > 0 {
		var cancel context.CancelFunc
		ctx, cancel = context.WithTimeout(ctx, b.pingTimeout)
		defer cancel()
	}
	return b.Ping(ctx, id)
}

// Discover finds every servo on the bus by pinging each ID in turn. It works
// for both STS and SCS protocols and reliably finds all servos (unlike a
// broadcast ping, whose simultaneous responses collide on a multi-servo bus).
// Each absent ID costs one PingTimeout; tune BusConfig.PingTimeout for the
// speed/coverage trade-off. For an explicit fast-but-lossy single broadcast
// ping, see BroadcastPing.
func (b *Bus) Discover(ctx context.Context) ([]FoundServo, error) {
	return b.Scan(ctx, 1, int(MaxServoID))
}

// BroadcastPing discovers servos with a single broadcast ping (0xFE), STS only.
//
// WARNING: the Feetech protocol manual forbids broadcast ping when more than one
// servo shares the bus — every servo answers at once and the responses collide
// on the half-duplex line, so this typically returns only one of several
// servos. Prefer Discover (or Scan) for reliable discovery; use BroadcastPing
// only when you know a single servo is attached and want the fastest probe.
func (b *Bus) BroadcastPing(ctx context.Context) ([]FoundServo, error) {
	if b.protocol.Version() != ProtocolSTS {
		return nil, errors.New("broadcast ping only supported in STS protocol")
	}

	b.mu.Lock()
	defer b.mu.Unlock()

	if b.closed {
		return nil, ErrBusClosed
	}

	packet := b.protocol.PingPacket(BroadcastID)
	if err := b.sendPacketLocked(packet); err != nil {
		return nil, &CommError{Op: "broadcast ping", Err: err}
	}

	var found []FoundServo
	deadline := time.Now().Add(b.timeout)

	// Collect whatever responses make it back uncorrupted.
	for time.Now().Before(deadline) {
		select {
		case <-ctx.Done():
			return found, ctx.Err()
		default:
		}

		data, err := b.readRawBytesLocked(ctx, 6)
		if err != nil {
			break // No more responses available
		}

		pkt, _, err := b.protocol.Decode(data)
		if err != nil {
			continue // Invalid/collided packet
		}
		if pkt.Error.HasError() {
			continue
		}

		servoID := int(pkt.ID)
		modelData, err := b.readRegisterLocked(ctx, byte(servoID), RegModelNumber.Address, byte(RegModelNumber.Size))
		if err != nil {
			continue
		}
		modelNum := int(b.protocol.DecodeWord(modelData))

		f := FoundServo{ID: servoID, ModelNumber: modelNum}
		if model, ok := GetModelByNumber(modelNum); ok {
			f.Model = model
		}
		found = append(found, f)
	}

	return found, nil
}

// FoundServo represents a servo discovered during scanning.
type FoundServo struct {
	ID          int
	ModelNumber int
	Model       *Model // May be nil if model is unknown
}

// Internal methods

func (b *Bus) validateID(id int) error {
	if id < 0 || id > int(MaxServoID) {
		return fmt.Errorf("%w: %d (valid range: 0-%d)", ErrInvalidID, id, MaxServoID)
	}
	return nil
}

func (b *Bus) enforceCommandGap() {
	elapsed := time.Since(b.lastCmdTime)
	if elapsed < b.minCmdGap {
		time.Sleep(b.minCmdGap - elapsed)
	}
}

func (b *Bus) sendPacketLocked(packet []byte) error {
	b.enforceCommandGap()

	// Flush any stale input
	b.transport.Flush()

	n, err := b.transport.Write(packet)
	if err != nil {
		return fmt.Errorf("write failed: %w", err)
	}
	if n != len(packet) {
		return fmt.Errorf("incomplete write: %d of %d bytes", n, len(packet))
	}

	b.lastCmdTime = time.Now()

	// Small delay for half-duplex turnaround
	time.Sleep(100 * time.Microsecond)

	return nil
}

func (b *Bus) readRegisterLocked(ctx context.Context, id, address, length byte) ([]byte, error) {
	packet := b.protocol.ReadPacket(id, address, length)
	if err := b.sendPacketLocked(packet); err != nil {
		return nil, err
	}

	resp, err := b.readResponseLocked(ctx, b.protocol.ExpectedResponseLength(int(length)))
	if err != nil {
		return nil, err
	}

	if resp.ID != id {
		return nil, fmt.Errorf("wrong servo ID in response: expected %d, got %d", id, resp.ID)
	}

	// A condition flag (overload/overheat/voltage/angle limit) describes the
	// motor, not the validity of the response: return the payload AND the flag
	// and let the caller decide. Callers that check `if err != nil` are
	// unaffected. See splitStatus.
	payloadValid, statusErr := splitStatus(resp.Error)
	if !payloadValid {
		return nil, statusErr
	}

	return resp.Parameters, statusErr
}

func (b *Bus) writeRegisterLocked(ctx context.Context, id, address byte, data []byte) error {
	packet := b.protocol.WritePacket(id, address, data)
	if err := b.sendPacketLocked(packet); err != nil {
		return err
	}

	resp, err := b.readResponseLocked(ctx, 6)
	if err != nil {
		return err
	}

	if resp.ID != id {
		return fmt.Errorf("wrong servo ID in response: expected %d, got %d", id, resp.ID)
	}

	if resp.Error.HasError() {
		return resp.Error
	}

	return nil
}

func (b *Bus) readResponseLocked(ctx context.Context, expectedLen int) (Packet, error) {
	data, err := b.readRawBytesLocked(ctx, expectedLen)
	if err != nil {
		return Packet{}, err
	}

	pkt, _, err := b.protocol.Decode(data)
	return pkt, err
}

func (b *Bus) readRawBytesLocked(ctx context.Context, expectedLen int) ([]byte, error) {
	buffer := make([]byte, expectedLen*2) // Extra space for safety
	totalRead := 0
	deadline := time.Now().Add(b.timeout)
	// Honor a sooner context deadline so per-ping timeouts (Scan/Discover) bound
	// each blocking read rather than waiting the full bus timeout per absent ID.
	if d, ok := ctx.Deadline(); ok && d.Before(deadline) {
		deadline = d
	}

	for totalRead < expectedLen {
		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		default:
		}

		if time.Now().After(deadline) {
			if totalRead == 0 {
				return nil, ErrNoResponse
			}
			return nil, fmt.Errorf("%w: read %d of %d expected bytes", ErrTimeout, totalRead, expectedLen)
		}

		remaining := max(time.Until(deadline), 10*time.Millisecond)
		b.transport.SetReadTimeout(remaining)

		n, err := b.transport.Read(buffer[totalRead:])
		if err != nil {
			// Check if it's a timeout (expected when waiting)
			if n == 0 {
				time.Sleep(time.Millisecond)
				continue
			}
			return nil, fmt.Errorf("read error: %w", err)
		}

		totalRead += n
	}

	return buffer[:totalRead], nil
}
