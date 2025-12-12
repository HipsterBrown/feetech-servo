// Package feetech provides a Go library for communicating with Feetech servo motors.
package feetech

import (
	"encoding/binary"
	"errors"
	"fmt"
)

// Protocol version constants.
const (
	ProtocolSTS = iota // STS/SMS series: little-endian, TTL level
	ProtocolSCS        // SCS series: big-endian, TTL level
)

// Instruction codes per the Feetech protocol specification.
const (
	InstPing      byte = 0x01
	InstRead      byte = 0x02
	InstWrite     byte = 0x03
	InstRegWrite  byte = 0x04
	InstAction    byte = 0x05
	InstReset     byte = 0x06
	InstSyncRead  byte = 0x82
	InstSyncWrite byte = 0x83
)

// Special ID values.
const (
	BroadcastID = 0xFE
	MaxServoID  = 0xFD
)

// Packet header bytes.
const (
	headerByte1 = 0xFF
	headerByte2 = 0xFF
)

// Status error flags returned by servos.
type StatusError byte

const (
	ErrVoltage     StatusError = 1 << 0
	ErrAngleLimit  StatusError = 1 << 1
	ErrOverheat    StatusError = 1 << 2
	ErrRange       StatusError = 1 << 3
	ErrChecksum    StatusError = 1 << 4
	ErrOverload    StatusError = 1 << 5
	ErrInstruction StatusError = 1 << 6
)

func (e StatusError) Error() string {
	if e == 0 {
		return "no error"
	}

	var msgs []string
	if e&ErrVoltage != 0 {
		msgs = append(msgs, "voltage")
	}
	if e&ErrAngleLimit != 0 {
		msgs = append(msgs, "angle limit")
	}
	if e&ErrOverheat != 0 {
		msgs = append(msgs, "overheat")
	}
	if e&ErrRange != 0 {
		msgs = append(msgs, "range")
	}
	if e&ErrChecksum != 0 {
		msgs = append(msgs, "checksum")
	}
	if e&ErrOverload != 0 {
		msgs = append(msgs, "overload")
	}
	if e&ErrInstruction != 0 {
		msgs = append(msgs, "instruction")
	}

	return fmt.Sprintf("servo status error: %v", msgs)
}

// HasError returns true if any error flag is set.
func (e StatusError) HasError() bool {
	return e != 0
}

// Packet represents a Feetech protocol packet.
type Packet struct {
	ID          byte
	Instruction byte
	Parameters  []byte
	Error       StatusError // Only valid for response packets
}

// Protocol handles packet encoding/decoding for a specific protocol version.
type Protocol struct {
	version   int
	byteOrder binary.ByteOrder
}

// NewProtocol creates a protocol handler for the specified version.
func NewProtocol(version int) *Protocol {
	p := &Protocol{version: version}
	if version == ProtocolSCS {
		p.byteOrder = binary.BigEndian
	} else {
		p.byteOrder = binary.LittleEndian
	}
	return p
}

// ByteOrder returns the byte order for multi-byte values.
func (p *Protocol) ByteOrder() binary.ByteOrder {
	return p.byteOrder
}

// Version returns the protocol version.
func (p *Protocol) Version() int {
	return p.version
}

// EncodeWord converts a 16-bit value to bytes in protocol byte order.
func (p *Protocol) EncodeWord(value uint16) []byte {
	buf := make([]byte, 2)
	p.byteOrder.PutUint16(buf, value)
	return buf
}

// DecodeWord converts bytes to a 16-bit value using protocol byte order.
func (p *Protocol) DecodeWord(data []byte) uint16 {
	if len(data) < 2 {
		return 0
	}
	return p.byteOrder.Uint16(data)
}

// Encode constructs a wire-format packet from the given components.
func (p *Protocol) Encode(pkt Packet) []byte {
	length := byte(len(pkt.Parameters) + 2) // params + instruction + checksum

	// Build packet: header(2) + id(1) + length(1) + instruction(1) + params(n) + checksum(1)
	buf := make([]byte, 0, 6+len(pkt.Parameters))
	buf = append(buf, headerByte1, headerByte2)
	buf = append(buf, pkt.ID)
	buf = append(buf, length)
	buf = append(buf, pkt.Instruction)
	buf = append(buf, pkt.Parameters...)

	checksum := p.calculateChecksum(buf[2:]) // From ID onwards
	buf = append(buf, checksum)

	return buf
}

// Decode parses a wire-format packet into its components.
// Returns the packet and number of bytes consumed, or an error.
func (p *Protocol) Decode(data []byte) (Packet, int, error) {
	if len(data) < 6 {
		return Packet{}, 0, errors.New("packet too short")
	}

	// Find header
	headerIdx := -1
	for i := 0; i <= len(data)-6; i++ {
		if data[i] == headerByte1 && data[i+1] == headerByte2 {
			headerIdx = i
			break
		}
	}
	if headerIdx < 0 {
		return Packet{}, 0, errors.New("header not found")
	}

	data = data[headerIdx:]
	if len(data) < 6 {
		return Packet{}, 0, errors.New("packet too short after header")
	}

	id := data[2]
	length := int(data[3])

	totalLen := 4 + length // header(2) + id(1) + length(1) + [length bytes]
	if len(data) < totalLen {
		return Packet{}, 0, fmt.Errorf("incomplete packet: need %d bytes, have %d", totalLen, len(data))
	}

	// Verify checksum
	expectedChecksum := p.calculateChecksum(data[2 : totalLen-1])
	actualChecksum := data[totalLen-1]
	if expectedChecksum != actualChecksum {
		return Packet{}, 0, fmt.Errorf("checksum mismatch: expected 0x%02X, got 0x%02X", expectedChecksum, actualChecksum)
	}

	// Parse packet
	// Response format: [header][id][length][error][params...][checksum]
	pkt := Packet{
		ID:    id,
		Error: StatusError(data[4]),
	}

	paramLen := length - 2 // Subtract error byte and checksum
	if paramLen > 0 {
		pkt.Parameters = make([]byte, paramLen)
		copy(pkt.Parameters, data[5:5+paramLen])
	}

	return pkt, headerIdx + totalLen, nil
}

// DecodeMultiple parses multiple response packets from a buffer.
func (p *Protocol) DecodeMultiple(data []byte, count int) ([]Packet, error) {
	packets := make([]Packet, 0, count)
	offset := 0

	for i := 0; i < count && offset < len(data); i++ {
		pkt, consumed, err := p.Decode(data[offset:])
		if err != nil {
			// Try to find next header
			found := false
			for j := offset + 1; j <= len(data)-6; j++ {
				if data[j] == headerByte1 && data[j+1] == headerByte2 {
					offset = j
					found = true
					break
				}
			}
			if !found {
				break
			}
			continue
		}
		packets = append(packets, pkt)
		offset += consumed
	}

	return packets, nil
}

// ExpectedResponseLength returns the expected wire length for a response packet.
func (p *Protocol) ExpectedResponseLength(dataLen int) int {
	// header(2) + id(1) + length(1) + error(1) + data(n) + checksum(1)
	return 6 + dataLen
}

func (p *Protocol) calculateChecksum(data []byte) byte {
	var sum byte
	for _, b := range data {
		sum += b
	}
	return ^sum
}

// Instruction packet builders

// PingPacket creates a ping instruction packet.
func (p *Protocol) PingPacket(id byte) []byte {
	return p.Encode(Packet{
		ID:          id,
		Instruction: InstPing,
	})
}

// ReadPacket creates a read instruction packet.
func (p *Protocol) ReadPacket(id, address, length byte) []byte {
	return p.Encode(Packet{
		ID:          id,
		Instruction: InstRead,
		Parameters:  []byte{address, length},
	})
}

// WritePacket creates a write instruction packet.
func (p *Protocol) WritePacket(id, address byte, data []byte) []byte {
	params := make([]byte, 1+len(data))
	params[0] = address
	copy(params[1:], data)

	return p.Encode(Packet{
		ID:          id,
		Instruction: InstWrite,
		Parameters:  params,
	})
}

// RegWritePacket creates a reg write (buffered write) instruction packet.
func (p *Protocol) RegWritePacket(id, address byte, data []byte) []byte {
	params := make([]byte, 1+len(data))
	params[0] = address
	copy(params[1:], data)

	return p.Encode(Packet{
		ID:          id,
		Instruction: InstRegWrite,
		Parameters:  params,
	})
}

// ActionPacket creates an action instruction packet (triggers reg writes).
func (p *Protocol) ActionPacket() []byte {
	return p.Encode(Packet{
		ID:          BroadcastID,
		Instruction: InstAction,
	})
}

// SyncWritePacket creates a sync write instruction packet.
// data is a map of servo ID to the data bytes to write.
func (p *Protocol) SyncWritePacket(address byte, dataLen byte, servoData map[byte][]byte) []byte {
	// Parameters: address(1) + dataLen(1) + [id(1) + data(n)]...
	params := make([]byte, 0, 2+len(servoData)*(1+int(dataLen)))
	params = append(params, address, dataLen)

	for id, data := range servoData {
		params = append(params, id)
		params = append(params, data...)
	}

	return p.Encode(Packet{
		ID:          BroadcastID,
		Instruction: InstSyncWrite,
		Parameters:  params,
	})
}

// SyncReadPacket creates a sync read instruction packet.
func (p *Protocol) SyncReadPacket(address, dataLen byte, ids []byte) []byte {
	params := make([]byte, 0, 2+len(ids))
	params = append(params, address, dataLen)
	params = append(params, ids...)

	return p.Encode(Packet{
		ID:          BroadcastID,
		Instruction: InstSyncRead,
		Parameters:  params,
	})
}
