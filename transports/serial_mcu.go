//go:build baremetal

package transports

import (
	"errors"
	"machine"
	"time"
)

type MCUTransport struct {
	*machine.UART
}

type SerialConfig struct {
	Device   *machine.UART
	Port     string // Ignored on MCU, but required for interface compatibility
	BaudRate int
	Timeout  time.Duration
}

var currentTransport MCUTransport

// OpenSerial gets a UART port with the given configuration.
func OpenSerial(cfg SerialConfig) (*MCUTransport, error) {
	if cfg.Device == nil {
		return nil, errors.New("UART device is required")
	}

	if cfg.BaudRate == 0 {
		cfg.BaudRate = 1000000
	}

	if cfg.Timeout == 0 {
		cfg.Timeout = time.Second
	}

	currentTransport = MCUTransport{cfg.Device}
	currentTransport.SetBaudRate(uint32(cfg.BaudRate))

	return &currentTransport, nil
}

func (t *MCUTransport) SetReadTimeout(time.Duration) error {
	return nil
}

func (t *MCUTransport) Close() error {
	return nil
}

func (t *MCUTransport) Flush() error {
	return nil
}
