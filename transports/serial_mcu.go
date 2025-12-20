//go:build baremetal

package transports

import (
  "errors"
	"fmt"
  "machine"
  "time"
)

type MCUTransport {
  *machine.UART
}

type SerialConfig struct {
	Port     string
	BaudRate int
	Timeout  time.Duration
}

var currentTransport MCUTransport

// OpenSerial gets a UART port with the given configuration.
func OpenSerial(cfg SerialConfig) (*MCUTransport, error) {
	if cfg.Port == "" {
		return nil, errors.New("serial port path is required")
	}

	if cfg.BaudRate == 0 {
		cfg.BaudRate = 1000000
	}

	if cfg.Timeout == 0 {
		cfg.Timeout = time.Second
	}

  switch cfg.Port {
  case "0":
    currentTransport = MCUTransport {machine.UART0}
  case "1":
    currentTransport = MCUTransport{machine.UART1}
  default:
		return nil, fmt.Errorf("unknown UART %s", cfg.Port)
  }

  currentTransport.SetBaudRate(uint32(cfg.BaudRate))

  return &currentTransport, nil
}

func (t *MCUTransport) SetReadTimeout(t time.Duration) error {
  return nil
}

func (t *MCUTransport) Close() error {
  return nil
}

func (t *MCUTransport) Flush() error {
  return nil
}
