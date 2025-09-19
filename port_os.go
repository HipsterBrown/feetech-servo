//go:build !baremetal

package feetech

import (
	"fmt"

	"go.bug.st/serial"
)

type osPort serial.Port

func OpenPort(port string, baud int) (osPort, error) {
	mode := &serial.Mode{
		BaudRate: baud,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	}

	p, err := serial.Open(port, mode)
	if err != nil {
		return nil, fmt.Errorf("failed to open port %s: %w", port, err)
	}

	return p, nil
}
