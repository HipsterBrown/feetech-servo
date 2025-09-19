//go:build baremetal

package feetech

import (
	"errors"
	"machine"
	"time"
)

type mcuPort struct {
	*machine.UART
}

var currentPort mcuPort

func OpenPort(port string, baud int) (*mcuPort, error) {
	switch port {
	case "0":
		currentPort = mcuPort{machine.UART0}
		return &currentPort, nil
	case "1":
		currentPort = mcuPort{machine.UART1}
	default:
		return nil, errors.New("unknown UART")
	}

	currentPort.SetBaudRate(uint32(baud))
	return &currentPort, nil
}

func (port *mcuPort) SetReadTimeout(t time.Duration) error {
	return nil
}

func (port *mcuPort) Close() error {
	return nil
}
