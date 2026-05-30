package feetech

import (
	"context"
	"testing"
)

// TestTypedConstants is a compile-time contract: it fails to build if the
// protocol-version / operating-mode constants lose their named types or the
// Register.Size field stops being a byte.
func TestTypedConstants(t *testing.T) {
	// Protocol-version constants are of type ProtocolVersion.
	var pv ProtocolVersion = ProtocolSTS
	_ = pv
	if ProtocolSTS == ProtocolSCS {
		t.Fatal("ProtocolSTS and ProtocolSCS must differ")
	}

	// Operating-mode constants are of type OperatingMode.
	var mode OperatingMode = ModePosition
	_ = mode

	// Config/model fields and method signatures use the named types.
	var _ ProtocolVersion = BusConfig{}.Protocol
	var _ ProtocolVersion = Model{}.Protocol
	var _ func(context.Context) (OperatingMode, error) = (&Servo{}).OperatingMode
	var _ func(context.Context, OperatingMode) error = (&Servo{}).SetOperatingMode
	var _ func(ProtocolVersion) *Protocol = NewProtocol

	// Register.Size is a byte (uniform with Address).
	var _ byte = RegGoalPosition.Size
}
