package feetech

import (
	"errors"
	"fmt"
	"strings"
	"testing"
)

func TestCommError_ErrorMessage(t *testing.T) {
	inner := errors.New("inner failure")
	e := &CommError{Op: "ping", Err: inner}
	got := e.Error()
	if !strings.Contains(got, "ping") {
		t.Errorf("Error message missing op: %q", got)
	}
	if !strings.Contains(got, "inner failure") {
		t.Errorf("Error message missing inner: %q", got)
	}
}

func TestCommError_Unwrap(t *testing.T) {
	inner := errors.New("inner")
	e := &CommError{Op: "x", Err: inner}
	if !errors.Is(e, inner) {
		t.Error("CommError should unwrap to inner")
	}
}

func TestServoError_ErrorMessageWithStatus(t *testing.T) {
	e := &ServoError{ID: 5, Op: "read", Status: ErrOverheat}
	got := e.Error()
	if !strings.Contains(got, "5") || !strings.Contains(got, "read") {
		t.Errorf("missing id/op: %q", got)
	}
	if !strings.Contains(got, "overheat") {
		t.Errorf("missing status: %q", got)
	}
}

func TestServoError_ErrorMessageWithErr(t *testing.T) {
	inner := errors.New("transport down")
	e := &ServoError{ID: 7, Op: "write", Err: inner}
	got := e.Error()
	if !strings.Contains(got, "7") || !strings.Contains(got, "write") || !strings.Contains(got, "transport down") {
		t.Errorf("missing fields: %q", got)
	}
}

func TestServoError_ErrorMessagePlain(t *testing.T) {
	// Neither Status nor Err set -- the plain branch.
	e := &ServoError{ID: 3, Op: "ping"}
	got := e.Error()
	if !strings.Contains(got, "3") || !strings.Contains(got, "ping") {
		t.Errorf("missing fields: %q", got)
	}
}

func TestServoError_Unwrap(t *testing.T) {
	inner := errors.New("inner")
	e := &ServoError{ID: 1, Op: "x", Err: inner}
	if !errors.Is(e, inner) {
		t.Error("ServoError should unwrap to inner")
	}
}

func TestIsTimeout(t *testing.T) {
	if !IsTimeout(ErrTimeout) {
		t.Error("IsTimeout(ErrTimeout) should be true")
	}
	if IsTimeout(errors.New("other")) {
		t.Error("IsTimeout(other) should be false")
	}
}

func TestIsNoResponse(t *testing.T) {
	if !IsNoResponse(ErrNoResponse) {
		t.Error("IsNoResponse(ErrNoResponse) should be true")
	}
	if IsNoResponse(errors.New("other")) {
		t.Error("IsNoResponse(other) should be false")
	}
}

func TestGetServoError(t *testing.T) {
	want := &ServoError{ID: 9, Op: "read"}
	got, ok := GetServoError(want)
	if !ok || got != want {
		t.Errorf("GetServoError direct: ok=%v got=%v", ok, got)
	}

	// Wrapped via fmt.Errorf("%w", ...) should still resolve.
	wrapped := errors.Join(errors.New("outer"), want)
	if got2, ok := GetServoError(wrapped); !ok || got2 != want {
		t.Errorf("GetServoError wrapped: ok=%v got=%v", ok, got2)
	}

	if _, ok := GetServoError(errors.New("not a servo error")); ok {
		t.Error("GetServoError on non-ServoError should be false")
	}
}

func TestStatusError_NoErrorString(t *testing.T) {
	if got := StatusError(0).Error(); got != "no error" {
		t.Errorf("StatusError(0).Error() = %q", got)
	}
}

func TestStatusError_AllFlags(t *testing.T) {
	all := ErrVoltage | ErrAngleLimit | ErrOverheat | ErrRange | ErrChecksum | ErrOverload | ErrInstruction
	got := all.Error()
	for _, want := range []string{"voltage", "angle limit", "overheat", "range", "checksum", "overload", "instruction"} {
		if !strings.Contains(got, want) {
			t.Errorf("StatusError(all) missing %q in %q", want, got)
		}
	}
}

func TestSplitStatus(t *testing.T) {
	tests := []struct {
		name      string
		status    StatusError
		wantValid bool
		wantErr   StatusError // 0 means no error expected
	}{
		{"no flags", 0, true, 0},
		{"overload is a condition", ErrOverload, true, ErrOverload},
		{"overheat is a condition", ErrOverheat, true, ErrOverheat},
		{"voltage is a condition", ErrVoltage, true, ErrVoltage},
		{"angle limit is a condition", ErrAngleLimit, true, ErrAngleLimit},
		{"multiple conditions", ErrOverload | ErrOverheat, true, ErrOverload | ErrOverheat},
		{"checksum invalidates", ErrChecksum, false, ErrChecksum},
		{"instruction invalidates", ErrInstruction, false, ErrInstruction},
		{"range invalidates", ErrRange, false, ErrRange},
		{"any request flag invalidates the whole response", ErrOverload | ErrChecksum, false, ErrOverload | ErrChecksum},
		{"undefined bit invalidates", StatusError(0x80), false, StatusError(0x80)},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			valid, err := splitStatus(tt.status)
			if valid != tt.wantValid {
				t.Errorf("payloadValid: got %v, want %v", valid, tt.wantValid)
			}
			if tt.wantErr == 0 {
				if err != nil {
					t.Errorf("err: got %v, want nil", err)
				}
			} else if !errors.Is(err, tt.wantErr) {
				t.Errorf("err: got %v, want %v", err, tt.wantErr)
			}
		})
	}
}

func TestConditionStatus(t *testing.T) {
	tests := []struct {
		name      string
		err       error
		wantFlags StatusError
		wantOK    bool
	}{
		{"nil error", nil, 0, false},
		{"bare condition flag", ErrOverload, ErrOverload, true},
		{"combined conditions", ErrOverload | ErrOverheat, ErrOverload | ErrOverheat, true},
		{"request flag is not a condition", ErrChecksum, 0, false},
		{"mixed flags are not a condition", ErrOverload | ErrChecksum, 0, false},
		{"undefined bit is not a condition", StatusError(0x80), 0, false},
		{"transport error", ErrTimeout, 0, false},
		{"wrapped in ServoError", &ServoError{ID: 6, Op: "ping", Status: ErrOverload}, ErrOverload, true},
		{"wrapped with fmt", fmt.Errorf("read: %w", ErrOverload), ErrOverload, true},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			flags, ok := ConditionStatus(tt.err)
			if ok != tt.wantOK {
				t.Errorf("ok: got %v, want %v", ok, tt.wantOK)
			}
			if flags != tt.wantFlags {
				t.Errorf("flags: got %v, want %v", flags, tt.wantFlags)
			}
		})
	}
}

// TestSyncReadError_EmptyStatus covers the degenerate shapes SyncRead itself
// never constructs (it guards on len(perServoFlags) > 0), so that a caller
// building one by hand, or a future regression in that guard, cannot produce a
// bogus "matched, no flags" result.
func TestSyncReadError_EmptyStatus(t *testing.T) {
	empty := &SyncReadError{Op: "sync_read"}

	// The As hook must decline rather than report a zero StatusError as a match:
	// a caller using errors.As directly would read a true return as success.
	var status StatusError
	if errors.As(error(empty), &status) {
		t.Errorf("errors.As matched an empty SyncReadError, got status %v", status)
	}
	if flags, ok := ConditionStatus(empty); ok {
		t.Errorf("ConditionStatus(empty) = (%v, true), want ok false", flags)
	}

	// And it must still render as something a human can read.
	if got := empty.Error(); got != "sync_read: no servo flags recorded" {
		t.Errorf("Error() = %q", got)
	}

	// A populated one still matches, so the guard didn't disable the hook.
	populated := &SyncReadError{Op: "sync_read", Status: map[int]StatusError{6: ErrOverload}}
	flags, ok := ConditionStatus(populated)
	if !ok || flags != ErrOverload {
		t.Errorf("ConditionStatus(populated) = (%v, %v), want (ErrOverload, true)", flags, ok)
	}
}
