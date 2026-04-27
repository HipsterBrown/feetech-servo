package feetech

import (
	"bytes"
	"encoding/hex"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/hipsterbrown/feetech-servo/transports"
)

// mustHex parses a whitespace-tolerant hex string into bytes. It calls t.Fatal on parse failure.
// Used to keep manual-fixture tests readable: mustHex(t, "FF FF 01 02 01 FB").
func mustHex(t *testing.T, s string) []byte {
	t.Helper()
	cleaned := strings.Map(func(r rune) rune {
		switch r {
		case ' ', '\t', '\n', '\r':
			return -1
		}
		return r
	}, s)
	out, err := hex.DecodeString(cleaned)
	if err != nil {
		t.Fatalf("mustHex(%q): %v", s, err)
	}
	return out
}

// expectWrite asserts that the mock's recorded WriteData equals the given hex fixture exactly.
// On mismatch it calls t.Errorf with a side-by-side hex diff.
func expectWrite(t *testing.T, mock *transports.MockTransport, hexFixture string) {
	t.Helper()
	want := mustHex(t, hexFixture)
	if !bytes.Equal(mock.WriteData, want) {
		t.Errorf("WriteData mismatch:\n  got:  %X\n  want: %X", mock.WriteData, want)
	}
}

func TestMustHex(t *testing.T) {
	t.Run("parses uppercase with spaces", func(t *testing.T) {
		got := mustHex(t, "FF FF 01 02 01 FB")
		want := []byte{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}
		if !bytes.Equal(got, want) {
			t.Fatalf("got %X, want %X", got, want)
		}
	})
	t.Run("parses lowercase without spaces", func(t *testing.T) {
		got := mustHex(t, "ffff0102")
		want := []byte{0xFF, 0xFF, 0x01, 0x02}
		if !bytes.Equal(got, want) {
			t.Fatalf("got %X, want %X", got, want)
		}
	})
	t.Run("ignores newlines and tabs", func(t *testing.T) {
		got := mustHex(t, "FF FF\n01\t02 01 FB")
		want := []byte{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}
		if !bytes.Equal(got, want) {
			t.Fatalf("got %X, want %X", got, want)
		}
	})
}

func TestExpectWrite(t *testing.T) {
	t.Run("matches exact write", func(t *testing.T) {
		mock := &transports.MockTransport{}
		if _, err := mock.Write([]byte{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}); err != nil {
			t.Fatal(err)
		}
		// Should pass without invoking t.Fatal:
		expectWrite(t, mock, "FF FF 01 02 01 FB")
	})

	t.Run("reports a readable diff on mismatch", func(t *testing.T) {
		mock := &transports.MockTransport{}
		if _, err := mock.Write([]byte{0xFF, 0xFF, 0x99, 0x02, 0x01, 0xFB}); err != nil {
			t.Fatal(err)
		}
		// Use a sub-tester so we can assert the failure occurred without aborting the outer test:
		fake := &testing.T{}
		expectWrite(fake, mock, "FF FF 01 02 01 FB")
		if !fake.Failed() {
			t.Fatal("expected expectWrite to fail on mismatch")
		}
	})
}

// loadHexFixture reads testdata/<name>.hex and parses it via mustHex.
func loadHexFixture(t *testing.T, name string) []byte {
	t.Helper()
	path := filepath.Join("testdata", name+".hex")
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("loadHexFixture(%q): %v", name, err)
	}
	return mustHex(t, string(data))
}

func TestLoadHexFixture_AllManualFixtures(t *testing.T) {
	fixtures := []struct {
		name   string
		minLen int
	}{
		{"manual_ping", 6},
		{"manual_ping_response", 6},
		{"manual_read_position", 8},
		{"manual_read_position_response", 8},
		{"manual_write_id", 8},
		{"manual_write_pos_time_speed", 13},
		{"manual_action", 6},
		{"manual_sync_write_4_servos", 36},
		{"manual_reset", 6},
		{"sync_read_two_responses", 16},
	}
	for _, f := range fixtures {
		t.Run(f.name, func(t *testing.T) {
			got := loadHexFixture(t, f.name)
			if len(got) != f.minLen {
				t.Fatalf("fixture %s: got %d bytes, want %d", f.name, len(got), f.minLen)
			}
		})
	}
}
