package feetech

import (
	"bytes"
	"encoding/hex"
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
