package feetech

import (
	"bytes"
	"encoding/hex"
	"strings"
	"testing"
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
