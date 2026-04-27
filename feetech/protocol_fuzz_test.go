package feetech

import "testing"

// FuzzProtocolDecode validates that Protocol.Decode never panics on arbitrary input.
// Seed corpus is drawn from manual fixtures and known-good responses.
func FuzzProtocolDecode(f *testing.F) {
	seeds := []string{
		"FF FF 01 02 01 FB",        // ping
		"FF FF 01 02 00 FC",        // ping reply
		"FF FF 01 04 02 38 02 BE",  // read
		"FF FF 01 04 00 18 05 DD",  // read reply
		"FF FF FE 04 03 05 01 F4",  // write ID
		"FF FF FE 02 05 FA",        // action
		"FF FF 01 02 06 F6",        // reset
		"00 00 00",                 // too short
		"00 12 FF FF 01 02 00 FC",  // garbage prefix
		"FF FF 01 02 00 00",        // bad checksum
	}
	for _, s := range seeds {
		f.Add(mustHexF(f, s))
	}

	p := NewProtocol(ProtocolSTS)
	f.Fuzz(func(t *testing.T, data []byte) {
		// Must not panic.
		_, _, _ = p.Decode(data)
	})
}
