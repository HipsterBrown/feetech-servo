package feetech

import "testing"

func TestSignMagnitude_RoundTrip(t *testing.T) {
	cases := []struct {
		signBit int
		value   int
	}{
		{15, 0}, {15, 100}, {15, -100}, {15, 32767}, {15, -32767},
		{11, 0}, {11, 1024}, {11, -1024},
		{9, 0}, {9, 256}, {9, -256},
		{0, 0}, {0, 42}, {0, -42}, // signBit=0 -> identity passthrough
	}
	for _, c := range cases {
		encoded := encodeSignMagnitude(c.value, c.signBit)
		decoded := decodeSignMagnitude(encoded, c.signBit)
		if decoded != c.value {
			t.Errorf("signBit=%d value=%d: encoded=%d decoded=%d",
				c.signBit, c.value, encoded, decoded)
		}
	}
}

func TestSignMagnitude_NoSignBitIsIdentity(t *testing.T) {
	// signBit=0 means raw passthrough.
	for _, v := range []int{0, 1, 100, -1, -100} {
		if got := encodeSignMagnitude(v, 0); got != v {
			t.Errorf("encodeSignMagnitude(%d, 0) = %d, want %d", v, got, v)
		}
		if got := decodeSignMagnitude(v, 0); got != v {
			t.Errorf("decodeSignMagnitude(%d, 0) = %d, want %d", v, got, v)
		}
	}
}

// TestPresentLoad_Decode covers real STS3215 readings: the direction bit is 10,
// so a 10-bit magnitude (0-1000) sits in bits 0-9.
func TestPresentLoad_Decode(t *testing.T) {
	if RegPresentLoad.SignBit != 10 {
		t.Fatalf("RegPresentLoad.SignBit = %d, want 10", RegPresentLoad.SignBit)
	}
	cases := []struct {
		raw  int
		want int
		desc string
	}{
		{0, 0, "free movement"},
		{200, 200, "overload protection backoff (protection_torque 20)"},
		{404, 404, "clamped jaw"},
		{1044, -20, "light holding load, reverse direction"},
	}
	for _, c := range cases {
		if got := decodeSignMagnitude(c.raw, RegPresentLoad.SignBit); got != c.want {
			t.Errorf("%s: decode(%d) = %d, want %d", c.desc, c.raw, got, c.want)
		}
	}
}
