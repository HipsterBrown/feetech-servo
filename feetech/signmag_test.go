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
