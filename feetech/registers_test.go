package feetech

import (
	"slices"
	"testing"
)

func TestGetModel_KnownAndUnknown(t *testing.T) {
	if m, ok := GetModel("sts3215"); !ok || m == nil || m.Number != 777 {
		t.Errorf("GetModel(sts3215) failed: ok=%v m=%v", ok, m)
	}
	if _, ok := GetModel("nonexistent"); ok {
		t.Error("GetModel(nonexistent) should be false")
	}
}

func TestGetModelByNumber_KnownAndUnknown(t *testing.T) {
	if m, ok := GetModelByNumber(777); !ok || m == nil || m.Name != "sts3215" {
		t.Errorf("GetModelByNumber(777) failed: ok=%v m=%v", ok, m)
	}
	if _, ok := GetModelByNumber(99999); ok {
		t.Error("GetModelByNumber(99999) should be false")
	}
}

func TestListModels_IncludesBuiltins(t *testing.T) {
	names := ListModels()
	for _, want := range []string{"sts3215", "sts3250", "scs0009", "scs15"} {
		if !slices.Contains(names, want) {
			t.Errorf("ListModels missing %q in %v", want, names)
		}
	}
}

func TestModel_GetRegister_ModelSpecific(t *testing.T) {
	m, _ := GetModel("scs0009")
	reg, ok := m.GetRegister("running_time")
	if !ok || reg.Address != 44 {
		t.Errorf("scs0009 running_time: ok=%v reg=%v", ok, reg)
	}
}

func TestModel_GetRegister_FallsBackToCommon(t *testing.T) {
	m, _ := GetModel("sts3215") // STS3215 has nil Registers; falls through to common.
	reg, ok := m.GetRegister("torque_enable")
	if !ok || reg.Address != RegTorqueEnable.Address {
		t.Errorf("sts3215 torque_enable: ok=%v reg=%v", ok, reg)
	}
}

func TestModel_GetRegister_Unknown(t *testing.T) {
	m, _ := GetModel("sts3215")
	if _, ok := m.GetRegister("not_a_real_register"); ok {
		t.Error("unknown register should return false")
	}
}

func TestModel_BaudRateIndex(t *testing.T) {
	m, _ := GetModel("sts3215")
	if got := m.BaudRateIndex(1000000); got != 0 {
		t.Errorf("1000000 baud index: got %d want 0", got)
	}
	if got := m.BaudRateIndex(115200); got != 4 {
		t.Errorf("115200 baud index: got %d want 4", got)
	}
	if got := m.BaudRateIndex(123); got != -1 {
		t.Errorf("unknown baud should return -1: got %d", got)
	}
}

func TestGetCommonRegister(t *testing.T) {
	reg, ok := getCommonRegister("goal_position")
	if !ok || reg.Address != RegGoalPosition.Address {
		t.Errorf("goal_position lookup: ok=%v reg=%v", ok, reg)
	}
	if _, ok := getCommonRegister("not_real"); ok {
		t.Error("unknown common register should return false")
	}
}
