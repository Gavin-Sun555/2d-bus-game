package sim

import (
	"math"
	"testing"
)

func TestBusAccelerationResponseIsConsumedOnce(t *testing.T) {
	normal, fast := NewBus(0, 0, 0), NewBus(0, 0, 0)
	normal.ShiftTo(GearDrive)
	fast.ShiftTo(GearDrive)
	fast.SetNextAccelerationScale(1.2)
	normal.Update(.1, 1, 0, 0, false, false)
	fast.Update(.1, 1, 0, 0, false, false)
	if math.Abs(fast.Speed-normal.Speed*1.2) > 1e-8 {
		t.Fatalf("response did not increase acceleration: normal%.2f fast%.2f", normal.Speed, fast.Speed)
	}
	beforeNormal, beforeFast := normal.Speed, fast.Speed
	normal.Update(.1, 1, 0, 0, false, false)
	fast.Update(.1, 1, 0, 0, false, false)
	if math.Abs((normal.Speed-beforeNormal)-(fast.Speed-beforeFast)) > 1e-8 {
		t.Fatal("boost leaked into subsequent manual step")
	}
	fast.DoorsOpen = true
	fast.SetNextAccelerationScale(1.2)
	fast.Update(.1, 1, 0, 0, false, false)
	fast.DoorsOpen = false
	fast.DoorProgress = 0
	fast.Update(.1, 1, 0, 0, false, false)
	if math.Abs(fast.Speed-beforeNormal) > 1e-8 {
		t.Fatal("door interlock retained acceleration boost")
	}
}
