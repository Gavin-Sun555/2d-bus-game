package game

import (
	"math"
	"os"
	"path/filepath"
	"runtime"
	"testing"
)

func TestSlot2RightApproachDoesNotOscillate(t *testing.T) {
	_, source, _, _ := runtime.Caller(0)
	p := filepath.Join(filepath.Dir(source), "../dist/slot2-sway-2026-11-04/slot2.json")
	if _, e := os.Stat(p); os.IsNotExist(e) {
		t.Skip("sway snapshot unavailable")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = p
	if e := g.LoadGame(); e != nil {
		t.Fatal(e)
	}
	g.Settings.IsOpen = false
	lastSign, reversals := 0, 0
	maxYaw := 0.
	initial := g.Bus.Pos
	for tick := 0; tick < 60*12; tick++ {
		if e := g.Update(); e != nil {
			t.Fatal(e)
		}
		yaw := simAngle(g.Bus.Heading + math.Pi/2)
		if math.Abs(yaw) > maxYaw {
			maxYaw = math.Abs(yaw)
		}
		sign := 0
		if yaw > .08 {
			sign = 1
		}
		if yaw < -.08 {
			sign = -1
		}
		if sign != 0 && sign != lastSign {
			if lastSign != 0 {
				reversals++
			}
			lastSign = sign
		}
		if tick%60 == 0 {
			t.Logf("tick%d x%.1f y%.1f yaw%.3f steer%.2f speed%.1f offset%.1f target%.1f", tick, g.Bus.Pos.X, g.Bus.Pos.Y, yaw, g.Bus.SteeringAngle, g.Bus.Speed, g.AutopilotDriver.LaneOffset, g.AutopilotDriver.TargetLaneOffset)
		}
	}
	t.Logf("reversals%d maxYaw%.2f traveled%.0f", reversals, maxYaw, g.Bus.Pos.Distance(initial))
	if g.Bus.Pos.Distance(initial) < 700 {
		t.Fatal("bus stayed stuck rather than stabilizing")
	}
	if reversals > 2 {
		t.Fatalf("repeated lane hunting: %d yaw reversals", reversals)
	}
}
func simAngle(a float64) float64 { return math.Atan2(math.Sin(a), math.Cos(a)) }
