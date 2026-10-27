package autopilot

import (
	"reflect"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestLegacyLeftPathRefreshOnlyBeforeChangedTurn(t *testing.T) {
	const lw = 52.
	for _, entered := range []bool{false, true} {
		w, in, out, _ := wideLeftExitWorld(lw, 0, true)
		ad := NewAutopilotDriver()
		ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
		current := append([]sim.Vec2(nil), ad.drivePath...)
		g := w.JunctionAt(sim.Vec2{}, lw)
		// The old curb-left arc exited in the median before its outside-lane taper.
		old := append([]sim.Vec2(nil), current[:3]...)
		arc := sim.QuarterTurnPath(sim.Vec2{}, in, out, 3.5*lw, .5*lw, g.TurnRadius)
		old = append(old, arc...)
		old = append(old, sim.Vec2{X: .5 * lw, Y: -g.HalfY - 80}, sim.Vec2{X: 3.5 * lw, Y: -g.HalfY - 500}, w.Stops[1].Pos)
		ad.drivePath = old
		ad.pathRevision = 0
		ad.cumLen = make([]float64, len(old))
		for i := 1; i < len(old); i++ {
			ad.cumLen[i] = ad.cumLen[i-1] + old[i].Distance(old[i-1])
		}
		pos := sim.Vec2{X: -1000, Y: 3.5 * lw}
		if entered {
			pos = arc[12]
		}
		ad.pathSynced = false
		ad.trackPath(pos)
		restored, err := ad.Snapshot().Restore(w)
		if err != nil {
			t.Fatal(err)
		}
		want := current
		if entered {
			want = old
		}
		if !reflect.DeepEqual(restored.drivePath, want) {
			t.Fatalf("entered%v: wrong saved path migration", entered)
		}
		if !entered && restored.pathRevision != currentDrivePathRevision {
			t.Fatal("legacy approach not updated")
		}
	}
}
