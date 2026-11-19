package autopilot

import (
	"fmt"
	"math"
	"reflect"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestLongStationArrivalRepairIsLocalAndIdempotent(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, long := range []bool{false, true} {
			t.Run(fmt.Sprintf("h%.2f/long%v", h, long), func(t *testing.T) {
				u := sim.UnitFromAngle(h)
				r := sim.Vec2{X: -u.Y, Y: u.X}
				stop := sim.NewBusStop(0, "target", u.Mul(7800), h, 8, sim.RGBA(0, 0, 0, 255), 52, 0)
				target := stop.FrontBerthPosition()
				straight, taper := 60., 170.
				if long {
					straight, taper = 160, 340
				}
				entry := target.Sub(u.Mul(straight))
				previous := entry.Sub(u.Mul(7000)).Sub(r.Mul(47.12))
				path := []sim.Vec2{previous.Sub(u.Mul(100)), previous, entry, target}
				old := append([]sim.Vec2(nil), path...)
				fixed, changed := repairLongStationArrival(path, stop, 52, long)
				want := entry.Sub(u.Mul(taper)).Sub(r.Mul(47.12))
				if !changed || len(fixed) != len(path)+1 || fixed[len(fixed)-3].Distance(want) > .001 {
					t.Fatal("missing local taper")
				}
				if !reflect.DeepEqual(path, old) || !reflect.DeepEqual(fixed[:2], old[:2]) || fixed[len(fixed)-1] != target {
					t.Fatal("changed original path, turn prefix or service target")
				}
				if _, again := repairLongStationArrival(fixed, stop, 52, long); again {
					t.Fatal("repaired an already correct path twice")
				}
				for _, mode := range []string{"nearby", "multi-lane", "not-front-berth", "backwards"} {
					candidate := append([]sim.Vec2(nil), old...)
					switch mode {
					case "nearby":
						candidate[len(candidate)-3] = entry.Sub(u.Mul(400)).Sub(r.Mul(47.12))
					case "multi-lane":
						candidate[len(candidate)-3] = previous.Sub(r.Mul(150))
					case "not-front-berth":
						candidate[len(candidate)-1] = target.Add(u.Mul(90))
					case "backwards":
						candidate[len(candidate)-3] = entry.Add(u.Mul(7000)).Sub(r.Mul(47.12))
					}
					if _, changed := repairLongStationArrival(candidate, stop, 52, long); changed {
						t.Fatalf("rewrote %s maneuver", mode)
					}
				}
			})
		}
	}
}
