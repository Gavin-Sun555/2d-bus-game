package traffic

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

// A diagonal bus and the car beside its rear can block each other's next
// movement. That is not the protected, aligned queue behind a stopped bus.
func TestObliqueBusDoesNotExemptCarFromDeadlockRecovery(t *testing.T) {
	for _, player := range []bool{false, true} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, yaw := range []float64{0, .1, .536, -.536} {
				t.Run(fmt.Sprintf("player%v/h%.2f/yaw%.2f", player, h, yaw), func(t *testing.T) {
					u := sim.UnitFromAngle(h)
					r := sim.Vec2{X: -u.Y, Y: u.X}
					b := sim.NewBus(0, 0, h+yaw)
					c := &sim.TrafficCar{ID: 1, Pos: u.Mul(-58).Add(r.Mul(-16)), Heading: h, Length: 46, Width: 22}
					tm := &TrafficManager{Cars: []*sim.TrafficCar{c}}
					var p *sim.Bus
					if player {
						p = b
					} else {
						tm.AIBusMgr = &AIBusManager{Buses: []*AIBus{{Bus: b}}}
					}
					w := &sim.World{}
					for i := 0; i < 160; i++ {
						tm.ResolveDeadlocks(.5, p, len(tm.Cars), 52, w)
					}
					preserved := math.Abs(yaw) < .2
					if (len(tm.Cars) == 1) != preserved {
						t.Fatalf("remaining cars%d; aligned queue preservation%v", len(tm.Cars), preserved)
					}
				})
			}
		}
	}
}
