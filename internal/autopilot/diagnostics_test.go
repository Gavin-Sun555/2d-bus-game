package autopilot

import (
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"reflect"
	"testing"
)

func TestStopDiagnosisIdentifiesSignalAndPhysicalBlocker(t *testing.T) {
	const lw = 52.
	w := sim.NewWorld(lw, sim.MapThemeMetropolis)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	var it *sim.TrafficIntersection
	for _, p := range tls.Intersections {
		if p.Center.Distance(sim.Vec2{X: 7800, Y: -1800}) < 1 {
			it = p
			break
		}
	}
	b := sim.NewBus(7800-3.5*lw, it.Center.Y-it.GetStopLineDistForHeading(math.Pi/2, lw)-62, math.Pi/2)
	r := &sim.Route{StopIDs: []int{3, 11}, CurrentIndex: 1}
	ad := NewAutopilotDriver()
	it.Phase = sim.PhaseEWGreen
	ad.ComputeControls(b, r, w, tls, nil, lw, 1./60)
	before := *ad
	d := ad.DiagnoseStop(b, r, w, tls, nil, lw)
	if d.Limits["signal"] != 0 {
		t.Fatal("missed red signal")
	}
	if !reflect.DeepEqual(before, *ad) {
		t.Fatal("diagnosis mutated live controller")
	}
	it.Phase = sim.PhaseNSGreen
	ob := VehicleObstacle{Pos: b.Pos.Add(sim.Vec2{Y: 90}), Heading: b.Heading, Length: 96, Width: 36}
	d = ad.DiagnoseStop(b, r, w, tls, []VehicleObstacle{ob}, lw)
	if d.Limits["signal"] <= 0 || d.Limits["obstacles"] != 0 || len(d.Blockers) != 1 {
		t.Fatalf("did not distinguish green obstacle stop: %+v", d)
	}
	if _, err := json.Marshal(d); err != nil {
		t.Fatal(err)
	}
}
