package autopilot

import (
	_ "embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/harbor_station_arrival.json
var harborArrivalFixture []byte

func harborArrivalCase(t *testing.T) (*sim.Bus, *sim.Bus, *AutopilotDriver, *sim.World) {
	t.Helper()
	var c struct {
		Bus, Follower sim.Bus
		Driver        DriverSnapshot
		World         sim.World
	}
	if err := json.Unmarshal(harborArrivalFixture, &c); err != nil {
		t.Fatal(err)
	}
	ad, err := c.Driver.Restore(&c.World)
	if err != nil {
		t.Fatal(err)
	}
	return &c.Bus, &c.Follower, ad, &c.World
}

// Captured user save: the lead was 31px short of its front marker, while the
// follower's nose overlapped the rear platform without touching the lead.
func TestHarborStationArrivalWithArticulatedFollower(t *testing.T) {
	bus, follower, ad, w := harborArrivalCase(t)
	r := &sim.Route{StopIDs: []int{3, 5, 6}, CurrentIndex: 1}
	obs := []VehicleObstacle{ObserveBus(follower)}
	start := bus.Pos
	for tick := 0; tick < 1800; tick++ {
		th, br, st, hb, doors, docked := ad.ComputeControls(bus, r, w, nil, obs, 52, 1./60)
		bus.Update(1./60, th, br, st, hb, doors)
		for _, part := range follower.BodyParts() {
			if part.HalfLen > 0 {
				if hit, _, _ := bus.CollidesBody(part); hit {
					t.Fatalf("follower contact at tick %d", tick)
				}
			}
		}
		if boundary := bus.CheckBoundaries(w, 52); boundary.Zone == sim.ZoneHardBarrier {
			t.Fatalf("boundary contact at tick %d: %+v", tick, boundary)
		}
		if bus.DoorsOpen && docked {
			if !w.Stops[5].ContainsBus(bus) {
				t.Fatal("boarded outside platform")
			}
			return
		}
	}
	t.Fatalf("did not board: moved %.2f", bus.Pos.Distance(start))
}

func TestStationBerthReservationPreservesVehicleOrder(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		leader, follower, _, w := harborArrivalCase(t)
		stop := *w.Stops[5]
		origin := stop.Pos
		angle := heading - stop.Heading
		rotate := func(p sim.Vec2) sim.Vec2 {
			d := p.Sub(origin)
			u := sim.UnitFromAngle(angle)
			return sim.Vec2{X: d.X*u.X - d.Y*u.Y, Y: d.X*u.Y + d.Y*u.X}
		}
		for _, b := range []*sim.Bus{leader, follower} {
			b.Pos = rotate(b.Pos)
			b.Heading += angle
			b.TrailerHeading += angle
		}
		stop.Pos = sim.Vec2{}
		stop.Heading = heading
		if got := availableStationBerths(leader, &stop, []VehicleObstacle{ObserveBus(follower)}); len(got) != 1 {
			t.Fatalf("heading %.2f: clear lead berth rejected", heading)
		}
		if got := availableStationBerths(follower, &stop, []VehicleObstacle{ObserveBus(leader)}); len(got) != 0 {
			t.Fatalf("heading %.2f: follower reserved occupied platform", heading)
		}
		// Move the follower alongside the lead: a genuine occupied platform must
		// continue to prevent a second long bus from choosing this berth.
		follower.Pos = leader.Pos
		follower.Heading = leader.Heading
		follower.TrailerHeading = leader.TrailerHeading
		if got := availableStationBerths(leader, &stop, []VehicleObstacle{ObserveBus(follower)}); len(got) != 0 {
			t.Fatalf("heading %.2f: overlapping bus ignored", heading)
		}
	}
}
