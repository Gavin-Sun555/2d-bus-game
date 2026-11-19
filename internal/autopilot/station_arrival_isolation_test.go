package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestDistantArrivalRunInDoesNotChangeDeparture(t *testing.T) {
	var reference sim.Bus
	for _, runIn := range []float64{160, 280} {
		const lw = 52.0
		w := &sim.World{
			Roads: []sim.RoadSegment{
				{P1: sim.Vec2{X: -3900, Y: -7200}, P2: sim.Vec2{X: -3900, Y: 7200}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane, HasLines: true},
			},
		}
		w.RebuildIntersections()
		stopFrom := sim.NewBusStop(0, "BayFrom", sim.Vec2{X: -3900, Y: -6300}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
		stopFrom.Length = 336
		stopTo := sim.NewBusStop(1, "BayTo", sim.Vec2{X: -3900, Y: 4500}, math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
		stopTo.Length = 336
		w.Stops = []*sim.BusStop{stopFrom, stopTo}

		// Leader articulated bus has pulled into the adjacent lane at a sharp east-northeast yaw,
		// while a trailing articulated bus sits stationary at the departure mouth behind it.
		lead := sim.NewBus(-4029.1, -6109.2, 0.527)
		lead.SetModel(sim.BusArticulated)
		lead.TrailerHeading = 1.353
		lead.TrailerInitialized = true
		lead.SteeringAngle = 0.54

		rear := sim.NewBus(-4129.0, -6175.7, 1.569)
		rear.SetModel(sim.BusArticulated)
		rear.TrailerHeading = 1.571
		rear.TrailerInitialized = true

		ad := NewAutopilotDriver()
		ad.FleetBus = true
		ad.State = AutopilotStateCruising
		ad.pathFromID = 0
		ad.pathToID = 1
		ad.drivePath = []sim.Vec2{
			{X: -4129.12, Y: -6300},
			{X: -4082, Y: -5950},
			{X: -4082, Y: 4000},
		}
		ad.cumLen = make([]float64, len(ad.drivePath))
		for i := 1; i < len(ad.drivePath); i++ {
			ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
		}
		route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
		obs := []VehicleObstacle{ObserveBus(rear)}

		ad.articulatedPath = true
		ad.pathWorld = w
		ad.pathLane = lw
		ad.pathRevision = currentDrivePathRevision
		ad.drivePath = buildDrivePathWithRunIn(w, stopFrom, stopTo, lw, false, false, runIn, sim.RoadPathOptions{LongVehicle: true})
		ad.cumLen = make([]float64, len(ad.drivePath))
		for i := 1; i < len(ad.drivePath); i++ {
			ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
		}
		startY := lead.Pos.Y
		for tick := 0; tick < 600; tick++ {
			th, br, st, hb, dr, _ := ad.ComputeControls(lead, route, w, nil, obs, lw, 1./60)
			lead.Update(1./60, th, br, st, hb, dr)
			if r := lead.CheckBoundaries(w, lw); r.Zone == sim.ZoneHardBarrier {
				t.Fatalf("struck hard barrier at tick %d: %s", tick, r.ObstacleName)
			}
			for _, pa := range lead.BodyParts() {
				for _, pb := range rear.BodyParts() {
					if hit, _, depth := sim.CheckOBBCollision(pa, pb); hit && depth > 1e-6 {
						t.Fatalf("collided with trailing bus at tick %d: depth=%g", tick, depth)
					}
				}
			}
		}
		if runIn == 160 {
			reference = *lead
		} else if lead.Pos != reference.Pos || lead.Heading != reference.Heading || lead.TailHeading() != reference.TailHeading() || lead.Speed != reference.Speed {
			t.Fatal("a distant station taper changed departure control")
		}
		if lead.Pos.Y-startY < 800 || lead.Speed < 50 {
			t.Fatal("yawed departure failed to clear")
		}
	}
}
