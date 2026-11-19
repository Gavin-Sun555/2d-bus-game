package autopilot

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/slot2-evening-blocks-20261116.json.gz
var eveningBlocks []byte

type eveningFleet struct {
	Vehicle struct {
		ID    int
		Route *sim.Route
	}
	State struct {
		Bus    *sim.Bus
		Driver DriverSnapshot
	}
}
type eveningBlocksScene struct {
	World  *sim.World
	Fleet  []eveningFleet
	Cars   []*sim.TrafficCar
	Lights *sim.TrafficLightSystem
	Player struct {
		Bus    *sim.Bus
		Driver DriverSnapshot
	}
}

func loadEveningBlocks(t *testing.T) eveningBlocksScene {
	t.Helper()
	gz, e := gzip.NewReader(bytes.NewReader(eveningBlocks))
	if e != nil {
		t.Fatal(e)
	}
	defer gz.Close()
	var s eveningBlocksScene
	if e := json.NewDecoder(gz).Decode(&s); e != nil {
		t.Fatal(e)
	}
	return s
}
func TestEveningOverrunBusBoardsWithoutReset(t *testing.T) {
	s := loadEveningBlocks(t)
	var a eveningFleet
	for _, v := range s.Fleet {
		if v.Vehicle.ID == 126 {
			a = v
		}
	}
	b := a.State.Bus
	ad, e := a.State.Driver.Restore(s.World)
	if e != nil {
		t.Fatal(e)
	}
	ad.FleetBus = true
	var obs []VehicleObstacle
	for _, o := range s.Fleet {
		if o.Vehicle.ID != 126 && o.State.Bus.Pos.Distance(b.Pos) < 1000 {
			obs = append(obs, ObserveBus(o.State.Bus))
		}
	}
	for _, c := range s.Cars {
		if c.Pos.Distance(b.Pos) < 1000 {
			obs = append(obs, VehicleObstacle{Car: c, Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width})
		}
	}
	ad.BeforeRecovery = func() { t.Fatal("bus relocated") }
	for tick := 0; tick < 1800; tick++ {
		p := b.Pos
		th, br, st, hb, doors, dk := ad.ComputeControls(b, a.Vehicle.Route, s.World, s.Lights, obs, 52, 1./60)
		b.Update(1./60, th, br, st, hb, doors)
		if b.Pos.Distance(p) > math.Abs(b.Speed)/60+.05 {
			t.Fatal("bus jumped")
		}
		if b.CheckBoundaries(s.World, 52).Zone == sim.ZoneHardBarrier {
			t.Fatal("hit boundary")
		}
		for _, o := range obs {
			if sweepBodiesCollision(b.BodyParts(), o, 0, 0, 0) {
				t.Fatal("hit another vehicle")
			}
		}
		if dk && b.DoorsOpen {
			t.Logf("boarded %.2fs pos%v yaw%.3f", float64(tick)/60, b.Pos, b.Heading)
			return
		}
	}
	t.Fatalf("bus126 still held pos%v speed%.1f recovery%+v", b.Pos, b.Speed, ad.headonRecovery)
}

func TestEveningBentRearBusDoesNotReserveTurnMerge(t *testing.T) {
	s := loadEveningBlocks(t)
	var rear, lead eveningFleet
	for _, v := range s.Fleet {
		if v.Vehicle.ID == 122 {
			rear = v
		}
		if v.Vehicle.ID == 124 {
			lead = v
		}
	}
	front, back := lead.State.Bus, rear.State.Bus
	front.LaneMergeRequest = lead.State.Driver.ApproachMerge
	req := front.LaneMergeRequest
	r := sim.Vec2{X: -math.Sin(req.Heading), Y: math.Cos(req.Heading)}
	f := autopilotFrame{bus: front, world: s.World, tls: s.Lights, laneWidth: 52}
	ob := ObserveBus(back)
	f.obstacles = []VehicleObstacle{ob}
	actual := front.Pos.Sub(req.Target).Dot(r)
	if !requiredMergeLaneClear(f, req.Heading, actual, 0) {
		t.Fatal("stationary bent follower reserved a physically clear merge")
	}
	for _, name := range []string{"moving rear", "body overlap", "completed merge", "no intent", "opposing rear", "wide merge", "no world"} {
		one := f
		copyFront, copyRear := *front, *back
		copyReq := *req
		copyFront.LaneMergeRequest = &copyReq
		one.bus = &copyFront
		other := ObserveBus(&copyRear)
		cur := actual
		switch name {
		case "moving rear":
			other.Speed = 10
		case "body overlap":
			copyRear.Pos = copyFront.Pos
			copyRear.ResetArticulation()
			other = ObserveBus(&copyRear)
		case "completed merge":
			copyReq.Joined = true
		case "no intent":
			copyFront.LaneMergeRequest = nil
		case "opposing rear":
			copyRear.Heading = req.Heading + math.Pi
			copyRear.ResetArticulation()
			other = ObserveBus(&copyRear)
		case "wide merge":
			cur = 100
		case "no world":
			one.world = nil
		}
		if stoppedJunctionFollowerClearsMerge(one, other, req.Heading, cur, 0) {
			t.Fatalf("released unsafe %s", name)
		}
	}
}

func TestEveningPassedStationBusDoesNotBrakeLeader(t *testing.T) {
	s := loadEveningBlocks(t)
	var front, rear eveningFleet
	for _, v := range s.Fleet {
		if v.Vehicle.ID == 126 {
			front = v
		}
		if v.Vehicle.ID == 127 {
			rear = v
		}
	}
	ad, e := front.State.Driver.Restore(s.World)
	if e != nil {
		t.Fatal(e)
	}
	f := autopilotFrame{bus: front.State.Bus, world: s.World, tls: s.Lights, targetStop: s.World.Stops[23], laneWidth: 52, s: ad.LastPathS, remaining: ad.LastRemaining, distToStop: front.State.Bus.Pos.Distance(s.World.Stops[23].Pos), obstacles: []VehicleObstacle{ObserveBus(rear.State.Bus)}}
	if speed := ad.stationSpeed(f, 150); speed < 14 {
		t.Fatalf("stopped for the bus behind: %.2f", speed)
	}
}

func TestEveningStationOverrunKeepsSafetyGuards(t *testing.T) {
	for _, name := range []string{"doors", "service", "moving", "unfinished path", "blocked reverse", "no world", "not a boarding target", "opposing heading", "no roadside permission"} {
		t.Run(name, func(t *testing.T) {
			s := loadEveningBlocks(t)
			var a eveningFleet
			for _, v := range s.Fleet {
				if v.Vehicle.ID == 126 {
					a = v
				}
			}
			ad, e := a.State.Driver.Restore(s.World)
			if e != nil {
				t.Fatal(e)
			}
			ad.FleetBus = true
			ad.stationOverrunWait = 2
			b := a.State.Bus
			stop := s.World.Stops[23]
			f := autopilotFrame{bus: b, world: s.World, tls: s.Lights, targetStop: stop, laneWidth: 52, s: ad.LastPathS, remaining: 178, distToStop: b.Pos.Distance(stop.Pos)}
			switch name {
			case "doors":
				b.DoorsOpen = true
			case "service":
				b.AddPassengerService(1, 1)
			case "moving":
				b.Speed = 30
				f.speedAbs = 30
			case "unfinished path":
				f.s -= 100
			case "blocked reverse":
				f.obstacles = []VehicleObstacle{{Pos: b.TrailerBody().Center, Heading: b.TailHeading(), Length: 30, Width: 30}}
			case "no world":
				f.world = nil
			case "not a boarding target":
				ad.drivePath = append([]sim.Vec2(nil), ad.drivePath...)
				ad.drivePath[len(ad.drivePath)-1].X += 100
			case "opposing heading":
				b.Heading = stop.Heading + math.Pi
			case "no roadside permission":
				ad.FleetBus = false
				ad.outerDockAllowed = false
			}
			ad.tryStationOverrunRetreat(f, 1./60)
			if ad.headonRecovery != nil {
				t.Fatal("started an unsafe retreat")
			}
		})
	}
}
