package sim

import (
	"math"
	"testing"
)

func TestArticulatedBusLengthJointAndFollowingPhysics(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		b := NewBus(0, 0, heading)
		b.SetModel(BusArticulated)
		if b.OverallLength() != 192 {
			t.Fatal("not twice the standard bus")
		}
		for i := 0; i < 180; i++ {
			b.Update(1./60, 1, 0, .35, false, false)
			tail := b.TrailerBody()
			joint := tail.Center.Add(Vec2{X: math.Cos(tail.Heading), Y: math.Sin(tail.Heading)}.Mul(48))
			if joint.Distance(b.HitchPosition()) > 1e-8 {
				t.Fatal("joint detached")
			}
			if math.Abs(NormalizeAngle(b.Heading-b.TailHeading())) > MaxArticulationAngle+1e-8 {
				t.Fatal("joint exceeded stop")
			}
		}
		if math.Abs(NormalizeAngle(b.Heading-b.TailHeading())) < .1 {
			t.Fatal("trailer stayed rigid while turning")
		}
		for i := 0; i < 1800; i++ {
			b.Update(1./60, 1, 0, 0, false, false)
		}
		if math.Abs(NormalizeAngle(b.Heading-b.TailHeading())) > .01 {
			t.Fatal("trailer did not straighten")
		}
	}
}
func TestArticulatedTailCollisionAndQueue(t *testing.T) {
	b := NewBus(0, 0, 0)
	b.SetModel(BusArticulated)
	car := &TrafficCar{Pos: Vec2{X: -105}, Length: 46, Width: 22}
	if hit, _, _ := b.CollidesBody(OBB{Center: car.Pos, HalfLen: 23, HalfWid: 11}); !hit {
		t.Fatal("tail has no collision")
	}
	if !ResolveBusCarSeparation(b, car) {
		t.Fatal("tail collision unresolved")
	}
	w := NewWorld(52)
	st := w.Stops[0]
	b.Pos = st.Pos
	b.Heading = st.Heading
	b.ResetArticulation()
	follower := NewBus(0, 0, st.Heading)
	PlaceBusInStationQueue(follower, st, []*Bus{b}, 52)
	if hit, _, _ := CheckBusesCollision(b, follower); hit {
		t.Fatal("queue overlaps long bus")
	}
	gap := b.TrailerBody().Center.Distance(follower.Pos) - (b.Length+follower.Length)/2
	if gap < 23.9 {
		t.Fatal("tail queue gap too short", gap)
	}
}
func TestArticulationCadence(t *testing.T) {
	run := func(dt float64) *Bus {
		b := NewBus(0, 0, 0)
		b.SetModel(BusArticulated)
		for i := 0; i < int(3/dt); i++ {
			b.Update(dt, 1, 0, .25, false, false)
		}
		return b
	}
	a, c := run(1./60), run(1./120)
	if math.Abs(NormalizeAngle(a.TailHeading()-c.TailHeading())) > .04 {
		t.Fatal("trailer depends on frame cadence", a.TailHeading(), c.TailHeading())
	}
}

func TestLongReplacementWaitsForWholeBodyClearance(t *testing.T) {
	b := NewBus(0, 0, 0)
	b.DoorsOpen = true
	b.RequestModel(BusArticulated)
	r := &Route{StopIDs: []int{0, 1, 0}, CurrentIndex: 2}
	site := VehicleHandoverSite{Obstacles: []OBB{{Center: Vec2{X: -105}, HalfLen: 23, HalfWid: 11}, {Center: Vec2{X: 110}, HalfLen: 23, HalfWid: 11}}}
	if b.ReplaceModelAtTerminus(r, 0, site) || b.Model != BusStandard || b.Pos != (Vec2{}) || b.PendingModel == nil {
		t.Fatal("replacement appeared inside surrounding vehicles")
	}
	site.Obstacles = site.Obstacles[:1]
	if !b.ReplaceModelAtTerminus(r, 0, site) {
		t.Fatal("failed to replace after space opened")
	}
	if hit, _, _ := b.CollidesBody(site.Obstacles[0]); hit {
		t.Fatal("replacement tail overlaps following car")
	}
	if b.Pos.X <= 0 || b.Pos.X > 96 {
		t.Fatal("replacement outside allowed berth", b.Pos)
	}
}
func TestArticulatedReverseJointStopAndRecovery(t *testing.T) {
	b := NewBus(0, 0, 0)
	b.SetModel(BusArticulated)
	b.TrailerHeading = .15
	b.ShiftTo(GearReverse)
	stopped := false
	for i := 0; i < 3600; i++ {
		b.Update(1./60, 1, 0, .4, false, false)
		if math.Abs(NormalizeAngle(b.Heading-b.TailHeading())) > MaxArticulationAngle+1e-9 {
			t.Fatal("jackknife exceeded hinge stop")
		}
		if b.IsBraking && b.Speed == 0 {
			stopped = true
			break
		}
	}
	if !stopped {
		t.Fatal("reverse did not reach hinge stop")
	}
	b.ShiftTo(GearDrive)
	for i := 0; i < 2400; i++ {
		b.Update(1./60, 1, 0, 0, false, false)
	}
	if b.Speed <= 0 || math.Abs(NormalizeAngle(b.Heading-b.TailHeading())) > .02 {
		t.Fatal("could not straighten after reverse", b.Speed, b.Heading, b.TailHeading())
	}
}
