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

//go:embed testdata/slot2-night-rear-departure-20261116.json.gz
var nightRearDeparture []byte

func TestNightRearBerthDepartureClearsShelter(t *testing.T) {

	z, e := gzip.NewReader(bytes.NewReader(nightRearDeparture))
	if e != nil {
		t.Fatal(e)
	}
	defer z.Close()
	var s eveningBlocksScene
	if e = json.NewDecoder(z).Decode(&s); e != nil {
		t.Fatal(e)
	}
	w, b := s.World, s.Player.Bus
	ad, e := s.Player.Driver.Restore(w)
	if e != nil {
		t.Fatal(e)
	}
	ad.drivePath = nil
	ad.PrepareLeg(w, w.Stops[9], w.Stops[8], 52)
	ad.connectDockDeparture(b, w.Stops[9])
	t.Logf("initial head%.3f tail%.3f runIn%.1f smooth%v", b.Heading, b.TailHeading(), ad.drivePath[1].Distance(b.Pos), ad.smoothDockDeparture)
	route := &sim.Route{StopIDs: []int{9, 8}, CurrentIndex: 1}
	start := b.Pos
	for tick := 0; tick < 60*15; tick++ {
		before := b.Pos
		th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, nil, nil, 52, 1./60)
		b.Update(1./60, th, br, st, hb, doors)
		if b.Pos.Distance(before) > math.Abs(b.Speed)/60+.05 {
			t.Fatal("bus jumped")
		}
		if hit := b.CheckBoundaries(w, 52); hit.Zone == sim.ZoneHardBarrier {
			t.Fatalf("barrier %.2fs %s pos%v tail%.3f", float64(tick)/60, hit.ObstacleName, b.Pos, b.TailHeading())
		}
		if b.Pos.Distance(start) > 650 {
			t.Logf("cleared %.2fs", float64(tick)/60)
			return
		}
	}
	t.Fatal("rear berth departure did not clear")
}
