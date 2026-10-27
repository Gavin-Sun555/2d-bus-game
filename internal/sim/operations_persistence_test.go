package sim

import (
	"encoding/json"
	"math"
	"testing"
)

func TestOperationsSaveCompletesUnfinishedLap(t *testing.T) {
	o := simpleOperations()
	b := NewBus(0, 0, 0)
	r := &Route{StopIDs: []int{0, 1, 2, 0}}
	o.Track(0, b, r, 0)
	b.Speed = SpeedKPH(36)
	for i := 0; i < 50; i++ {
		b.Pos.X += b.Speed
		o.Track(0, b, r, 1)
	}
	raw, err := json.Marshal(o.Snapshot())
	if err != nil {
		t.Fatal(err)
	}
	var snapshot OperationsSnapshot
	if err = json.Unmarshal(raw, &snapshot); err != nil {
		t.Fatal(err)
	}
	restored := RestoreOperations(snapshot, nil, r, 52, DefaultFarePolicy(), map[int]*Bus{0: b}, map[int]*Route{0: r})
	// The simple fixture uses a supplied distance table instead of a world.
	restored.Distances = o.Distances
	for i := 0; i < 50; i++ {
		b.Pos.X += b.Speed
		restored.Track(0, b, r, 1)
	}
	b.Speed = 0
	for i := 0; i < 100; i++ {
		restored.Track(0, b, r, 1)
	}
	r.TotalTrips++
	restored.Track(0, b, r, .001)
	speed, ok := restored.Vehicle(0).AverageKPH()
	if !ok || math.Abs(speed-18) > .01 || restored.Vehicle(0).CompletedLaps != 1 {
		t.Fatalf("unfinished lap lost at save: speed %.3f, valid %v", speed, ok)
	}
}
