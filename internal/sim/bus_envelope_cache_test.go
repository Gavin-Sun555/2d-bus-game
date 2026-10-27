package sim

import (
	"encoding/json"
	"math"
	"testing"
)

func TestBusEnvelopeCacheTracksPoseAndGeometry(t *testing.T) {
	b := NewBus(120, -80, .4)
	b.SetModel(BusArticulated)
	check := func(label string) {
		t.Helper()
		got := b.BodyEnvelope()
		want := b.computeBodyEnvelope()
		if got != want {
			t.Fatalf("%s: stale body: got%+v want%+v", label, got, want)
		}
		if repeated := b.BodyEnvelope(); repeated != want {
			t.Fatalf("%s: repeated body changed", label)
		}
	}
	check("initial")
	changes := []struct {
		name   string
		change func()
	}{
		{"position", func() { b.Pos.X += 30; b.Pos.Y -= 8 }},
		{"heading", func() { b.Heading += .1 }},
		{"tail", func() { b.TrailerHeading -= .2 }},
		{"length", func() { b.Length += 5 }},
		{"width", func() { b.Width += 3 }},
		{"uninitialized tail", func() { b.TrailerInitialized = false }},
		{"initialized tail", func() { b.TrailerInitialized = true }},
		{"legacy geometry", func() { b.GeometryVersion = 0 }},
		{"articulated geometry", func() { b.GeometryVersion = 1 }},
		{"short model", func() { b.SetModel(BusDoubleDecker) }},
		{"long model", func() { b.SetModel(BusArticulated) }},
	}
	for _, c := range changes {
		c.change()
		check(c.name)
	}
	for tick := 0; tick < 600; tick++ {
		b.Update(1./60, 1, 0, math.Sin(float64(tick)*.03)*.4, false, false)
		check("turning")
	}
	// Serialized states contain no runtime cache and restore identical geometry.
	data, err := json.Marshal(b)
	if err != nil {
		t.Fatal(err)
	}
	var restored Bus
	if err = json.Unmarshal(data, &restored); err != nil {
		t.Fatal(err)
	}
	if restored.envelopeCache.valid {
		t.Fatal("runtime cache leaked into save")
	}
	if got, want := restored.BodyEnvelope(), b.computeBodyEnvelope(); got != want {
		t.Fatal("save round trip changed body", got, want)
	}
}
