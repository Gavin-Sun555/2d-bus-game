package sim

import (
	"fmt"
	"math"
	"testing"
)

func TestOpposingLeftTurnCorridorsSeparate(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, busLane := range []bool{false, true} {
			t.Run(fmt.Sprintf("w%.0f/busLane%v", lw, busLane), func(t *testing.T) {
				half := 2 * lw
				offset := .5 * lw
				if busLane {
					half = 4 * lw
					offset = 3.5 * lw
				}
				g := NewJunctionGeometry(half, 2*lw, lw)
				a := QuarterTurnPath(Vec2{}, Vec2{X: 1}, Vec2{Y: -1}, offset, .5*lw, g.TurnRadius)
				b := QuarterTurnPath(Vec2{}, Vec2{X: -1}, Vec2{Y: 1}, .5*lw, 1.5*lw, g.TurnRadius)
				for i := 1; i < len(a); i++ {
					for j := 1; j < len(b); j++ {
						ba := OBB{Center: a[i], Heading: math.Atan2(a[i].Y-a[i-1].Y, a[i].X-a[i-1].X), HalfLen: 48, HalfWid: 18}
						bb := OBB{Center: b[j], Heading: math.Atan2(b[j].Y-b[j-1].Y, b[j].X-b[j-1].X), HalfLen: 24, HalfWid: 11}
						if hit, _, _ := CheckOBBCollision(ba, bb); hit {
							t.Fatalf("opposing bus/car swept corridors overlap at %d/%d", i, j)
						}
					}
				}
			})
		}
	}
}
