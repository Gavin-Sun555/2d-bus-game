package game

import (
	"image/color"
	"math"
)

// Vec2 represents a 2D vector
type Vec2 struct {
	X float64
	Y float64
}

func (v Vec2) Add(o Vec2) Vec2 {
	return Vec2{X: v.X + o.X, Y: v.Y + o.Y}
}

func (v Vec2) Sub(o Vec2) Vec2 {
	return Vec2{X: v.X - o.X, Y: v.Y - o.Y}
}

func (v Vec2) Mul(s float64) Vec2 {
	return Vec2{X: v.X * s, Y: v.Y * s}
}

func (v Vec2) Length() float64 {
	return math.Hypot(v.X, v.Y)
}

func (v Vec2) Normalize() Vec2 {
	l := v.Length()
	if l == 0 {
		return Vec2{}
	}
	return Vec2{X: v.X / l, Y: v.Y / l}
}

func (v Vec2) Distance(o Vec2) float64 {
	return math.Hypot(v.X-o.X, v.Y-o.Y)
}

func (v Vec2) Dot(o Vec2) float64 {
	return v.X*o.X + v.Y*o.Y
}

// Lerp calculates linear interpolation between a and b
func Lerp(a, b, t float64) float64 {
	return a + (b-a)*t
}

// Clamp restricts v between min and max
func Clamp(v, min, max float64) float64 {
	if v < min {
		return min
	}
	if v > max {
		return max
	}
	return v
}

// NormalizeAngle wraps an angle to [-pi, pi]
func NormalizeAngle(rad float64) float64 {
	for rad > math.Pi {
		rad -= 2 * math.Pi
	}
	for rad < -math.Pi {
		rad += 2 * math.Pi
	}
	return rad
}

// ColorRGBA helper
func RGBA(r, g, b, a uint8) color.RGBA {
	return color.RGBA{R: r, G: g, B: b, A: a}
}

// OBB represents an Oriented Bounding Box in 2D
type OBB struct {
	Center  Vec2
	HalfLen float64 // Half-length along heading
	HalfWid float64 // Half-width perpendicular to heading
	Heading float64
}

// CheckOBBCollision checks intersection between two OBBs using Separating Axis Theorem (SAT).
// Returns (collides, normal, depth) where normal points from B to A.
func CheckOBBCollision(a, b OBB) (bool, Vec2, float64) {
	// Broadphase circle check
	maxRadiusA := math.Hypot(a.HalfLen, a.HalfWid)
	maxRadiusB := math.Hypot(b.HalfLen, b.HalfWid)
	distCenters := a.Center.Distance(b.Center)
	if distCenters > maxRadiusA+maxRadiusB {
		return false, Vec2{}, 0
	}

	// Local axes of A
	uA := Vec2{X: math.Cos(a.Heading), Y: math.Sin(a.Heading)}
	vA := Vec2{X: -math.Sin(a.Heading), Y: math.Cos(a.Heading)}

	// Local axes of B
	uB := Vec2{X: math.Cos(b.Heading), Y: math.Sin(b.Heading)}
	vB := Vec2{X: -math.Sin(b.Heading), Y: math.Cos(b.Heading)}

	axes := [4]Vec2{uA, vA, uB, vB}
	d := a.Center.Sub(b.Center)

	minOverlap := math.MaxFloat64
	bestAxis := Vec2{}

	for _, axis := range axes {
		distProj := math.Abs(d.Dot(axis))
		rA := a.HalfLen*math.Abs(uA.Dot(axis)) + a.HalfWid*math.Abs(vA.Dot(axis))
		rB := b.HalfLen*math.Abs(uB.Dot(axis)) + b.HalfWid*math.Abs(vB.Dot(axis))

		overlap := (rA + rB) - distProj
		if overlap <= 0 {
			return false, Vec2{}, 0
		}

		if overlap < minOverlap {
			minOverlap = overlap
			bestAxis = axis
		}
	}

	// Orient normal from B towards A
	if d.Dot(bestAxis) < 0 {
		bestAxis = Vec2{X: -bestAxis.X, Y: -bestAxis.Y}
	}

	return true, bestAxis, minOverlap
}
