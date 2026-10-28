package sim

import (
	"image/color"
	"math"
)

type Rect struct {
	X float64
	Y float64
	W float64
	H float64
}

func (r Rect) Contains(p Vec2) bool {
	return p.X >= r.X && p.X <= r.X+r.W && p.Y >= r.Y && p.Y <= r.Y+r.H
}

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

// LerpAngle calculates linear interpolation between angles a and b following the shortest angular distance
func LerpAngle(a, b, t float64) float64 {
	diff := NormalizeAngle(b - a)
	return NormalizeAngle(a + diff*t)
}

// RotateTowards smoothly steps an angle towards target by at most maxDelta radians
func RotateTowards(current, target, maxDelta float64) float64 {
	diff := NormalizeAngle(target - current)
	if math.Abs(diff) <= maxDelta {
		return target
	}
	if diff > 0 {
		return NormalizeAngle(current + maxDelta)
	}
	return NormalizeAngle(current - maxDelta)
}

// ColorRGBA helper
func RGBA(r, g, b, a uint8) color.RGBA {
	return color.RGBA{R: r, G: g, B: b, A: a}
}

// NRGBA helper returns non-alpha-premultiplied color for Ebitengine vector rendering
func NRGBA(r, g, b, a uint8) color.NRGBA {
	return color.NRGBA{R: r, G: g, B: b, A: a}
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
	uA := UnitFromAngle(a.Heading)
	vA := Vec2{X: -uA.Y, Y: uA.X}

	// Local axes of B
	uB := UnitFromAngle(b.Heading)
	vB := Vec2{X: -uB.Y, Y: uB.X}

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

// ResolveBusCarSeparation checks for OBB collision between a Bus and a TrafficCar,
// and applies physical positional separation and velocity damping to prevent any visual overlap.
func ResolveBusCarSeparation(bus *Bus, car *TrafficCar) bool {
	if bus == nil || car == nil {
		return false
	}
	dist := bus.Pos.Distance(car.Pos)
	if dist > (bus.OverallLength()+car.Length)*1.1 {
		return false
	}
	carOBB := OBB{
		Center:  car.Pos,
		HalfLen: car.Length / 2.0,
		HalfWid: car.Width / 2.0,
		Heading: car.Heading,
	}

	collides, normal, depth := bus.CollidesBody(carOBB)
	if !collides || depth <= 0 {
		return false
	}

	carForward := UnitFromAngle(car.Heading)
	busForward := UnitFromAngle(bus.Heading)
	toBus := bus.Pos.Sub(car.Pos)
	isCarFacingBus := carForward.Dot(toBus) > 0
	isBusFacingCar := busForward.Dot(car.Pos.Sub(bus.Pos)) > 0

	// When both vehicles are aligned in the same road corridor (e.g. queue or car following bus),
	// physical separation should strictly push longitudinally along the travel lane rather than sideways.
	headDiff := math.Abs(NormalizeAngle(bus.Heading - car.Heading))
	if headDiff < 0.7 && !bus.IsArticulated() {
		fwd := toBus.Dot(carForward)
		fwdOverlap := (bus.Length/2.0 + car.Length/2.0) - math.Abs(fwd)
		if fwdOverlap > 0 {
			depth = fwdOverlap
			if fwd >= 0 {
				normal = carForward // points from car to bus
			} else {
				normal = carForward.Mul(-1)
			}
		}
	}

	sepDepth := depth + 2.0
	// normal points from car to bus
	isBusStopped := math.Abs(bus.Speed) < 8.0
	if isBusStopped {
		bus.Pos = bus.Pos.Add(normal.Mul(sepDepth * 0.05))
		car.Pos = car.Pos.Sub(normal.Mul(sepDepth * 0.95))
	} else if car.Speed < 5.0 {
		bus.Pos = bus.Pos.Add(normal.Mul(sepDepth))
	} else {
		bus.Pos = bus.Pos.Add(normal.Mul(sepDepth * 0.7))
		car.Pos = car.Pos.Sub(normal.Mul(sepDepth * 0.3))
	}

	busVel := Vec2{X: math.Cos(bus.Heading) * bus.Speed, Y: math.Sin(bus.Heading) * bus.Speed}
	carVel := Vec2{X: math.Cos(car.Heading) * car.Speed, Y: math.Sin(car.Heading) * car.Speed}
	relVelAlongNormal := busVel.Sub(carVel).Dot(normal)

	if isCarFacingBus && (!isBusFacingCar || car.Speed > math.Abs(bus.Speed)) {
		car.Speed = 0
		car.IsBraking = true
	} else if relVelAlongNormal < 0 {
		if isBusFacingCar && bus.Speed > 5.0 {
			car.Speed = math.Max(car.Speed+20.0, math.Abs(bus.Speed)*0.8+15.0)
			car.IsBraking = false
		} else {
			car.Speed = math.Min(car.Speed, 10.0)
			car.IsBraking = true
		}
	}

	return true
}

// ResolveBusBusSeparation checks for OBB collision between two Buses,
// and applies physical positional separation to prevent any visual overlap.
func ResolveBusBusSeparation(busA, busB *Bus) bool {
	if busA == nil || busB == nil || busA == busB {
		return false
	}
	dist := busA.Pos.Distance(busB.Pos)
	if dist > (busA.OverallLength()+busB.OverallLength())*1.1 {
		return false
	}
	_ = OBB{
		Center:  busB.Pos,
		HalfLen: busB.Length / 2.0,
		HalfWid: busB.Width / 2.0,
		Heading: busB.Heading,
	}

	collides, normal, depth := CheckBusesCollision(busA, busB)
	if !collides || depth <= 0 {
		return false
	}

	headDiff := math.Abs(NormalizeAngle(busA.Heading - busB.Heading))
	if headDiff < 0.7 && !busA.IsArticulated() && !busB.IsArticulated() {
		fwdA := UnitFromAngle(busA.Heading)
		toA := busA.Pos.Sub(busB.Pos)
		fwd := toA.Dot(fwdA)
		fwdOverlap := (busA.Length/2.0 + busB.Length/2.0) - math.Abs(fwd)
		if fwdOverlap > 0 {
			depth = fwdOverlap
			if fwd >= 0 {
				normal = fwdA // points from B to A
			} else {
				normal = fwdA.Mul(-1)
			}
		}
	}

	sepDepth := depth + 2.0
	// normal points from B to A
	isAStopped := math.Abs(busA.Speed) < 5.0
	isBStopped := math.Abs(busB.Speed) < 5.0

	if isAStopped && !isBStopped {
		busA.Pos = busA.Pos.Add(normal.Mul(sepDepth * 0.05))
		busB.Pos = busB.Pos.Sub(normal.Mul(sepDepth * 0.95))
		busB.Speed = 0
	} else if isBStopped && !isAStopped {
		busA.Pos = busA.Pos.Add(normal.Mul(sepDepth * 0.95))
		busB.Pos = busB.Pos.Sub(normal.Mul(sepDepth * 0.05))
		busA.Speed = 0
	} else {
		busA.Pos = busA.Pos.Add(normal.Mul(sepDepth * 0.5))
		busB.Pos = busB.Pos.Sub(normal.Mul(sepDepth * 0.5))
		busA.Speed = -busA.Speed * 0.25
		busB.Speed = -busB.Speed * 0.25
	}

	return true
}

// ResolveCarCarSeparation checks for OBB collision between two TrafficCars,
// and applies physical positional separation to prevent any visual overlap.
func ResolveCarCarSeparation(carA, carB *TrafficCar) bool {
	if carA == nil || carB == nil || carA == carB {
		return false
	}
	delta := carA.Pos.Sub(carB.Pos)
	radius := (carA.Length + carB.Length) * 0.6
	if delta.Dot(delta) > radius*radius {
		return false
	}

	obbA := OBB{
		Center:  carA.Pos,
		HalfLen: carA.Length / 2.0,
		HalfWid: carA.Width / 2.0,
		Heading: carA.Heading,
	}
	obbB := OBB{
		Center:  carB.Pos,
		HalfLen: carB.Length / 2.0,
		HalfWid: carB.Width / 2.0,
		Heading: carB.Heading,
	}

	collides, normal, depth := CheckOBBCollision(obbA, obbB)
	if !collides || depth <= 0 {
		return false
	}

	headDiff := math.Abs(NormalizeAngle(carA.Heading - carB.Heading))
	if headDiff < 0.7 {
		fwdA := UnitFromAngle(carA.Heading)
		toA := carA.Pos.Sub(carB.Pos)
		fwd := toA.Dot(fwdA)
		fwdOverlap := (carA.Length/2.0 + carB.Length/2.0) - math.Abs(fwd)
		if fwdOverlap > 0 {
			depth = fwdOverlap
			if fwd >= 0 {
				normal = fwdA // points from B to A
			} else {
				normal = fwdA.Mul(-1)
			}
		}
	}

	sepDepth := depth + 2.0
	// normal points from B to A
	isAStopped := math.Abs(carA.Speed) < 5.0
	isBStopped := math.Abs(carB.Speed) < 5.0

	if isAStopped && !isBStopped {
		carA.Pos = carA.Pos.Add(normal.Mul(sepDepth * 0.05))
		carB.Pos = carB.Pos.Sub(normal.Mul(sepDepth * 0.95))
		carB.Speed = 0
		carB.IsBraking = true
	} else if isBStopped && !isAStopped {
		carA.Pos = carA.Pos.Add(normal.Mul(sepDepth * 0.95))
		carB.Pos = carB.Pos.Sub(normal.Mul(sepDepth * 0.05))
		carA.Speed = 0
		carA.IsBraking = true
	} else {
		carA.Pos = carA.Pos.Add(normal.Mul(sepDepth * 0.5))
		carB.Pos = carB.Pos.Sub(normal.Mul(sepDepth * 0.5))
		carA.Speed = math.Min(carA.Speed, 10.0)
		carB.Speed = math.Min(carB.Speed, 10.0)
	}

	return true
}

// UnitFromAngle returns {cos a, sin a}. math.Sincos is bit-identical to
// separate Sin/Cos calls but shares the argument reduction.
func UnitFromAngle(a float64) Vec2 {
	s, c := math.Sincos(a)
	return Vec2{X: c, Y: s}
}
