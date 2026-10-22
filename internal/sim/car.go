package sim

import (
	"image/color"
	"math"
)

type CarType int

const (
	CarSedan CarType = iota
	CarTaxi
	CarSUV
)

// TrafficCar represents an autonomous AI civilian vehicle
type TrafficCar struct {
	ID              int
	Pos             Vec2
	Heading         float64
	Speed           float64
	TargetSpeed     float64
	MaxSpeedKPH     int // Fixed at spawn; slowing for traffic never changes this limit.
	CanOvertake     bool
	PassingBaseLane int
	PassingLeadID   int
	PassingLeadBus  *Bus
	Passing         bool
	Type            CarType
	Color           color.RGBA

	Length float64
	Width  float64

	IsBraking  bool
	RoadCenter float64 // The Y or X axis of the road being followed
	IsEW       bool    // true if on East-West road, false if on North-South
	LaneIndex  int     // Counted from median: on eight-lane roads 0 left, 1/2 through, 3 right/shared bus lane.

	// Intersection turns follow the same swept corridors as buses.
	JunctionTurn     *CarJunctionTurn
	JunctionCorridor *JunctionCorridor
	YieldingToBus    *Bus // Keep a granted gap until the bus completes its merge.

	// Roundabout navigation
	InRoundabout          bool
	RoundaboutCenter      Vec2
	RoundaboutRadius      float64
	RoundaboutAngle       float64
	RoundaboutExitAngle   float64
	RoundaboutExitHeading float64
	RoundaboutExitEW      bool
	RoundaboutExitRC      float64
	RoundaboutCooldown    float64
	RoundaboutTraveled    float64
	RoundaboutTargetR     float64
	RoundaboutOuterRadius float64
	RoundaboutExitOffset  float64
	RoundaboutExitLane    int
	RoundaboutCurve       *CarRoundaboutCurve

	// Deadlock prevention
	DeadlockTimer   float64 // Time in seconds spent motionless while jammed against another non-player vehicle
	DeadlockTracker StationaryTracker
}

// CarJunctionTurn retains a continuous path until the vehicle clears its arc.
type CarJunctionTurn struct {
	EntryHeading   float64 // Original approach heading, retained while rotating.
	SpeedLimit     float64 // Optional approach-merge cap; zero uses normal cruising.
	Center         Vec2    // Junction owning this trajectory; excludes unrelated crossings.
	Straight       bool    // Straight-through lane continuity across road-width changes.
	LaneChange     bool    // Approach merge using the same continuous path follower; signals still apply.
	Points         []Vec2
	Segment        int
	Progress       float64
	ExitHeading    float64
	ExitEW         bool
	ExitRoadCenter float64
	ExitLane       int
}

// CarRoundaboutCurve is a tangent-continuous join parameterized by driving
// distance. The lookup table avoids speed changes along a cubic Bezier curve.
type CarRoundaboutCurve struct {
	Start, Control1, Control2, End Vec2
	ArcLengths                     [65]float64
	Distance                       float64
	Exiting                        bool
	Entry                          *RoundaboutJoin
}

// Predict follows the committed arc by travelled distance, then continues on
// its outgoing lane. It never mutates the live follower's progress.
func (turn *CarJunctionTurn) Predict(distance float64) (Vec2, float64) {
	if turn == nil || len(turn.Points) == 0 {
		return Vec2{}, 0
	}
	segment := turn.Segment
	distance = math.Max(0, distance) + turn.Progress
	for segment < len(turn.Points)-1 {
		a, b := turn.Points[segment], turn.Points[segment+1]
		delta := b.Sub(a)
		length := delta.Length()
		if distance <= length && length > 0 {
			return a.Add(delta.Mul(distance / length)), math.Atan2(delta.Y, delta.X)
		}
		distance -= length
		segment++
	}
	u := Vec2{X: math.Cos(turn.ExitHeading), Y: math.Sin(turn.ExitHeading)}
	return turn.Points[len(turn.Points)-1].Add(u.Mul(distance)), turn.ExitHeading
}
