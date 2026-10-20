package sim

import (
	"image/color"
)

type CarType int

const (
	CarSedan CarType = iota
	CarTaxi
	CarSUV
)

// TrafficCar represents an autonomous AI civilian vehicle
type TrafficCar struct {
	ID          int
	Pos         Vec2
	Heading     float64
	Speed       float64
	TargetSpeed float64
	Type        CarType
	Color       color.RGBA

	Length float64
	Width  float64

	IsBraking  bool
	RoadCenter float64 // The Y or X axis of the road being followed
	IsEW       bool    // true if on East-West road, false if on North-South
	LaneIndex  int     // Counted from median: on eight-lane roads 0 left, 1/2 through, 3 right/shared bus lane.

	// Intersection turns follow the same swept corridors as buses.
	JunctionTurn  *CarJunctionTurn
	YieldingToBus *Bus // Keep a granted station gap until the bus completes its merge.

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
	LaneChange     bool // Approach merge using the same continuous path follower; signals still apply.
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
}
