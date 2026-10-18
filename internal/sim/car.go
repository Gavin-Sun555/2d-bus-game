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
	LaneIndex  int     // 0 = inner / left-turn lane, 1 = outer / through lane

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

	// Deadlock prevention
	DeadlockTimer float64 // Time in seconds spent motionless while jammed against another non-player vehicle
}
