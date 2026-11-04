package sim

import "github.com/Gavin-Sun555/2d-bus-game/internal/locale"

// PassengerSpawnRate defines the arrival frequency of passengers at bus stops.
type PassengerSpawnRate int

const (
	PassengerSpawnModerate PassengerSpawnRate = 0 // 适中 (默认)
	PassengerSpawnFew      PassengerSpawnRate = 1 // 偏少
	PassengerSpawnMany     PassengerSpawnRate = 2 // 偏多
)

// Multiplier returns the time advancement factor for stop passenger generation.
func (r PassengerSpawnRate) Multiplier() float64 {
	switch r {
	case PassengerSpawnFew:
		return 0.5
	case PassengerSpawnMany:
		return 2.0
	default:
		return 1.0
	}
}

// Label returns the localized display text for the setting button.
func (r PassengerSpawnRate) Label() string {
	switch r {
	case PassengerSpawnFew:
		return locale.T("【偏少】", "【Few】")
	case PassengerSpawnMany:
		return locale.T("【偏多】", "【Many】")
	default:
		return locale.T("【适中 (默认)】", "【Moderate (Default)】")
	}
}

// Valid returns whether the spawn rate is a recognized option.
func (r PassengerSpawnRate) Valid() bool {
	return r >= PassengerSpawnModerate && r <= PassengerSpawnMany
}
