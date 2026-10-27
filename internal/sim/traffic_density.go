package sim

const (
	TrafficDensityLevels = 4
	MaxTrafficCars       = 864
)

// TrafficCarCount maps the four settings to low, medium, high and congested.
// Unknown levels retain the default medium density.
func TrafficCarCount(level int) int {
	counts := [TrafficDensityLevels]int{144, 288, 576, MaxTrafficCars}
	if level < 1 || level > len(counts) {
		level = 2
	}
	return counts[level-1]
}
