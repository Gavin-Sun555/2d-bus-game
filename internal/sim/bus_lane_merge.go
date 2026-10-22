package sim

// LaneMergeRequest reserves an adjacent lane for a bus returning to its route
// before a junction. Controllers refresh the intent; manual driving expires it.
type BusLaneMergeRequest struct {
	Target    Vec2
	Heading   float64
	Remaining float64
	RightTurn bool // Return to the curb corridor before following a right-turn arc.
	LeftTurn  bool // Return to or join the dedicated left-turn lane corridor.
	Is2Lane   bool // Return to the single travel lane on a 2-lane road.
	Junction  Vec2
	Joined    bool // Through-lane entry is complete; rear traffic should follow normally.
}
