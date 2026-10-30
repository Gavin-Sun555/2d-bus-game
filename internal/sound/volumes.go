package sound

// Volumes are independent percentages. Zero is an intentional mute.
type Volumes struct {
	Engine, Announcements, Horn int
}

func DefaultVolumes() Volumes { return Volumes{100, 100, 100} }
func (v Volumes) Normalize() Volumes {
	v.Engine = max(0, min(100, v.Engine))
	v.Announcements = max(0, min(100, v.Announcements))
	v.Horn = max(0, min(100, v.Horn))
	return v
}
