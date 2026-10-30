package sim

// PassengerRates are simulated seconds per passenger, with parallel door work.
type PassengerRates struct{ Boarding, Alighting float64 }

var PassengerTimeOptions = [...]float64{.25, .5, .75, 1, 1.5, 2, 3}

func (r PassengerRates) Valid() bool {
	valid := func(v float64) bool {
		for _, option := range PassengerTimeOptions {
			if v == option {
				return true
			}
		}
		return false
	}
	return valid(r.Boarding) && valid(r.Alighting)
}
func DefaultPassengerRates(model BusModel) PassengerRates {
	if model == BusArticulated {
		return PassengerRates{1, .5}
	}
	return PassengerRates{2, 1}
}
func (r PassengerRates) ForModel(model BusModel) PassengerRates {
	if r.Valid() {
		return r
	}
	return DefaultPassengerRates(model)
}
