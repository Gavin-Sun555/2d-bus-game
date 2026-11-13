package sim

import "math"

// Fictional manufacturers keep the vehicle economy independent of real brands.
type VehicleSpec struct {
	ID, Name, EnglishName                                                   string
	Model                                                                   BusModel
	Price                                                                   int
	TopSpeedKPH, Acceleration, CostPerKM, RepairPerPoint, DepreciationPerKM float64
	Color                                                                   [3]uint8
	Style                                                                   int
}

var vehicleCatalog = [...]VehicleSpec{
	{"xingtu-s40", "星途 S40 城市", "Xingtu S40 City", BusStandard, 400000, 80, 190, 3.2, 900, 1.2, [3]uint8{34, 197, 94}, 0},
	{"yuanchuan-s70", "远川 S70 旅行", "Yuanchuan S70 Touring", BusStandard, 700000, 100, 230, 4.0, 1400, 1.8, [3]uint8{14, 165, 233}, 1},
	{"lingfeng-s100", "凌峰 S100 旗舰", "Lingfeng S100 Prime", BusStandard, 1000000, 120, 270, 5.2, 2000, 2.5, [3]uint8{245, 158, 11}, 2},
	{"xingtu-a100", "星途 A100 联城", "Xingtu A100 Link", BusArticulated, 1000000, 80, 165, 5.5, 1700, 2.5, [3]uint8{20, 184, 166}, 0},
	{"yuanchuan-a130", "远川 A130 干线", "Yuanchuan A130 Trunk", BusArticulated, 1300000, 100, 195, 6.5, 2200, 3.2, [3]uint8{99, 102, 241}, 1},
	{"lingfeng-a160", "凌峰 A160 快线", "Lingfeng A160 Express", BusArticulated, 1600000, 110, 225, 8.0, 2800, 4.0, [3]uint8{244, 63, 94}, 2},
	{"xingtu-d100", "星途 D100 云景", "Xingtu D100 Skyline", BusDoubleDecker, 1000000, 80, 170, 5.0, 1800, 2.5, [3]uint8{6, 182, 212}, 0},
	{"yuanchuan-d140", "远川 D140 都会", "Yuanchuan D140 Metro", BusDoubleDecker, 1400000, 100, 205, 6.2, 2500, 3.5, [3]uint8{168, 85, 247}, 1},
	{"lingfeng-d180", "凌峰 D180 天际", "Lingfeng D180 Horizon", BusDoubleDecker, 1800000, 110, 240, 7.8, 3200, 4.5, [3]uint8{249, 115, 22}, 2},
}

func VehicleCatalog() []VehicleSpec { return append([]VehicleSpec(nil), vehicleCatalog[:]...) }
func FindVehicleSpec(id string) (VehicleSpec, bool) {
	for _, s := range vehicleCatalog {
		if s.ID == id {
			return s, true
		}
	}
	return VehicleSpec{}, false
}
func (s VehicleSpec) ResaleValue(km float64) int {
	return int(math.Max(0, float64(s.Price)*.8-math.Max(0, km)*s.DepreciationPerKM))
}
func (s VehicleSpec) RepairQuote(condition float64) int {
	return int(math.Ceil(math.Max(0, 100-condition)*s.RepairPerPoint - 1e-7))
}
func (b *Bus) ApplyCatalogLimits() {
	if s, ok := FindVehicleSpec(b.CatalogID); ok {
		b.MaxSpeedFwd = math.Min(b.MaxSpeedFwd, SpeedKPH(int(s.TopSpeedKPH)))
		b.AccelRate = s.Acceleration
	}
}
