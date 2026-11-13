package sim

import (
	"fmt"
	"math"
)

const CareerStartingCash = 200000
const MaxOwnedVehicles = 49

type OwnedVehicle struct {
	ID            int
	SpecID        string
	KM, Condition float64
	// Line is -1 for the depot, otherwise a zero-based service. The player's
	// vehicle is identified by PlayerVehicleID and does not consume an AI slot.
	Line int
}
type CareerProfile struct {
	DriverName                                  string
	Balance                                     float64
	Vehicles                                    []OwnedVehicle
	PlayerVehicleID, NextVehicleID, PeakFleet   int
	TotalKM, OperatingCost, RepairCost, Revenue float64
	Passengers                                  int
	Achievements                                map[string]bool
}
type CareerAchievement struct {
	ID, Name, EnglishName string
	Reward                int
}

var careerAchievements = [...]CareerAchievement{
	{"first-service", "初次运营：运送 25 人", "First service: 25 passengers", 50000},
	{"people-100", "社区伙伴：运送 100 人", "Community partner: 100 passengers", 100000},
	{"km-10", "城市探索：运营 10 公里", "City explorer: 10 km", 100000},
	{"fleet-3", "车队初成：拥有 3 辆车", "Fleet builder: own 3 buses", 100000},
	{"fleet-5", "跨城经营：拥有 5 辆车", "Regional operator: own 5 buses", 150000},
	{"km-100", "百里通勤：运营 100 公里", "Commuter network: 100 km", 200000},
	{"people-1000", "城市伙伴：运送 1000 人", "City partner: 1000 passengers", 250000},
	{"fleet-8", "公交集团：拥有 8 辆车", "Transit group: own 8 buses", 300000},
}

func CareerAchievements() []CareerAchievement {
	return append([]CareerAchievement(nil), careerAchievements[:]...)
}
func NewCareerProfile() *CareerProfile {
	return &CareerProfile{DriverName: "新任司机", Balance: CareerStartingCash, Vehicles: []OwnedVehicle{{ID: 1, SpecID: vehicleCatalog[0].ID, Condition: 100, Line: 0}}, PlayerVehicleID: 1, NextVehicleID: 2, PeakFleet: 1, Achievements: map[string]bool{}}
}
func (c *CareerProfile) Vehicle(id int) *OwnedVehicle {
	if c != nil {
		for i := range c.Vehicles {
			if c.Vehicles[i].ID == id {
				return &c.Vehicles[i]
			}
		}
	}
	return nil
}
func (c *CareerProfile) Buy(specID string) (int, error) {
	s, ok := FindVehicleSpec(specID)
	if !ok {
		return 0, fmt.Errorf("未知车型 / Unknown vehicle")
	}
	if len(c.Vehicles) >= MaxOwnedVehicles {
		return 0, fmt.Errorf("车库已满 / Garage full")
	}
	if c.Balance < float64(s.Price) {
		return 0, fmt.Errorf("资金不足 / Insufficient funds")
	}
	id := c.NextVehicleID
	c.NextVehicleID++
	c.Balance -= float64(s.Price)
	c.Vehicles = append(c.Vehicles, OwnedVehicle{ID: id, SpecID: s.ID, Condition: 100, Line: -1})
	c.PeakFleet = max(c.PeakFleet, len(c.Vehicles))
	return id, nil
}
func (c *CareerProfile) Sell(id int) (int, error) {
	v := c.Vehicle(id)
	if v == nil {
		return 0, fmt.Errorf("车辆不存在 / Vehicle missing")
	}
	if id == c.PlayerVehicleID || v.Line >= 0 {
		return 0, fmt.Errorf("请先将非玩家车辆调回车库 / Return a non-player vehicle to depot first")
	}
	s, _ := FindVehicleSpec(v.SpecID)
	price := s.ResaleValue(v.KM)
	for i := range c.Vehicles {
		if c.Vehicles[i].ID == id {
			c.Vehicles = append(c.Vehicles[:i], c.Vehicles[i+1:]...)
			break
		}
	}
	c.Balance += float64(price)
	return price, nil
}
func (c *CareerProfile) Repair(id int) (int, error) {
	v := c.Vehicle(id)
	if v == nil {
		return 0, fmt.Errorf("车辆不存在 / Vehicle missing")
	}
	s, _ := FindVehicleSpec(v.SpecID)
	quote := s.RepairQuote(v.Condition)
	if c.Balance < float64(quote) {
		return 0, fmt.Errorf("维修资金不足 / Cannot afford repair")
	}
	c.Balance -= float64(quote)
	c.RepairCost += float64(quote)
	v.Condition = 100
	return quote, nil
}
func (c *CareerProfile) MapUnlocked(theme MapTheme) bool {
	if c == nil {
		return false
	}
	thresholds := map[MapTheme]int{MapThemeMetropolis: 1, MapThemeCoastalBay: 3, MapThemeRiverValley: 5, MapThemeTwinCities: 8}
	n, ok := thresholds[theme]
	return ok && c.PeakFleet >= n
}
func (c *CareerProfile) AchievementReady(id string) bool {
	switch id {
	case "first-service":
		return c.Passengers >= 25
	case "people-100":
		return c.Passengers >= 100
	case "people-1000":
		return c.Passengers >= 1000
	case "km-10":
		return c.TotalKM >= 10
	case "km-100":
		return c.TotalKM >= 100
	case "fleet-3":
		return c.PeakFleet >= 3
	case "fleet-5":
		return c.PeakFleet >= 5
	case "fleet-8":
		return c.PeakFleet >= 8
	}
	return false
}
func (c *CareerProfile) Claim(id string) (int, error) {
	for _, a := range careerAchievements {
		if a.ID == id {
			if c.Achievements[id] || !c.AchievementReady(id) {
				return 0, fmt.Errorf("成就未达成或已领取 / Not ready or already claimed")
			}
			if c.Achievements == nil {
				c.Achievements = map[string]bool{}
			}
			c.Achievements[id] = true
			c.Balance += float64(a.Reward)
			return a.Reward, nil
		}
	}
	return 0, fmt.Errorf("未知成就 / Unknown achievement")
}
func finiteCareer(x float64) bool { return !math.IsNaN(x) && !math.IsInf(x, 0) && x >= 0 }
func (c *CareerProfile) Validate() error {
	bad := func() error { return fmt.Errorf("invalid career profile") }
	if c == nil || math.IsNaN(c.Balance) || math.IsInf(c.Balance, 0) || math.Abs(c.Balance) > 1e12 || len(c.Vehicles) < 1 || len(c.Vehicles) > MaxOwnedVehicles || c.PeakFleet < len(c.Vehicles) || c.PeakFleet > MaxOwnedVehicles || c.NextVehicleID < 2 || len(c.DriverName) > 96 || !finiteCareer(c.TotalKM) || !finiteCareer(c.OperatingCost) || !finiteCareer(c.RepairCost) || !finiteCareer(c.Revenue) || c.Passengers < 0 {
		return bad()
	}
	ids := map[int]bool{}
	counts := [6]int{}
	for _, v := range c.Vehicles {
		_, ok := FindVehicleSpec(v.SpecID)
		if !ok || v.ID < 1 || v.ID >= c.NextVehicleID || ids[v.ID] || !finiteCareer(v.KM) || !finiteCareer(v.Condition) || v.Condition > 100 || v.Line < -1 || v.Line > 5 {
			return bad()
		}
		ids[v.ID] = true
		if v.ID != c.PlayerVehicleID && v.Line >= 0 {
			counts[v.Line]++
		}
	}
	if !ids[c.PlayerVehicleID] {
		return bad()
	}
	for _, n := range counts {
		if n > MaxBusesPerLine {
			return bad()
		}
	}
	for id := range c.Achievements {
		known := false
		for _, a := range careerAchievements {
			if a.ID == id {
				known = true
			}
		}
		if !known {
			return bad()
		}
	}
	return nil
}
