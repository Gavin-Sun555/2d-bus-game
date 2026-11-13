package sim

import (
	"math"
	"testing"
)

func TestCareerCatalogAndTransactions(t *testing.T) {
	counts := map[BusModel]int{}
	seen := map[string]bool{}
	for _, s := range VehicleCatalog() {
		if seen[s.ID] {
			t.Fatal("duplicate catalog ID")
		}
		seen[s.ID] = true
		counts[s.Model]++
		low, high := 400000, 1000000
		if s.Model == BusArticulated {
			low, high = 1000000, 1600000
		}
		if s.Model == BusDoubleDecker {
			low, high = 1000000, 1800000
		}
		if s.Price < low || s.Price > high || s.Acceleration <= 0 || s.CostPerKM <= 0 || s.RepairPerPoint <= 0 || s.DepreciationPerKM <= 0 {
			t.Fatal("invalid spec", s)
		}
		if s.ResaleValue(100) != int(float64(s.Price)*.8-100*s.DepreciationPerKM) || s.ResaleValue(1e9) != 0 {
			t.Fatal("resale formula", s)
		}
		if s.RepairQuote(94) != int(6*s.RepairPerPoint) {
			t.Fatal("repair formula")
		}
	}
	for model := BusStandard; model < BusModelCount; model++ {
		if counts[model] != 3 {
			t.Fatal("missing model choices", model)
		}
	}
	c := NewCareerProfile()
	if c.Validate() != nil || len(c.Vehicles) != 1 {
		t.Fatal("bad starter")
	}
	before := c.Balance
	if _, err := c.Buy("xingtu-s40"); err == nil || c.Balance != before || len(c.Vehicles) != 1 {
		t.Fatal("unaffordable purchase changed profile")
	}
	c.Balance = 1000000
	id, err := c.Buy("xingtu-s40")
	if err != nil {
		t.Fatal(err)
	}
	v := c.Vehicle(id)
	v.KM = 100
	v.Condition = 90
	repair, err := c.Repair(id)
	if err != nil || repair != 9000 || c.Vehicle(id).Condition != 100 {
		t.Fatal("repair transaction", repair, err)
	}
	balance := c.Balance
	price, err := c.Sell(id)
	if err != nil || price != 319880 || c.Balance != balance+float64(price) || c.Vehicle(id) != nil {
		t.Fatal("sale transaction", price, err)
	}
	if _, err = c.Sell(c.PlayerVehicleID); err == nil {
		t.Fatal("sold player's final bus")
	}
}
func TestCareerUnlocksAndAchievementClaims(t *testing.T) {
	c := NewCareerProfile()
	if !c.MapUnlocked(MapThemeMetropolis) || c.MapUnlocked(MapThemeCoastalBay) || c.MapUnlocked(MapThemeCustom) {
		t.Fatal("wrong initial access")
	}
	c.Balance = 20000000
	for i := 1; i < 8; i++ {
		if _, err := c.Buy("xingtu-s40"); err != nil {
			t.Fatal(err)
		}
	}
	for _, th := range BuiltinMapThemes() {
		if !c.MapUnlocked(th) {
			t.Fatal("city not unlocked", th)
		}
	}
	balance := c.Balance
	reward, err := c.Claim("fleet-8")
	if err != nil || reward != 300000 || c.Balance != balance+300000 {
		t.Fatal("claim", err)
	}
	if _, err = c.Claim("fleet-8"); err == nil {
		t.Fatal("duplicate grant")
	}
	if _, err = c.Claim("km-10"); err == nil {
		t.Fatal("unearned grant")
	}
	for len(c.Vehicles) > 1 {
		if _, err = c.Sell(c.Vehicles[len(c.Vehicles)-1].ID); err != nil {
			t.Fatal(err)
		}
	}
	if !c.MapUnlocked(MapThemeTwinCities) {
		t.Fatal("sale relocked earned city")
	}
	c.Vehicles[0].KM = math.NaN()
	if c.Validate() == nil {
		t.Fatal("accepted invalid mileage")
	}
}
func TestCareerBusHardwareLimits(t *testing.T) {
	for _, s := range VehicleCatalog() {
		b := NewBus(0, 0, 0)
		b.SetModel(s.Model)
		b.CatalogID = s.ID
		b.MaxSpeedFwd = SpeedKPH(200)
		b.Speed = SpeedKPH(int(s.TopSpeedKPH))
		b.Update(1./60, 1, 0, 0, false, false)
		if b.AccelRate != s.Acceleration || b.MaxSpeedFwd != SpeedKPH(int(s.TopSpeedKPH)) || b.Speed > b.MaxSpeedFwd+1e-7 {
			t.Fatal("catalog performance not enforced", s.ID, b.Speed, b.MaxSpeedFwd)
		}
	}
}
