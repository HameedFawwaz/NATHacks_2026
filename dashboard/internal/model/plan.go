// Package model holds the dashboard's data types and the rules for presenting
// them. It has no browser dependencies, so it can be tested natively.
package model

import (
	"math"
	"slices"
	"time"
)

const earthRadiusM = 6371000.0

// CruiseSpeedMS is the speed used to estimate flight time for a plan.
const CruiseSpeedMS = 8.0

// DefaultAltM is the altitude given to a newly added waypoint.
const DefaultAltM = 20.0

// Altitude limits for a waypoint, in metres above home. The ceiling is the usual
// legal limit for small drones.
const (
	MinAltM = 1.0
	MaxAltM = 120.0
)

// DefaultHome is ArduPilot SITL's default start location, so the map lines up
// with a SITL flight out of the box.
var DefaultHome = LatLon{Lat: -35.363262, Lon: 149.165237}

type LatLon struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
}

// DistanceM returns the great-circle distance between two points.
func DistanceM(a, b LatLon) float64 {
	dLat := rad(b.Lat - a.Lat)
	dLon := rad(b.Lon - a.Lon)
	h := math.Pow(math.Sin(dLat/2), 2) +
		math.Cos(rad(a.Lat))*math.Cos(rad(b.Lat))*math.Pow(math.Sin(dLon/2), 2)
	return 2 * earthRadiusM * math.Asin(math.Min(1, math.Sqrt(h)))
}

// ToLocal returns p's position in metres east and north of origin. It is a flat
// approximation, accurate at the scale of a single flight.
func ToLocal(origin, p LatLon) (eastM, northM float64) {
	eastM = rad(p.Lon-origin.Lon) * earthRadiusM * math.Cos(rad(origin.Lat))
	northM = rad(p.Lat-origin.Lat) * earthRadiusM
	return eastM, northM
}

// FromLocal is the inverse of ToLocal.
func FromLocal(origin LatLon, eastM, northM float64) LatLon {
	return LatLon{
		Lat: origin.Lat + deg(northM/earthRadiusM),
		Lon: origin.Lon + deg(eastM/(earthRadiusM*math.Cos(rad(origin.Lat)))),
	}
}

func rad(d float64) float64 { return d * math.Pi / 180 }
func deg(r float64) float64 { return r * 180 / math.Pi }

type Waypoint struct {
	LatLon
	AltM float64 `json:"alt_m"`
}

// Plan is a route that starts at Home, visits Waypoints in order and returns.
type Plan struct {
	Home      LatLon     `json:"home"`
	Waypoints []Waypoint `json:"waypoints"`
}

// Clone returns a copy that shares nothing with p.
func (p Plan) Clone() Plan {
	p.Waypoints = slices.Clone(p.Waypoints)
	return p
}

// DefaultPlan returns the plan shown on first launch.
func DefaultPlan() Plan {
	at := func(eastM, northM float64) Waypoint {
		return Waypoint{LatLon: FromLocal(DefaultHome, eastM, northM), AltM: DefaultAltM}
	}
	return Plan{
		Home:      DefaultHome,
		Waypoints: []Waypoint{at(120, 90), at(300, 110), at(340, -80), at(160, -150)},
	}
}

// LengthM returns the length of the whole route, including the leg back home.
func (p Plan) LengthM() float64 {
	if len(p.Waypoints) == 0 {
		return 0
	}
	total := 0.0
	prev := p.Home
	for _, w := range p.Waypoints {
		total += DistanceM(prev, w.LatLon)
		prev = w.LatLon
	}
	return total + DistanceM(prev, p.Home)
}

// EstimatedDuration returns the time to fly the route at CruiseSpeedMS. It
// leaves out takeoff and landing.
func (p Plan) EstimatedDuration() time.Duration {
	return time.Duration(p.LengthM() / CruiseSpeedMS * float64(time.Second))
}
