package bridge

import (
	"errors"
	"fmt"
	"math"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/mavlink"
	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/model"
)

// BuildMission converts a plan to mission items. ArduPilot reserves item 0 for
// home and needs a takeoff before AUTO will leave the ground. The closing RTL
// matches the plan's return leg.
func BuildMission(plan model.Plan) []mavlink.MissionItem {
	items := []mavlink.MissionItem{{
		Command: mavlink.CmdNavWaypoint,
		Frame:   mavlink.FrameGlobal,
		Lat:     plan.Home.Lat,
		Lon:     plan.Home.Lon,
	}}
	if len(plan.Waypoints) == 0 {
		return items
	}
	items = append(items, mavlink.MissionItem{
		Command: mavlink.CmdNavTakeoff,
		Frame:   mavlink.FrameGlobalRelativeAlt,
		AltM:    plan.Waypoints[0].AltM,
	})
	for _, w := range plan.Waypoints {
		items = append(items, mavlink.MissionItem{
			Command: mavlink.CmdNavWaypoint,
			Frame:   mavlink.FrameGlobalRelativeAlt,
			Lat:     w.Lat,
			Lon:     w.Lon,
			AltM:    w.AltM,
		})
	}
	return append(items, mavlink.MissionItem{
		Command: mavlink.CmdNavRTL,
		Frame:   mavlink.FrameGlobalRelativeAlt,
	})
}

// ValidatePlan rejects a plan the vehicle shouldn't be given. The browser limits
// these too, but this is the last stop before the aircraft.
func ValidatePlan(plan model.Plan) error {
	if len(plan.Waypoints) == 0 {
		return errors.New("the plan has no waypoints")
	}
	if !validLatLon(plan.Home) {
		return errors.New("the home position is invalid")
	}
	prev := plan.Home
	for i, w := range plan.Waypoints {
		if !validLatLon(w.LatLon) {
			return fmt.Errorf("waypoint %d has an invalid position", i+1)
		}
		if !(w.AltM >= model.MinAltM && w.AltM <= model.MaxAltM) { // also catches NaN
			return fmt.Errorf("waypoint %d altitude must be %.0f to %.0f m", i+1, model.MinAltM, model.MaxAltM)
		}
		if d := model.DistanceM(prev, w.LatLon); d > model.MaxLegM {
			return fmt.Errorf("waypoint %d is %.0f km from the previous point; was this plan made for somewhere else?", i+1, d/1000)
		}
		prev = w.LatLon
	}
	return nil
}

func validLatLon(p model.LatLon) bool {
	return math.Abs(p.Lat) <= 90 && math.Abs(p.Lon) <= 180 && p != model.LatLon{}
}

// missionResults names the common MAV_MISSION_RESULT values.
var missionResults = map[uint8]string{
	1:  "generic error",
	2:  "unsupported frame",
	3:  "unsupported command",
	4:  "no space left",
	5:  "invalid item",
	13: "invalid sequence",
	14: "cancelled",
	15: "denied",
}

func missionResultError(result uint8) error {
	if name, ok := missionResults[result]; ok {
		return fmt.Errorf("the drone rejected the mission: %s", name)
	}
	return fmt.Errorf("the drone rejected the mission: result %d", result)
}

// upload is one mission upload in progress. The vehicle drives the transfer by
// requesting items one at a time, and the sender retries when it goes quiet.
type upload struct {
	items    []mavlink.MissionItem
	next     int    // the next item not yet requested; len(items) means all were
	resent   bool   // the final item may have been sent twice
	lastSent func() // resends the most recent message
	progress chan struct{}
	done     chan error
}

// errUnconfirmed means the mission may or may not have been stored: ArduPilot ends
// the transfer when it stores the last item, so if its acceptance gets lost, the
// resent item is answered with an error.
var errUnconfirmed = errors.New("the drone didn't confirm the mission")

func (u *upload) finish(err error) {
	select {
	case u.done <- err:
	default:
	}
}
