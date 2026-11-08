package bridge

import (
	"fmt"

	"github.com/HameedFawwaz/NATHacks_2026/dashboard/internal/mavlink"
)

// ArduCopter custom_mode values used to start a mission (see copterModes in
// vehicle.go for the full table this dashboard displays).
const (
	copterModeAuto   = 3
	copterModeGuided = 4
)

// commandStep is one COMMAND_LONG in a sequence, waiting for its own
// COMMAND_ACK before the next is sent.
type commandStep struct {
	command        uint16
	param1, param2 float32
}

// startFlightSteps arms and starts the uploaded mission: switch to GUIDED so
// arming doesn't need a flight mode that insists on receiver input, arm, switch
// to AUTO, and start the mission from its first item.
var startFlightSteps = []commandStep{
	{mavlink.CmdDoSetMode, mavlink.ModeFlagCustomModeEnabled, copterModeGuided},
	{mavlink.CmdComponentArmDisarm, 1, 0},
	{mavlink.CmdDoSetMode, mavlink.ModeFlagCustomModeEnabled, copterModeAuto},
	{mavlink.CmdMissionStart, 0, 0},
}

// command is one command sequence in progress. Unlike an upload, a step is
// retried in place rather than resumed from where the vehicle asks, because
// there's nothing analogous to MISSION_REQUEST driving it.
type command struct {
	steps    []commandStep
	index    int
	lastSent func()
	progress chan struct{}
	done     chan error
}

func (c *command) finish(err error) {
	select {
	case c.done <- err:
	default:
	}
}

// commandNames describes each step for error messages.
var commandNames = map[uint16]string{
	mavlink.CmdDoSetMode:          "changing flight mode",
	mavlink.CmdComponentArmDisarm: "arming",
	mavlink.CmdMissionStart:       "starting the mission",
}

func commandName(cmd uint16) string {
	if name, ok := commandNames[cmd]; ok {
		return name
	}
	return fmt.Sprintf("command %d", cmd)
}

// commandResults names the MAV_RESULT values a COMMAND_ACK can carry.
var commandResults = map[uint8]string{
	mavlink.ResultTemporarilyRejected: "temporarily rejected",
	mavlink.ResultDenied:              "denied",
	mavlink.ResultUnsupported:         "unsupported",
	mavlink.ResultFailed:              "failed",
}

func commandResultError(cmd uint16, result uint8) error {
	name, ok := commandResults[result]
	if !ok {
		name = fmt.Sprintf("result %d", result)
	}
	return fmt.Errorf("the drone refused to start the flight (%s: %s)", commandName(cmd), name)
}
