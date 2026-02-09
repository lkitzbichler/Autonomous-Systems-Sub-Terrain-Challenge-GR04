#pragma once

#include <cstdint>

namespace statemachine_pkg::protocol
{

enum class MissionStates : uint8_t {
    WAITING = 0,        // Waiting for ready signals to start mission
    TAKEOFF = 1,        // Takeoff from start position
    TRAVELLING = 2,     // Use basic_waypoint to reach cave entrance
    EXPLORING = 3,      // Autonomous exploration (path planning)
    RETURN_HOME = 4,    // Return to exit/home position
    LAND = 5,           // Land and stop
    DONE = 6,           // Mission finished
    ERROR = 98,         // Error state
    ABORTED = 99        // Manual abort
};

enum class Commands : uint8_t {
    TAKEOFF = 0,            // Takeoff command
    START = 1,              // Start travelling/exploring
    SWITCH_TO_EXPLORE = 2,  // Switch to exploring
    HOLD = 3,               // Hold command -> stop immediately -> hover
    RETURN_HOME = 4,        // Return home command
    LAND = 5,               // Land command
    ABORT = 99,             // Stop node
    NONE = 100              // No command
};

enum class AnswerStates : uint8_t {
    RUNNING = 0,        // Node is actively working
    DONE = 1,           // Planner completion
    UNKNOWN = 255       // Unknown/undefined status
};

}  // namespace statemachine_pkg::protocol
