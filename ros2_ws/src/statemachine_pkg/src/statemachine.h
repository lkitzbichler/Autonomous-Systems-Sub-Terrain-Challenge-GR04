#pragma once

#include <rclcpp/rclcpp.hpp>

class StateMachine : public rclcpp::Node
{
public:
    StateMachine();

private:
    enum class States {
        INIT, // All nodes booting
        READY, // All nodes ready & running -> send start commands
        BUSY, // All nodes running -> Waiting for a change of state
        DONE, // Finished
        CRASH, // Crashed
        ERROR, // Error
        ABORT // Aborted due to manual abortion, error or crash
    };

    enum class FlyStates {
        INIT, // All nodes booting
        TAKEOFF, // Drone shall start and not crash within 3 s
        SCANNING, // Scanning for obstacles -> flying around object 
        GOTO_ENTRANCE, // On its way to the entrance
        ARRIVE_ENTRANCE, // Arrived at the entrance
        WAITING, // Waiting for computational reasons
        LAND, // Landing
        DONE, // Finished Landing Successfully
        CRASH, // Crashed
        ABORT // Aborted
    };

    enum class OdomStates {
        INIT,
        PLAN,
        WAIT_MAP_READY,
        PLAN_PATH,
        PLAN_TRAJ,
        TRACK,
        EXECUTE,
        DONE,
        ABORT
    };

protected:


};