// tasks.hpp
//
// Declares the function to start all FreeRTOS tasks used in the project.
// The implementation in tasks.cpp will create:
//  - physics_task   (50 Hz)
//  - flock_task     (10 Hz)
//  - radio_task     (4 Hz)
//  - telemetry_task (2 Hz)

#pragma once

#include "control.hpp"

namespace tasks {

    // Create and start all application tasks.
    // Called once from app_main() in main.cpp.
    void start_all_tasks();

    // Get a snapshot of the current local state (thread-safe copy)
    control::LocalState get_local_state();

} // namespace tasks

// For convenience in main.cpp:
using tasks::start_all_tasks;
using tasks::get_local_state;
