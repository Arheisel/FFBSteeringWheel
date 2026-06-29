#pragma once
#include "shared_state.h"

// Set the global shared state pointer for Core 1
void core1_set_shared_state(SharedState &state);

// Entry point for Core 1
void core1_main();
