#pragma once
#include <Arduino.h>

namespace reset_scheduler
{
    // Request a software reset after delay_ms. Non-blocking.
    void request(uint32_t delay_ms);

    // Call frequently (e.g., once per loop). Triggers reset when due.
    void update();

    // Returns true if a reset has been scheduled.
    bool pending();
}
