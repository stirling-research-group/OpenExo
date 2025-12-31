#include "ResetScheduler.h"

namespace reset_scheduler
{
    static volatile bool _pending = false;
    static volatile uint32_t _due_ms = 0;

    static inline void _do_reset()
    {
    #if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
        // Teensy (ARM): request system reset via AIRCR SYSRESETREQ
        *(volatile uint32_t*)0xE000ED0C = 0x05FA0004;  // VECTKEY | SYSRESETREQ
        while (1) {}
    #elif defined(ARDUINO_ARDUINO_NANO33BLE) || defined(ARDUINO_NANO_RP2040_CONNECT)
        NVIC_SystemReset();
        while (1) {}
    #else
        NVIC_SystemReset();
        while (1) {}
    #endif
    }

    void request(uint32_t delay_ms)
    {
        _pending = true;
        _due_ms = millis() + delay_ms;
    }

    void update()
    {
        if (!_pending) return;

        const uint32_t now = millis();
        // signed subtraction handles millis() wraparound
        if ((int32_t)(now - _due_ms) >= 0)
        {
            _pending = false;
            _do_reset();
        }
    }

    bool pending()
    {
        return _pending;
    }
}
