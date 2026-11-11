// IRADS session lifecycle.
#include "stdafx.h"
#include "IradsInternal.h"

namespace CockpitInterop
{
    // Called from ed_fm_release. Cockpit teardown destroys the context's indicators, so drop every
    // pointer into that context and re-inject on the next start. The old sensor block is
    // intentionally not freed: ownership after cockpit teardown is unconfirmed, and a small leak is
    // safer than a double free.
    void ResetIRADSSession()
    {
        ClearIRADSLock();
        s_irads_ray_valid = false;
        s_irads_ray_age = 1e9;
        s_irads_analog_horizontal = s_irads_analog_vertical = 0.0f;
        s_irads_keyboard_horizontal = s_irads_keyboard_vertical = 0.0f;
        s_irads_horizontal_axis = 0.0f;
        s_irads_vertical_axis = 0.0f;
        s_irads_zoom_level = 1;
        s_irads_power = true;
        ResetCameraBridge();
        g_avTVSensor_instance = nullptr;
        s_added = false;
        Log_("ResetIRADSSession: IRADS session state cleared");
    }
} // namespace CockpitInterop
