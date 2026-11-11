#pragma once

// Engine and yaw debug CSV (F117_Engine_Debug.csv in the DCS Logs folder).
// Called once per simulation frame; writes a row every kEngineLogIntervalFrames calls.
void log_engine_debug_sample();
