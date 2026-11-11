#pragma once

// Named cockpit parameters shared with the mod's Lua devices. The handle is resolved on first use
// and cached by the caller, so each call site keeps its own static handle.

// Lazily resolve and publish a cockpit param by name.
void publish_named_param(void*& handle, const char* name, double value);

// Lazily resolve and read a cockpit param that the mod's Lua devices publish. False without the cockpit API.
bool read_named_param(void*& handle, const char* name, double& value);
