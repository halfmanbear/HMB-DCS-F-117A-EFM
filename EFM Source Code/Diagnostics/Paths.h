#pragma once

#include <string>

// Path inside the user's Saved Games\<dcsFolderName> tree, or empty when the profile is unknown.
std::string build_saved_games_dcs_path(const char* dcsFolderName, const char* relativePath);

// Path inside the Saved Games\<dcsFolderName>\Logs folder. The root honours F117_LOG_ROOT (set by the
// offline flight-model bench so its runs cannot overwrite the log from the last flight), then the
// Windows Saved Games known folder, then %USERPROFILE%\Saved Games. Empty when none is available.
std::string build_saved_games_dcs_log_path(const char* dcsFolderName, const char* fileName);
