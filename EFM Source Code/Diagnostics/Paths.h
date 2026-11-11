#pragma once

#include <string>

// Path inside the user's Saved Games\<dcsFolderName> tree, or empty when the profile is unknown.
std::string build_saved_games_dcs_path(const char* dcsFolderName, const char* relativePath);
