#include "stdafx.h"

#include <cstdlib>

#include "Paths.h"

std::string build_saved_games_dcs_path(const char* dcsFolderName, const char* relativePath)
{
    const char* userProfile = getenv("USERPROFILE");
    if (userProfile == nullptr)
    {
        return std::string();
    }

    return std::string(userProfile) + "\\Saved Games\\" + dcsFolderName + "\\" + relativePath;
}

