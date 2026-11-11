#include "stdafx.h"

#include <cstdlib>
#include <vector>
#include <shlobj.h>

#include "Paths.h"

#pragma comment(lib, "Shell32.lib")
#pragma comment(lib, "Ole32.lib")

std::string build_saved_games_dcs_path(const char* dcsFolderName, const char* relativePath)
{
    const char* userProfile = getenv("USERPROFILE");
    if (userProfile == nullptr)
    {
        return std::string();
    }

    return std::string(userProfile) + "\\Saved Games\\" + dcsFolderName + "\\" + relativePath;
}

namespace
{
std::string narrow_from_wide(const std::wstring& wide)
{
    if (wide.empty())
    {
        return std::string();
    }

    const int requiredChars = WideCharToMultiByte(CP_ACP, 0, wide.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (requiredChars <= 1)
    {
        return std::string();
    }

    std::vector<char> buffer(requiredChars);
    WideCharToMultiByte(CP_ACP, 0, wide.c_str(), -1, buffer.data(), requiredChars, nullptr, nullptr);
    return std::string(buffer.data());
}

std::string get_saved_games_root_path()
{
    const char* logRootOverride = getenv("F117_LOG_ROOT");
    if (logRootOverride != nullptr && *logRootOverride)
    {
        return std::string(logRootOverride);
    }

    PWSTR savedGamesPath = nullptr;
    if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_SavedGames, KF_FLAG_DEFAULT, nullptr, &savedGamesPath)))
    {
        const std::wstring widePath(savedGamesPath);
        CoTaskMemFree(savedGamesPath);
        const std::string narrowPath = narrow_from_wide(widePath);
        if (!narrowPath.empty())
        {
            return narrowPath;
        }
    }

    const char* userProfile = getenv("USERPROFILE");
    if (userProfile != nullptr)
    {
        return std::string(userProfile) + "\\Saved Games";
    }

    return std::string();
}
}  // namespace

std::string build_saved_games_dcs_log_path(const char* dcsFolderName, const char* fileName)
{
    const std::string savedGamesRoot = get_saved_games_root_path();
    if (savedGamesRoot.empty())
    {
        return std::string();
    }

    return savedGamesRoot + "\\" + dcsFolderName + "\\Logs\\" + fileName;
}
