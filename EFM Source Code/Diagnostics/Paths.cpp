#include "stdafx.h"

#include <vector>
#include <shlobj.h>

#include "Paths.h"

#pragma comment(lib, "Shell32.lib")
#pragma comment(lib, "Ole32.lib")

static constexpr size_t environment_value_bytes = 4096;

namespace
{
// Value of an environment variable, or an empty string when it is unset or empty.
std::string read_environment_variable(const char* name)
{
    char buffer[environment_value_bytes];
    const DWORD length = GetEnvironmentVariableA(name, buffer, sizeof(buffer));
    if (length == 0 || length >= sizeof(buffer))
    {
        return std::string();
    }
    return std::string(buffer, length);
}
}  // namespace

std::string build_saved_games_dcs_path(const char* dcs_folder_name, const char* relative_path)
{
    const std::string user_profile = read_environment_variable("USERPROFILE");
    if (user_profile.empty())
    {
        return std::string();
    }

    return user_profile + "\\Saved Games\\" + dcs_folder_name + "\\" + relative_path;
}

namespace
{
std::string narrow_from_wide(const std::wstring& wide)
{
    if (wide.empty())
    {
        return std::string();
    }

    const int required_chars = WideCharToMultiByte(CP_ACP, 0, wide.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (required_chars <= 1)
    {
        return std::string();
    }

    std::vector<char> buffer(required_chars);
    WideCharToMultiByte(CP_ACP, 0, wide.c_str(), -1, buffer.data(), required_chars, nullptr, nullptr);
    return std::string(buffer.data());
}

std::string get_saved_games_root_path()
{
    const std::string log_root_override = read_environment_variable("F117_LOG_ROOT");
    if (!log_root_override.empty())
    {
        return log_root_override;
    }

    PWSTR saved_games_path = nullptr;
    if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_SavedGames, KF_FLAG_DEFAULT, nullptr, &saved_games_path)))
    {
        const std::wstring wide_path(saved_games_path);
        CoTaskMemFree(saved_games_path);
        const std::string narrow_path = narrow_from_wide(wide_path);
        if (!narrow_path.empty())
        {
            return narrow_path;
        }
    }

    const std::string user_profile = read_environment_variable("USERPROFILE");
    if (!user_profile.empty())
    {
        return user_profile + "\\Saved Games";
    }

    return std::string();
}
}  // namespace

std::string build_saved_games_dcs_log_path(const char* dcs_folder_name, const char* file_name)
{
    const std::string saved_games_root = get_saved_games_root_path();
    if (saved_games_root.empty())
    {
        return std::string();
    }

    return saved_games_root + "\\" + dcs_folder_name + "\\Logs\\" + file_name;
}
