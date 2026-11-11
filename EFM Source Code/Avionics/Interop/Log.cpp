// IRADS interop log: F117_IRADS.log beside the DLL, bounded in size.
#include "stdafx.h"
#include "IradsInternal.h"
#include "../../Diagnostics/LogFile.h"
#include <cstdarg>
#include <cstring>
#include <string>

namespace CockpitInterop
{
// Keeps the log bounded on disk. When the DLL loads, a log over kLogRotateBytes becomes
// F117_IRADS.old.log, replacing the previous one. One load stops writing to the file after
// kLogLoadBytes, so a runaway message can't fill the disk or push out the older log.
static constexpr unsigned long long kLogRotateBytes = 10ULL * 1024 * 1024;
static constexpr unsigned long long kLogLoadBytes = 20ULL * 1024 * 1024;
static constexpr size_t kLogLineBytes = 512;

static LogFile s_log;
static unsigned long long s_log_load_bytes = 0;

static void RotateLogIfLarge(const std::string& directory, const std::string& path)
{
    WIN32_FILE_ATTRIBUTE_DATA info = {};
    if (!GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &info))
        return;
    const unsigned long long size = (static_cast<unsigned long long>(info.nFileSizeHigh) << 32) | info.nFileSizeLow;
    if (size <= kLogRotateBytes)
        return;
    // On failure the log keeps appending to the same file, and the next load tries again.
    MoveFileExA(path.c_str(), (directory + "F117_IRADS.old.log").c_str(), MOVEFILE_REPLACE_EXISTING);
}

static void OpenLog()
{
    if (s_log.is_open() || !g_hDLL)
        return;
    char dllPath[MAX_PATH] = {};
    if (GetModuleFileNameA(g_hDLL, dllPath, MAX_PATH))
    {
        char* lastSlash = strrchr(dllPath, '\\');
        if (lastSlash)
            *(lastSlash + 1) = '\0';
        std::string path = std::string(dllPath) + "F117_IRADS.log";
        RotateLogIfLarge(dllPath, path);
        s_log.open(path.c_str(), true);
    }
    if (s_log.is_open())
    {
        s_log.write("\n--- F117_IRADS log opened ---\n");
        s_log.write("[F117-IRADS] BUILD %s\n", kIRADSBuildMarker);
    }
}

void Log_(const char* msg)
{
    OpenLog();
    if (s_log.is_open() && s_log_load_bytes < kLogLoadBytes)
    {
        const int written = s_log.write("[F117-IRADS] %s\n", msg);
        if (written > 0)
            s_log_load_bytes += static_cast<unsigned long long>(written);
        if (s_log_load_bytes >= kLogLoadBytes)
            s_log.write("[F117-IRADS] Log limit reached; file output stops until the DLL next loads\n");
    }
    OutputDebugStringA("[F117-IRADS] ");
    OutputDebugStringA(msg);
    OutputDebugStringA("\n");
}

void LogV(const char* fmt, va_list args)
{
    char buf[kLogLineBytes];
    format_into(buf, sizeof(buf), fmt, args);
    Log_(buf);
}

void Log(const char* fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    LogV(fmt, va);
    va_end(va);
}
}  // namespace CockpitInterop
