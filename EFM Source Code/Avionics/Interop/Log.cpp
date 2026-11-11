// IRADS interop log: F117_IRADS.log beside the DLL, bounded in size.
#include "stdafx.h"
#include "IradsInternal.h"
#include <cstdarg>
#include <cstdio>
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

    static FILE* s_log = nullptr;
    static unsigned long long s_log_load_bytes = 0;

    static void RotateLogIfLarge(const std::string& directory, const std::string& path)
    {
        WIN32_FILE_ATTRIBUTE_DATA info = {};
        if (!GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &info)) return;
        const unsigned long long size =
            (static_cast<unsigned long long>(info.nFileSizeHigh) << 32) | info.nFileSizeLow;
        if (size <= kLogRotateBytes) return;
        // On failure the log keeps appending to the same file, and the next load tries again.
        MoveFileExA(path.c_str(), (directory + "F117_IRADS.old.log").c_str(), MOVEFILE_REPLACE_EXISTING);
    }

    static void OpenLog()
    {
        if (s_log || !g_hDLL) return;
        char dllPath[MAX_PATH] = {};
        if (GetModuleFileNameA(g_hDLL, dllPath, MAX_PATH))
        {
            char* lastSlash = strrchr(dllPath, '\\');
            if (lastSlash) *(lastSlash + 1) = '\0';
            std::string path = std::string(dllPath) + "F117_IRADS.log";
            RotateLogIfLarge(dllPath, path);
            s_log = fopen(path.c_str(), "a");
        }
        if (s_log)
        {
            fprintf(s_log, "\n--- F117_IRADS log opened ---\n");
            fprintf(s_log, "[F117-IRADS] BUILD %s\n", kIRADSBuildMarker);
            fflush(s_log);
        }
    }

    void Log_(const char* msg)
    {
        OpenLog();
        if (s_log && s_log_load_bytes < kLogLoadBytes)
        {
            const int written = fprintf(s_log, "[F117-IRADS] %s\n", msg);
            if (written > 0) s_log_load_bytes += static_cast<unsigned long long>(written);
            if (s_log_load_bytes >= kLogLoadBytes)
                fprintf(s_log, "[F117-IRADS] Log limit reached; file output stops until the DLL next loads\n");
            fflush(s_log);
        }
        OutputDebugStringA("[F117-IRADS] ");
        OutputDebugStringA(msg);
        OutputDebugStringA("\n");
    }

    void Log(const char* fmt, ...)
    {
        char buf[kLogLineBytes];
        va_list va;
        va_start(va, fmt);
        vsnprintf(buf, sizeof(buf), fmt, va);
        va_end(va);
        Log_(buf);
    }
} // namespace CockpitInterop
