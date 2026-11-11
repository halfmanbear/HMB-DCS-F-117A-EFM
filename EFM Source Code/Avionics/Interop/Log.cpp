// IRADS interop log: F117_IRADS.log beside the DLL, bounded in size.
#include "stdafx.h"
#include "IradsInternal.h"
#include "../../Diagnostics/LogFile.h"
#include <cstdarg>
#include <cstring>
#include <string>

namespace Cockpit_interop
{
// Keeps the log bounded on disk. When the DLL loads, a log over kLogRotateBytes becomes
// F117_IRADS.old.log, replacing the previous one. One load stops writing to the file after
// kLogLoadBytes, so a runaway message can't fill the disk or push out the older log.
static constexpr unsigned long long log_rotate_bytes = 10ULL * 1024 * 1024;
static constexpr unsigned long long log_load_bytes = 20ULL * 1024 * 1024;
static constexpr size_t log_line_bytes = 512;

static Log_file s_log;
static unsigned long long s_log_load_bytes = 0;

static void rotate_log_if_large(const std::string& directory, const std::string& path)
{
    WIN32_FILE_ATTRIBUTE_DATA info = {};
    if (!GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &info))
        return;
    const unsigned long long size = (static_cast<unsigned long long>(info.nFileSizeHigh) << 32) | info.nFileSizeLow;
    if (size <= log_rotate_bytes)
        return;
    // On failure the log keeps appending to the same file, and the next load tries again.
    MoveFileExA(path.c_str(), (directory + "F117_IRADS.old.log").c_str(), MOVEFILE_REPLACE_EXISTING);
}

static void open_log()
{
    if (s_log.is_open() || !g_h_dll)
        return;
    char dll_path[MAX_PATH] = {};
    if (GetModuleFileNameA(g_h_dll, dll_path, MAX_PATH))
    {
        char* last_slash = strrchr(dll_path, '\\');
        if (last_slash)
            *(last_slash + 1) = '\0';
        std::string path = std::string(dll_path) + "F117_IRADS.log";
        rotate_log_if_large(dll_path, path);
        s_log.open(path.c_str(), true);
    }
    if (s_log.is_open())
    {
        s_log.write("\n--- F117_IRADS log opened ---\n");
        s_log.write("[F117-IRADS] BUILD %s\n", irads_build_marker);
    }
}

void log_message(const char* msg)
{
    open_log();
    if (s_log.is_open() && s_log_load_bytes < log_load_bytes)
    {
        const int written = s_log.write("[F117-IRADS] %s\n", msg);
        if (written > 0)
            s_log_load_bytes += static_cast<unsigned long long>(written);
        if (s_log_load_bytes >= log_load_bytes)
            s_log.write("[F117-IRADS] Log limit reached; file output stops until the DLL next loads\n");
    }
    OutputDebugStringA("[F117-IRADS] ");
    OutputDebugStringA(msg);
    OutputDebugStringA("\n");
}

void log_vformat(const char* fmt, va_list args)
{
    char buf[log_line_bytes];
    format_into(buf, sizeof(buf), fmt, args);
    log_message(buf);
}

void log_format(const char* fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    log_vformat(fmt, va);
    va_end(va);
}
}  // namespace Cockpit_interop
