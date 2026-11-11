#pragma once
// Printf-style text log. This and LogFile.cpp are the only places that use C stdio (JSF AV 22 deviation,
// see docs/JSF-deviations.md); everything else logs through here.
#include <cstdarg>
#include <cstddef>

class Log_file
{
public:
    Log_file() = default;
    Log_file(const Log_file&) = delete;
    Log_file& operator=(const Log_file&) = delete;
    ~Log_file();

    // Opens `path` for appending or truncating. Returns false when the file cannot be opened.
    bool open(const char* path, bool append);
    bool is_open() const;
    void close();

    // Formats, writes and flushes one message. Returns the number of characters written, or a negative
    // value when the log is closed or the write failed.
    int write(const char* format, ...);

private:
    void* m_file = nullptr;  // FILE*, opaque so callers do not need <cstdio>
};

// Formats into `buffer` (truncating, always terminated). Returns the length the full message needed.
int format_into(char* buffer, std::size_t size, const char* format, va_list args);
