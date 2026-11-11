#include "stdafx.h"
#include "LogFile.h"

#include <cstdio>

Log_file::~Log_file()
{
    close();
}

bool Log_file::open(const char* path, bool append)
{
    close();
    m_file = std::fopen(path, append ? "a" : "w");
    return m_file != nullptr;
}

bool Log_file::is_open() const
{
    return m_file != nullptr;
}

void Log_file::close()
{
    if (m_file != nullptr)
    {
        std::fclose(static_cast<std::FILE*>(m_file));
        m_file = nullptr;
    }
}

int Log_file::write(const char* format, ...)
{
    if (m_file == nullptr)
    {
        return -1;
    }

    std::FILE* file = static_cast<std::FILE*>(m_file);
    va_list args;
    va_start(args, format);
    const int written = std::vfprintf(file, format, args);
    va_end(args);
    std::fflush(file);
    return written;
}

int format_into(char* buffer, std::size_t size, const char* format, va_list args)
{
    return std::vsnprintf(buffer, size, format, args);
}
