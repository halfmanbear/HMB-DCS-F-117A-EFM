#pragma once

#include <cstddef>
#include <cstdint>

// Raw views kept only where DCS exports no accessor. Everything else goes through exported functions.
namespace Cockpit_interop
{
// Standard library vector layout: first, last, end of storage.
struct Pointer_vector_view
{
    void** begin;
    void** end;
    void** cap;
};
static_assert(sizeof(Pointer_vector_view) == 0x18);

}  // namespace Cockpit_interop
