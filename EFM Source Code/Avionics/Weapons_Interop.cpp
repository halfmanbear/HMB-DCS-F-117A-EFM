// ============================================================================
// Weapons_Interop.cpp — IRADS target hand-off to DCS weapons
// ============================================================================
// Store launch and laser spot calls go through exported DCS functions and the
// weapon manager interface. The interface positions are read at start-up from
// DCS's own exported weapon functions, so they follow DCS updates.
// ============================================================================

#include "stdafx.h"
#include "Weapons_Interop.h"
#include "CockpitBase_Interop.h"
#include <cstdint>
#include <cstdarg>
#include <cstring>
#include <cstdio>
#include <cmath>

namespace Weapons_interop
{
namespace
{
struct Vec3d
{
    double x;
    double y;
    double z;
};
struct Vec3f
{
    float x;
    float y;
    float z;
};

constexpr size_t suicide_scan_bytes = 0x400;

using Pfn_get_manager = void*(__cdecl*)();
using Pfn_select_station = void(__fastcall*)(void* self, int station);
using Pfn_weapon_count = int(__fastcall*)(void* self, int station);
using Pfn_current_station = int(__fastcall*)(void* self);
using Pfn_station_count = int(__fastcall*)(void* self);
using Pfn_launch = int(__fastcall*)(void* self, int station, int substation, unsigned int target_id, const Vec3f* point,
                                    bool armed);
using Pfn_create_laser_spot = void*(__cdecl*)(unsigned int owner_id, const Vec3d* local_pos, const Vec3d* aim_point,
                                              unsigned int code);
using Pfn_set_point = void(__fastcall*)(void* spot, const Vec3d* point);
using Pfn_register_to_kill = void(__cdecl*)(void* suicide);
using Pfn_id_of_human = unsigned int(__fastcall*)(void* world_manager);

bool s_resolved = false;
bool s_laser_disabled = false;
bool s_disabled = false;  // set after a fault; stops further native calls
Pfn_create_laser_spot s_create_laser_spot = nullptr;
Pfn_set_point s_set_point = nullptr;
Pfn_register_to_kill s_register_to_kill = nullptr;
const void* s_spot_suicide_vtbl = nullptr;
// The laser spot owner is the player aircraft, identified through DCS's world manager.
void* const* s_world_manager = nullptr;
Pfn_id_of_human s_id_of_human = nullptr;
// Returns the player aircraft's weapon manager (its human payload interface).
Pfn_get_manager s_get_payload = nullptr;
bool s_payload_resolved = false;

// Weapon manager interface table positions (entry indices).
struct Manager_slots
{
    size_t select_station;
    size_t weapon_count;
    size_t current_station;
    size_t launch;
    size_t station_count;
};
Manager_slots s_slots = {};

void* s_spot = nullptr;
unsigned int s_spot_code = 0;
ptrdiff_t s_suicide_offset = -1;

void log_format(const char* fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    Cockpit_interop::log_vformat(fmt, va);
    va_end(va);
}

void log_message(const char* msg)
{
    Cockpit_interop::log_message(msg);
}

// The player's unit ID comes from the exported ID_OF_Human member. Its class is read from that
// export's decorated name, and the instance is the exported global of that class.
void resolve_player_id(HMODULE module)
{
    static constexpr char member[] = "?ID_OF_Human@";
    const char* name = nullptr;
    const auto id_of_human =
        reinterpret_cast<Pfn_id_of_human>(Cockpit_interop::unique_export(module, member, &name));
    if (!id_of_human || !name)
        return;
    const char* owner = name + sizeof(member) - 1;
    const char* end = strstr(owner, "@@");
    if (!end || end == owner || end - owner > 64)
        return;
    char global[96];
    snprintf(global, sizeof(global), "?global%.*s@@", static_cast<int>(end - owner), owner);
    const auto instance = reinterpret_cast<void* const*>(Cockpit_interop::unique_export(module, global));
    if (!instance)
        return;
    s_world_manager = instance;
    s_id_of_human = id_of_human;
}

void resolve()
{
    if (s_resolved)
        return;

    HMODULE h_wg = GetModuleHandleA("WorldGeneral.dll");
    HMODULE h_w = GetModuleHandleA("World.dll");
    if (h_wg)
    {
        s_create_laser_spot = reinterpret_cast<Pfn_create_laser_spot>(
            GetProcAddress(h_wg, "?create_laser_spot@woSpot@@SAPEAV1@IAEBVVec3d@osg@@0I@Z"));
        s_set_point =
            reinterpret_cast<Pfn_set_point>(GetProcAddress(h_wg, "?set_point@woSpot@@QEAAXAEBVVec3d@osg@@@Z"));
        s_spot_suicide_vtbl = reinterpret_cast<const void*>(GetProcAddress(h_wg, "??_7woSpot@@6BSuicide@@@"));
        resolve_player_id(h_wg);
    }
    if (h_w)
    {
        s_register_to_kill = reinterpret_cast<Pfn_register_to_kill>(
            GetProcAddress(h_w, "?registerToKill@wSimCalendar@@SAXPEAVSuicide@@@Z"));
    }
    s_resolved =
        s_create_laser_spot && s_set_point && s_register_to_kill && s_spot_suicide_vtbl && s_world_manager && s_id_of_human;
    log_format("WeaponsInterop: create_laser_spot=%s set_point=%s registerToKill=%s woSpot::Suicide table=%s "
               "player ID=%s",
               s_create_laser_spot ? "ok" : "missing", s_set_point ? "ok" : "missing",
               s_register_to_kill ? "ok" : "missing", s_spot_suicide_vtbl ? "ok" : "missing",
               s_world_manager && s_id_of_human ? "ok" : "missing");
}

// The live cockpit context; only tested for presence.
void* context()
{
    if (!Cockpit_interop::g_contexts_ptr_addr)
        return nullptr;
    return *Cockpit_interop::g_contexts_ptr_addr;
}

template <typename Fn>
Fn slot(void* object, size_t slot)
{
    return reinterpret_cast<Fn>((*reinterpret_cast<void***>(object))[slot]);
}

// SEH wrappers. Kept free of C++ objects so __try is allowed.
void* get_weapon_manager_raw()
{
    __try
    {
        return s_get_payload();
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        s_disabled = true;
        return nullptr;
    }
}

bool launch_station_raw(void* manager, int station, const Vec3f* point, int* out_result)
{
    __try
    {
        if (slot<Pfn_weapon_count>(manager, s_slots.weapon_count)(manager, station) <= 0)
        {
            *out_result = 0;
            return true;
        }
        slot<Pfn_select_station>(manager, s_slots.select_station)(manager, station);
        *out_result = slot<Pfn_launch>(manager, s_slots.launch)(manager, station, -1, 0u, point, true);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        s_disabled = true;
        return false;
    }
}

bool station_info_raw(void* manager, int* out_current, int* out_count)
{
    __try
    {
        *out_current = slot<Pfn_current_station>(manager, s_slots.current_station)(manager);
        *out_count = slot<Pfn_station_count>(manager, s_slots.station_count)(manager);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        s_disabled = true;
        return false;
    }
}

unsigned int player_id_raw()
{
    __try
    {
        void* manager = *s_world_manager;
        return manager ? s_id_of_human(manager) : 0u;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return 0u;
    }
}

void* create_spot_raw(unsigned int owner_id, const Vec3d* local, const Vec3d* aim, unsigned int code)
{
    __try
    {
        return s_create_laser_spot(owner_id, local, aim, code);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        s_laser_disabled = true;
        return nullptr;
    }
}

bool set_point_raw(void* spot, const Vec3d* aim)
{
    __try
    {
        s_set_point(spot, aim);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

bool kill_raw(void* suicide)
{
    __try
    {
        s_register_to_kill(suicide);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// Stock time-of-flight predictor, exported by the weapons library as a small handle object. Resolved lazily.
using Pfn_tof_ctor = void*(__fastcall*)(void* self);
using Pfn_gbu_tof = double(__fastcall*)(void* self, double a, double b, double c, double d, double e);
bool s_tof_resolved = false;
Pfn_tof_ctor s_tof_ctor = nullptr;
Pfn_gbu_tof s_gbu_tof = nullptr;

bool resolve_tof()
{
    if (s_tof_resolved)
        return s_tof_ctor && s_gbu_tof;
    s_tof_resolved = true;
    HMODULE h = GetModuleHandleA("WeaponsBase.dll");
    if (h)
    {
        s_tof_ctor = reinterpret_cast<Pfn_tof_ctor>(GetProcAddress(h, "??0TOFEstimator@@QEAA@XZ"));
        s_gbu_tof =
            reinterpret_cast<Pfn_gbu_tof>(GetProcAddress(h, "?getGBU24TOF@TOFEstimator@@QEAANNNNNN@Z"));
    }
    log_format("WeaponsInterop: tof predictor ctor=%s call=%s", s_tof_ctor ? "ok" : "missing",
               s_gbu_tof ? "ok" : "missing");
    return s_tof_ctor && s_gbu_tof;
}

bool call_gbu_tof_raw(double a, double b, double c, double d, double e, double* out)
{
    alignas(16) unsigned char handle[64] = {};
    __try
    {
        s_tof_ctor(handle);
        *out = s_gbu_tof(handle, a, b, c, d, e);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        s_tof_ctor = nullptr;  // stop calling after a fault
        return false;
    }
}

// Stock GPS-bomb time-of-flight predictor, a larger state object with its own constructor. Resolved lazily.
using Pfn_jdam_tof = double(__fastcall*)(void* self, double alt, double speed, double pitch, double bank_a,
                                         double bank_b, double azimuth, double range, double target_alt,
                                         double override_a, double override_b, int type);
bool s_jdam_resolved = false;
Pfn_tof_ctor s_jdam_ctor = nullptr;
Pfn_tof_ctor s_jdam_dtor = nullptr;
Pfn_jdam_tof s_jdam_tof = nullptr;

bool resolve_jdam_tof()
{
    if (s_jdam_resolved)
        return s_jdam_ctor && s_jdam_tof;
    s_jdam_resolved = true;
    HMODULE h = GetModuleHandleA("WeaponsBase.dll");
    if (h)
    {
        s_jdam_ctor = reinterpret_cast<Pfn_tof_ctor>(GetProcAddress(h, "??0LARBuilder@@QEAA@XZ"));
        s_jdam_dtor = reinterpret_cast<Pfn_tof_ctor>(GetProcAddress(h, "??1LARBuilder@@QEAA@XZ"));
        s_jdam_tof = reinterpret_cast<Pfn_jdam_tof>(
            GetProcAddress(h, "?getJDAMTOF@LARBuilder@@QEAANNNNNNNNNNNW4JDAM_Type@1@@Z"));
    }
    log_format("WeaponsInterop: jdam tof predictor ctor=%s call=%s", s_jdam_ctor ? "ok" : "missing",
               s_jdam_tof ? "ok" : "missing");
    return s_jdam_ctor && s_jdam_tof;
}

bool call_jdam_tof_raw(int type, double alt, double speed, double pitch, double range, double target_alt, double* out)
{
    alignas(16) unsigned char state[1024] = {};
    __try
    {
        s_jdam_ctor(state);
        // The predictor refines its release solution over repeated calls; a fresh object answers 0 at first.
        // Repeat until two consecutive answers agree.
        double previous = 0.0;
        *out = 0.0;
        for (int pass = 0; pass < 60; ++pass)
        {
            *out = s_jdam_tof(state, alt, speed, pitch, 0.0, 0.0, 0.0, range, target_alt, -1.0, -1.0, type);
            if (*out > 0.0 && std::fabs(*out - previous) < 0.01)
                break;
            previous = *out;
        }
        if (s_jdam_dtor)
            s_jdam_dtor(state);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        s_jdam_ctor = nullptr;  // stop calling after a fault
        return false;
    }
}

// -- Weapon manager interface positions ---------------------------------------
// DCS exports no functions on the weapon manager interface. Several exported DCS weapon
// functions call it, so the positions are read from those calls and accepted only when
// every function that shares a position agrees. Any mismatch leaves store launch off.

constexpr size_t max_interface_calls = 16;
constexpr std::uint32_t max_interface_offset = 0x2000;

std::uint32_t read_u32(const std::uint8_t* p)
{
    std::uint32_t v;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

// Operand form [register + displacement] without an index register; returns the displacement size.
size_t displacement_size(std::uint8_t operand)
{
    if ((operand & 7) == 4)
        return 0;
    switch (operand & 0xC0)
    {
    case 0x40:
        return 1;
    case 0x80:
        return 4;
    default:
        return 0;
    }
}

std::uint32_t read_displacement(const std::uint8_t* p, size_t size)
{
    return size == 1 ? static_cast<std::uint32_t>(static_cast<std::int8_t>(p[0])) : read_u32(p);
}

bool is_prefix(std::uint8_t b)
{
    return (b & 0xF0) == 0x40;
}

bool plausible_offset(std::uint32_t offset)
{
    return offset > 0 && offset < max_interface_offset && offset % sizeof(void*) == 0;
}

// Table offsets of the calls one function makes through an object's function table, in code
// order. Covers a call or jump through [register + offset], and a load of [register + offset]
// into a register that is then called or jumped to.
size_t interface_calls(const void* function, size_t length, std::uint32_t* out)
{
    const auto* code = static_cast<const std::uint8_t*>(function);
    if (code[0] == 0xE9)  // follow an entry jump
        code += 5 + static_cast<std::int32_t>(read_u32(code + 1));
    size_t count = 0;
    for (size_t i = 0; i + 3 <= length && count < max_interface_calls; ++i)
    {
        const std::uint8_t* p = code + i;
        // Call or jump through memory, with an optional register-extension prefix.
        const size_t prefix = is_prefix(p[0]) ? 1u : 0u;
        if (p[prefix] == 0xFF)
        {
            const std::uint8_t operand = p[prefix + 1];
            const int kind = (operand >> 3) & 7;  // 2 = call, 4 = jump
            const size_t size = displacement_size(operand);
            if ((kind == 2 || kind == 4) && size && i + prefix + 2 + size <= length)
            {
                const std::uint32_t offset = read_displacement(p + prefix + 2, size);
                if (plausible_offset(offset))
                {
                    out[count++] = offset;
                    i += prefix + 1 + size;
                    continue;
                }
            }
        }
        // 64-bit register load followed by a call or jump through that register.
        if ((p[0] & 0xF8) == 0x48 && p[1] == 0x8B)
        {
            const size_t size = displacement_size(p[2]);
            if (!size || i + 3 + size > length)
                continue;
            const std::uint32_t offset = read_displacement(p + 3, size);
            const int reg = ((p[0] & 4) << 1) | ((p[2] >> 3) & 7);
            const size_t load_end = i + 3 + size;
            for (size_t j = load_end; j + 2 <= length && j < load_end + 24; ++j)
            {
                if (is_prefix(code[j - 1]))  // checked at the prefix position instead
                    continue;
                const bool prefixed = is_prefix(code[j]);
                if (prefixed && j + 3 > length)
                    break;
                if ((prefixed && (code[j] & 1)) != (reg >= 8))
                    continue;
                const std::uint8_t* op = prefixed ? code + j + 1 : code + j;
                if (op[0] == 0xFF && (op[1] == (0xD0 | (reg & 7)) || op[1] == (0xE0 | (reg & 7))))
                {
                    if (plausible_offset(offset))
                        out[count++] = offset;
                    break;
                }
            }
            i += 2 + size;
        }
    }
    return count;
}

struct Call_list
{
    std::uint32_t at[max_interface_calls];
    size_t count;
    bool has(size_t index, std::uint32_t offset) const { return index < count && at[index] == offset; }
};

bool read_calls(HMODULE module, const char* export_name, size_t length, Call_list& list)
{
    list.count = 0;
    const FARPROC function = GetProcAddress(module, export_name);
    if (!function)
    {
        log_format("WeaponsInterop: %s missing", export_name);
        return false;
    }
    list.count = interface_calls(reinterpret_cast<const void*>(function), length, list.at);
    return list.count > 0;
}

bool discover_manager_slots(HMODULE cockpit_base, Manager_slots& slots)
{
    Call_list payload, launch_weapon, station_selected, weapon_count, missile_launch, launch, launch_station;
    if (!read_calls(cockpit_base, "?c_payload@cockpit@@YAPEAVIwHumanPayload@@XZ", 0x20, payload) ||
        !read_calls(cockpit_base, "?launch_weapon@avWeaponsCommon@cockpit@@SAX_N@Z", 0x40, launch_weapon) ||
        !read_calls(cockpit_base, "?is_station_selected@avArcadeRadar@cockpit@@UEBA_NH@Z", 0x60, station_selected) ||
        !read_calls(cockpit_base, "?get_weapon_count@avSimpleWeaponSystem@cockpit@@QEAAHH@Z", 0x58, weapon_count) ||
        !read_calls(cockpit_base, "?launch@eqSidewinder@cockpit@@QEAAIPEAVIwHumanPlane@@@Z", 0x70, missile_launch) ||
        !read_calls(cockpit_base, "?Launch@avSimpleWeaponSystem@cockpit@@EEAAX_N@Z", 0xE0, launch) ||
        !read_calls(cockpit_base, "?launch_station@avSimpleWeaponSystem@cockpit@@AEAAXHH@Z", 0x180, launch_station))
        return false;

    // The aircraft's weapon manager getter, as c_payload() calls it.
    const std::uint32_t getter = payload.at[0];
    // launch_weapon() first asks the manager for the current station.
    const std::uint32_t current = launch_weapon.at[0];
    // is_station_selected() asks for the current station, then queries the station twice.
    const std::uint32_t select =
        station_selected.count > 2 && station_selected.at[1] == station_selected.at[2] ? station_selected.at[1] : 0;
    // get_weapon_count() fetches the manager twice, then asks for the station's weapon count.
    const std::uint32_t count = weapon_count.has(0, getter) && weapon_count.has(1, getter) && weapon_count.count > 2
                                    ? weapon_count.at[2]
                                    : 0;
    // The missile launch fetches the manager, then calls the launch with a target point.
    const std::uint32_t fire = missile_launch.has(0, getter) && missile_launch.count > 1 ? missile_launch.at[1] : 0;
    // The weapon system's Launch fetches the manager, asks for the current station, then loops
    // over the station count, asking for it again on every pass.
    std::uint32_t stations = 0;
    if (launch.has(0, getter) && launch.has(1, current) && launch.count > 2)
        for (size_t i = 3; i < launch.count; ++i)
            if (launch.at[i] == launch.at[2])
                stations = launch.at[2];

    bool agrees = station_selected.has(0, current) && select && count && fire && stations;
    // launch_station() selects the station, checks its weapon count, then launches, in that order.
    size_t next = 0;
    const std::uint32_t sequence[] = {select, count, fire};
    for (size_t i = 0; i < launch_station.count && next < 3; ++i)
        if (launch_station.at[i] == sequence[next])
            ++next;
    agrees = agrees && next == 3;
    const std::uint32_t all[] = {getter, current, select, count, fire, stations};
    for (size_t a = 0; a < 6 && agrees; ++a)
        for (size_t b = a + 1; b < 6; ++b)
            if (all[a] == all[b])
                agrees = false;
    if (!agrees)
    {
        log_format("WeaponsInterop: weapon manager positions not confirmed (current=%u select=%u count=%u launch=%u "
                   "stations=%u sequence=%zu); store launch disabled",
                   current, select, count, fire, stations, next);
        return false;
    }
    slots = {select / sizeof(void*), count / sizeof(void*), current / sizeof(void*), fire / sizeof(void*),
             stations / sizeof(void*)};
    log_format("WeaponsInterop: weapon manager positions confirmed (select=%zu count=%zu current=%zu launch=%zu "
               "stations=%zu)",
               slots.select_station, slots.weapon_count, slots.current_station, slots.launch, slots.station_count);
    return true;
}

void* weapon_manager()
{
    if (s_disabled)
        return nullptr;
    if (!s_payload_resolved)
    {
        const HMODULE cockpit_base = GetModuleHandleA("CockpitBase.dll");
        if (!cockpit_base)
            return nullptr;
        s_payload_resolved = true;
        if (discover_manager_slots(cockpit_base, s_slots))
            s_get_payload = reinterpret_cast<Pfn_get_manager>(
                GetProcAddress(cockpit_base, "?c_payload@cockpit@@YAPEAVIwHumanPayload@@XZ"));
        log_format("WeaponsInterop: c_payload=%s", s_get_payload ? "ok" : "missing");
    }
    if (!s_get_payload || !context())
        return nullptr;
    return get_weapon_manager_raw();
}

// This position differs between DCS builds; find it by locating the
// exported table inside a live spot.
ptrdiff_t find_suicide_offset(void* spot)
{
    if (!s_spot_suicide_vtbl)
        return -1;
    const uintptr_t target = reinterpret_cast<uintptr_t>(s_spot_suicide_vtbl);
    const auto* words = reinterpret_cast<const uintptr_t*>(spot);
    for (size_t i = 1; i < suicide_scan_bytes / sizeof(uintptr_t); ++i)
    {
        if (words[i] == target)
            return static_cast<ptrdiff_t>(i * sizeof(uintptr_t));
    }
    return -1;
}

void kill_spot()
{
    if (!s_spot)
        return;
    if (s_register_to_kill && s_suicide_offset > 0)
    {
        if (!kill_raw(reinterpret_cast<char*>(s_spot) + s_suicide_offset))
            log_message("WeaponsInterop: registerToKill faulted; dropping laser spot pointer");
    }
    else
    {
        log_message("WeaponsInterop: cannot remove laser spot (kill path unresolved); dropping pointer");
    }
    s_spot = nullptr;
}
}  // namespace

bool is_launch_available()
{
    return weapon_manager() != nullptr;
}

bool launch_at_point(const double world_point[3], int station)
{
    if (!world_point || station < 0 || station > 8)
        return false;
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(world_point[i]) || std::abs(world_point[i]) > 1.0e8)
            return false;
    void* manager = weapon_manager();
    if (!manager)
        return false;
    int current = -1;
    int count = 0;
    if (!station_info_raw(manager, &current, &count) || station >= count)
        return false;
    const Vec3f point{static_cast<float>(world_point[0]), static_cast<float>(world_point[1]),
                      static_cast<float>(world_point[2])};
    int result = 0;
    if (!launch_station_raw(manager, station, &point, &result))
    {
        log_format("WeaponsInterop: launch fault station=%d; disabled", station);
        return false;
    }
    log_format("WeaponsInterop: launch station=%d target=(%.2f,%.2f,%.2f) result=%d", station, point.x, point.y,
               point.z, result);
    return result != 0;  // matches native avSimpleWeaponSystem acceptance test
}

bool stock_laser_bomb_tof(double release_alt_m, double release_speed_mps, double pitch_rad, double target_alt_m,
                          double ground_range_m, double* out)
{
    if (!out || !resolve_tof())
        return false;
    return call_gbu_tof_raw(release_alt_m, release_speed_mps, pitch_rad, target_alt_m, ground_range_m, out);
}

bool stock_jdam_tof(int type, double release_alt_m, double release_speed_mps, double pitch_rad, double target_alt_m,
                    double ground_range_m, double* out)
{
    if (!out || !resolve_jdam_tof())
        return false;
    return call_jdam_tof_raw(type, release_alt_m, release_speed_mps, pitch_rad, ground_range_m, target_alt_m, out);
}

void update_laser(bool want_on, const double world_point[3], unsigned int code)
{
    resolve();

    if (!want_on || s_laser_disabled || !world_point)
    {
        kill_spot();
        return;
    }

    // Changing code on a live spot is build-specific; recreate instead.
    if (s_spot && code != s_spot_code)
        kill_spot();

    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(world_point[i]))
        {
            kill_spot();
            return;
        }
    const Vec3d aim{world_point[0], world_point[1], world_point[2]};
    if (s_spot)
    {
        if (!set_point_raw(s_spot, &aim))
        {
            log_message("WeaponsInterop: set_point faulted; laser spot dropped");
            kill_spot();
            s_laser_disabled = true;
        }
        return;
    }

    if (!s_resolved || !context())
        return;

    const unsigned int owner_id = player_id_raw();
    if (owner_id == 0)
        return;
    const Vec3d mount{Cockpit_interop::irads_pos_x, Cockpit_interop::irads_pos_y, Cockpit_interop::irads_pos_z};

    s_spot = create_spot_raw(owner_id, &mount, &aim, code);
    s_spot_code = code;
    if (!s_spot)
    {
        log_message("WeaponsInterop: create_laser_spot failed; laser disabled");
        return;
    }
    if (s_suicide_offset < 0)
        s_suicide_offset = find_suicide_offset(s_spot);
    log_format("WeaponsInterop: laser spot created owner=%u code=%u expiry link=%s", owner_id, code,
               s_suicide_offset >= 0 ? "ok" : "missing");
}

bool is_laser_firing()
{
    return s_spot != nullptr;
}

void reset()
{
    kill_spot();
    s_disabled = false;
    s_laser_disabled = false;
    s_suicide_offset = -1;
}
}  // namespace Weapons_interop
