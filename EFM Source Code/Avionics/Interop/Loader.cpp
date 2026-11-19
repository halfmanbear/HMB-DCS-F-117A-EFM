// Resolves the avTVSensor entry points and the cockpit context table from CockpitBase.dll.
// avTVSensor is NOT an avDevice: never register/start it through devices_keeper.
#include "stdafx.h"
#include "IradsInternal.h"

namespace Cockpit_interop
{
HMODULE g_h_dll = nullptr;
Pfn_av_tv_sensor_ctor g_av_tv_sensor_ctor = nullptr;
Pfn_av_tv_sensor_initialize g_av_tv_sensor_initialize = nullptr;
Pfn_av_tv_sensor_init_zoom g_av_tv_sensor_init_zoom = nullptr;
Pfn_av_tv_sensor_update g_av_tv_sensor_update = nullptr;
Pfn_av_tv_sensor_update_frame g_av_tv_sensor_update_frame = nullptr;
Pfn_av_tv_sensor_slew_axis g_av_tv_sensor_slew_left = nullptr;
Pfn_av_tv_sensor_slew_axis g_av_tv_sensor_slew_right = nullptr;
Pfn_av_tv_sensor_slew_axis g_av_tv_sensor_slew_up = nullptr;
Pfn_av_tv_sensor_slew_axis g_av_tv_sensor_slew_down = nullptr;
Pfn_av_tv_sensor_abs_axis g_av_tv_sensor_move_horizontal_abs = nullptr;
Pfn_av_tv_sensor_abs_axis g_av_tv_sensor_move_vertical_abs = nullptr;
Pfn_av_tv_sensor_set_external_designation g_av_tv_sensor_set_external_designation = nullptr;
Pfn_av_tv_sensor_stabilize_on_ground g_av_tv_sensor_stabilize_on_ground = nullptr;
Pfn_av_tv_sensor_void_method g_av_tv_sensor_search = nullptr;
Pfn_av_tv_sensor_void_method g_av_tv_sensor_uncage = nullptr;
Pfn_av_tv_sensor_void_method g_av_tv_sensor_slew_stop = nullptr;
Pfn_av_tv_sensor_void_method g_av_tv_sensor_zoom_in = nullptr;
Pfn_av_tv_sensor_void_method g_av_tv_sensor_zoom_out = nullptr;
Pfn_binding_context g_binding_context = nullptr;
Pfn_platform_pose g_platform_position = nullptr;
Pfn_platform_pose g_platform_initial_position = nullptr;
Pfn_platform_update g_platform_update_position = nullptr;
Pfn_sensor_flag g_av_tv_sensor_caged = nullptr;
Pfn_sensor_value g_av_tv_sensor_zoom = nullptr;
Pfn_sensor_value g_av_tv_sensor_default_fov = nullptr;
Pfn_sensor_value g_av_tv_sensor_view_angle = nullptr;
Pfn_sensor_point g_av_tv_sensor_reference_point = nullptr;
Pfn_motor_value g_motor_get = nullptr;
Pfn_motor_value g_motor_get_target = nullptr;
Pfn_motor_force g_motor_force = nullptr;
Pfn_motor_stop g_motor_stop = nullptr;
void** g_contexts_ptr_addr = nullptr;
void* g_av_tv_sensor_instance = nullptr;

static bool s_initialized = false;

// g_hDLL is our EFM module (used for the log path), not CockpitBase.
FARPROC resolve_cockpit_base_export(const char* name)
{
    const HMODULE module = GetModuleHandleA("CockpitBase.dll");
    return module ? GetProcAddress(module, name) : nullptr;
}

FARPROC unique_export(HMODULE module, const char* prefix, const char** name)
{
    if (!module || !prefix)
        return nullptr;
    const auto* base = reinterpret_cast<const unsigned char*>(module);
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    if (dos->e_magic != IMAGE_DOS_SIGNATURE)
        return nullptr;
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
    const IMAGE_DATA_DIRECTORY& entry = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_EXPORT];
    if (nt->Signature != IMAGE_NT_SIGNATURE || !entry.VirtualAddress)
        return nullptr;
    const auto* exports = reinterpret_cast<const IMAGE_EXPORT_DIRECTORY*>(base + entry.VirtualAddress);
    const auto* names = reinterpret_cast<const DWORD*>(base + exports->AddressOfNames);
    const size_t length = strlen(prefix);
    const char* found = nullptr;
    for (DWORD i = 0; i < exports->NumberOfNames; ++i)
    {
        const char* candidate = reinterpret_cast<const char*>(base + names[i]);
        if (strncmp(candidate, prefix, length) != 0)
            continue;
        if (found)
            return nullptr;  // an overload: refuse to guess
        found = candidate;
    }
    if (found && name)
        *name = found;
    return found ? GetProcAddress(module, found) : nullptr;
}

void* get_global_context()
{
    return g_contexts_ptr_addr ? *g_contexts_ptr_addr : nullptr;
}

void* get_binding_context()
{
    return g_binding_context ? g_binding_context() : nullptr;
}

// An exported function's code, following an initial jump.
static const unsigned char* function_code(const void* function)
{
    auto* code = static_cast<const unsigned char*>(function);
    if (code && code[0] == 0xE9)
    {
        std::int32_t jump;
        memcpy(&jump, code + 1, 4);
        code += 5 + jump;
    }
    return code;
}

static const unsigned char* exported_code(HMODULE module, const char* prefix)
{
    return function_code(reinterpret_cast<const void*>(unique_export(module, prefix)));
}

static std::uint32_t code_u32(const unsigned char* code)
{
    std::uint32_t value;
    memcpy(&value, code, 4);
    return value;
}

// A [base+disp32] memory operand.
static bool memory_operand(unsigned char modrm)
{
    return (modrm & 0xC0) == 0x80 && (modrm & 7) != 4;
}

// ccIndicator::get_screen_condition ends by returning the field ("mov eax,[base+disp32]; ...;
// ret"). ccKneeboard::get_screen_condition stores update_camera's result in the same field
// ("call update_camera; mov [base+disp32],eax"). Both must name the same position.
size_t screen_condition_offset()
{
    static size_t s_offset = SIZE_MAX;
    if (s_offset != SIZE_MAX)
        return s_offset;
    const HMODULE module = GetModuleHandleA("CockpitBase.dll");
    if (!module)
        return 0;  // not loaded yet: look again later
    s_offset = 0;
    const unsigned char* getter = exported_code(module, "?get_screen_condition@ccIndicator@cockpit@@");
    const unsigned char* kneeboard = exported_code(module, "?get_screen_condition@ccKneeboard@cockpit@@");
    const auto* update_entry =
        reinterpret_cast<const unsigned char*>(unique_export(module, "?update_camera@ccKneeboard@cockpit@@"));
    const unsigned char* update = function_code(update_entry);
    if (!getter || !kneeboard || !update)
        return 0;
    std::uint32_t read = 0;
    for (size_t i = 0; i + 6 <= 0x100 && !read; ++i)
    {
        if (getter[i] != 0x8B || !memory_operand(getter[i + 1]) || (getter[i + 1] & 0x38) != 0 ||
            (i > 0 && (getter[i - 1] & 0xF0) == 0x40))  // eax only, not a REX-prefixed register
            continue;
        for (size_t j = i + 6; j < i + 16; ++j)
            if (getter[j] == 0xC3)
            {
                read = code_u32(getter + i + 2);
                break;
            }
    }
    std::uint32_t stored = 0;
    for (size_t i = 0; i + 11 <= 0x100 && !stored; ++i)
    {
        if (kneeboard[i] != 0xE8)
            continue;
        std::int32_t call;
        memcpy(&call, kneeboard + i + 1, 4);
        const unsigned char* target = kneeboard + i + 5 + call;  // compared only, never read
        if ((target == update || target == update_entry) &&
            kneeboard[i + 5] == 0x89 && memory_operand(kneeboard[i + 6]) && (kneeboard[i + 6] & 0x38) == 0)
            stored = code_u32(kneeboard + i + 7);
    }
    if (read && read == stored && read < 0x1000 && read % 4 == 0)
        s_offset = read;
    log_format("Initialize: screen condition position %s", s_offset ? "found" : "not confirmed");
    return s_offset;
}

// getCurrentZoom reads the zoom motor's value: the motor's position plus where avMotor::get reads.
// zoom_in and zoom_out each move "this" to the motor before calling it. All three must agree.
size_t zoom_motor_offset()
{
    static size_t s_offset = SIZE_MAX;
    if (s_offset != SIZE_MAX)
        return s_offset;
    const HMODULE module = GetModuleHandleA("CockpitBase.dll");
    if (!module)
        return 0;  // not loaded yet: look again later
    s_offset = 0;
    const unsigned char* zoom = exported_code(module, "?getCurrentZoom@avTVSensor@cockpit@@");
    const unsigned char* get = exported_code(module, "?get@avMotor@cockpit@@");
    const unsigned char* zoom_in = exported_code(module, "?zoom_in@avTVSensor@cockpit@@");
    const unsigned char* zoom_out = exported_code(module, "?zoom_out@avTVSensor@cockpit@@");
    if (!zoom || !get || !zoom_in || !zoom_out)
        return 0;
    // "movsd xmm0,[rcx+disp]" at the start of each getter.
    const bool zoom_read = zoom[0] == 0xF2 && zoom[1] == 0x0F && zoom[2] == 0x10 && zoom[3] == 0x81;
    const bool get_read = get[0] == 0xF2 && get[1] == 0x0F && get[2] == 0x10 && (get[3] == 0x41 || get[3] == 0x81);
    if (!zoom_read || !get_read)
        return 0;
    const std::uint32_t value = code_u32(zoom + 4);
    const std::uint32_t inside = get[3] == 0x41 ? get[4] : code_u32(get + 4);
    if (inside >= value)
        return 0;
    const std::uint32_t motor = value - inside;
    // "add rcx,imm32" before the motor call.
    const auto moves_to_motor = [motor](const unsigned char* code)
    {
        for (size_t i = 0; i + 7 <= 0x80; ++i)
            if (code[i] == 0x48 && code[i + 1] == 0x81 && code[i + 2] == 0xC1)
                return code_u32(code + i + 3) == motor;
        return false;
    };
    if (motor < 0x1000 && motor % 8 == 0 && moves_to_motor(zoom_in) && moves_to_motor(zoom_out))
        s_offset = motor;
    log_format("Initialize: zoom motor position %s", s_offset ? "found" : "not confirmed");
    return s_offset;
}

template <typename Fn>
static void load_export(HMODULE module, Fn& target, const char* name)
{
    target = reinterpret_cast<Fn>(GetProcAddress(module, name));
    log_format("Initialize: %-60s -> %s", name, target ? "ok" : "missing");
}

// Same, by "?method@class@namespace@@" prefix; the one matching export is used.
template <typename Fn>
static void load_member_export(HMODULE module, Fn& target, const char* prefix)
{
    target = reinterpret_cast<Fn>(unique_export(module, prefix));
    log_format("Initialize: %-60s -> %s", prefix, target ? "ok" : "missing");
}

// The exports the sensor bridge cannot work without.
static bool required_exports_present()
{
    return g_av_tv_sensor_ctor && g_av_tv_sensor_initialize && g_contexts_ptr_addr && g_av_tv_sensor_update &&
           g_av_tv_sensor_update_frame && g_av_tv_sensor_zoom_in && g_av_tv_sensor_zoom_out && g_av_tv_sensor_uncage &&
           g_av_tv_sensor_caged && g_platform_position && g_platform_initial_position && g_platform_update_position &&
           g_av_tv_sensor_zoom && g_av_tv_sensor_default_fov && g_av_tv_sensor_view_angle &&
           g_av_tv_sensor_reference_point && g_motor_get && g_motor_get_target && g_motor_force && g_motor_stop;
}

static void load_sensor_exports(HMODULE module)
{
    load_export(module, g_av_tv_sensor_ctor, "??0avTVSensor@cockpit@@QEAA@XZ");
    load_export(module, g_av_tv_sensor_initialize, "?initialize@avTVSensor@cockpit@@UEAAXXZ");
    load_export(module, g_av_tv_sensor_init_zoom, "?initZoom@avTVSensor@cockpit@@MEAAXXZ");
    load_export(module, g_av_tv_sensor_update, "?update@avTVSensor@cockpit@@UEAAXN@Z");
    load_export(module, g_av_tv_sensor_update_frame, "?update_frame@avTVSensor@cockpit@@MEAAXXZ");
    load_export(module, g_av_tv_sensor_slew_left, "?slew_left@avTVSensor@cockpit@@UEAAXN@Z");
    load_export(module, g_av_tv_sensor_slew_right, "?slew_right@avTVSensor@cockpit@@UEAAXN@Z");
    load_export(module, g_av_tv_sensor_slew_up, "?slew_up@avTVSensor@cockpit@@UEAAXN@Z");
    load_export(module, g_av_tv_sensor_slew_down, "?slew_down@avTVSensor@cockpit@@UEAAXN@Z");
    load_export(module, g_av_tv_sensor_move_horizontal_abs,
                "?on_TV_SENSOR_move_horizontal_abs@avTVSensor@cockpit@@UEAA_NMMN@Z");
    load_export(module, g_av_tv_sensor_move_vertical_abs,
                "?on_TV_SENSOR_move_vertical_abs@avTVSensor@cockpit@@UEAA_NMMN@Z");
    load_export(module, g_av_tv_sensor_slew_stop, "?slew_stop@avTVSensor@cockpit@@UEAAXXZ");
    load_export(module, g_av_tv_sensor_search, "?search@avTVSensor@cockpit@@UEAAXXZ");
    load_export(module, g_av_tv_sensor_uncage, "?uncage@avTVSensor@cockpit@@UEAAXXZ");
    load_export(module, g_av_tv_sensor_stabilize_on_ground, "?stabilizeOnGround@avTVSensor@cockpit@@UEAAX_N@Z");
    load_export(module, g_av_tv_sensor_zoom_in, "?zoom_in@avTVSensor@cockpit@@UEAAXXZ");
    load_export(module, g_av_tv_sensor_zoom_out, "?zoom_out@avTVSensor@cockpit@@UEAAXXZ");
    load_export(module, g_binding_context, "?binding_context@cockpit@@YAPEAVccCockpitContext@1@XZ");
    load_member_export(module, g_platform_position, "?getPlatformPosition@avPlatform@cockpit@@");
    load_member_export(module, g_platform_initial_position, "?getPlatformInitialPosition@avPlatform@cockpit@@");
    load_export(module, g_platform_update_position, "?update_platform_position@avPlatform@cockpit@@QEAAXXZ");
    load_export(module, g_av_tv_sensor_caged, "?caged@avTVSensor@cockpit@@QEBA_NXZ");
    load_export(module, g_av_tv_sensor_zoom, "?getCurrentZoom@avTVSensor@cockpit@@QEBANXZ");
    load_export(module, g_av_tv_sensor_default_fov, "?getDefaultCameraAngle@avTVSensor@cockpit@@QEBANXZ");
    load_export(module, g_av_tv_sensor_view_angle, "?getCurrentViewAngle@avTVSensor@cockpit@@QEBANXZ");
    load_export(module, g_av_tv_sensor_reference_point,
                "?get_reference_point@avTVSensor@cockpit@@QEBAAEBVVec3d@osg@@XZ");
    load_export(module, g_motor_get, "?get@avMotor@cockpit@@QEBANXZ");
    load_export(module, g_motor_get_target, "?get_target@avMotor@cockpit@@QEBANXZ");
    load_export(module, g_motor_force, "?force@avMotor@cockpit@@QEAAXN@Z");
    load_export(module, g_motor_stop, "?stop@avMotor@cockpit@@QEAAXXZ");
    g_contexts_ptr_addr =
        reinterpret_cast<void**>(GetProcAddress(module, "?contexts_ptr@ccCockpitContext@cockpit@@0PAPEAV12@A"));
    load_export(module, g_av_tv_sensor_set_external_designation,
                "?set_external_designation@avTVSensor@cockpit@@UEAAX_N0@Z");
}

static void warn_about_missing_exports()
{
    if (!g_av_tv_sensor_initialize)
        log_message("Initialize: WARNING - avTVSensor::initialize not exported; "
                    "standalone sensor cannot initialize");
    if (!g_av_tv_sensor_init_zoom)
        log_message("Initialize: WARNING - avTVSensor::initZoom not exported; "
                    "will fall back to direct zoom reset");
    if (!g_av_tv_sensor_move_horizontal_abs || !g_av_tv_sensor_move_vertical_abs)
        log_message("Initialize: WARNING - avTVSensor absolute-axis exports missing; "
                    "will fall back to directional slew handlers");
    if (!g_av_tv_sensor_search)
        log_message(
            "Initialize: WARNING - avTVSensor::search not exported; designation can only capture existing tracks");
    if (!g_av_tv_sensor_stabilize_on_ground)
        log_message("Initialize: WARNING - avTVSensor::stabilizeOnGround not exported; "
                    "ground designation will fall back to search");
}

bool initialize()
{
    if (s_initialized)
        return required_exports_present();

    HMODULE module = GetModuleHandleA("CockpitBase.dll");
    log_format("Initialize: build marker %s", irads_build_marker);
    if (!module)
    {
        log_message("Initialize: CockpitBase.dll not yet loaded - will retry");
        return false;
    }

    load_sensor_exports(module);
    log_format("Initialize: contexts_ptr=%s  context=%s  binding_context=%s", g_contexts_ptr_addr ? "ok" : "missing",
               g_contexts_ptr_addr && *g_contexts_ptr_addr ? "ok" : "missing",
               g_binding_context && g_binding_context() ? "ok" : "missing");

    s_initialized = true;

    const bool ok = required_exports_present();
    log_format(ok ? "Initialize: OK" : "Initialize: PARTIAL - some exports missing");
    warn_about_missing_exports();
    return ok;
}
}  // namespace Cockpit_interop
