#include "stdafx.h"
#include "MassLog.h"

#include <cmath>
#include <string>

#include "LogFile.h"
#include "Paths.h"

namespace
{
Log_file mass_log;
bool mass_log_tried = false;
double logged[7] = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // mass, CG x/y/z, inertia roll/yaw/pitch

Log_file* open_mass_log()
{
    if (!mass_log_tried)
    {
        mass_log_tried = true;
        const char* folders[] = {"DCS", "DCS.openbeta"};
        for (const char* folder : folders)
        {
            const std::string path = build_saved_games_dcs_log_path(folder, "F117_Mass_State.log");
            if (!path.empty() && mass_log.open(path.c_str(), false))
                break;
        }
        mass_log.write("event,mass_kg,x,y,z,I_roll,I_yaw,I_pitch\n");
    }
    return mass_log.is_open() ? &mass_log : nullptr;
}
}  // namespace

void mass_log_state(double mass_kg, double cg_x, double cg_y, double cg_z, double roll_inertia, double yaw_inertia,
                    double pitch_inertia)
{
    const double now[7] = {mass_kg, cg_x, cg_y, cg_z, roll_inertia, yaw_inertia, pitch_inertia};
    const double tolerance[7] = {20.0, 0.005, 0.005, 0.005, 50.0, 50.0, 50.0};
    bool changed = false;
    for (int i = 0; i < 7; ++i)
        changed = changed || std::abs(now[i] - logged[i]) > tolerance[i];
    if (!changed)
        return;
    for (int i = 0; i < 7; ++i)
        logged[i] = now[i];
    if (Log_file* log = open_mass_log())
        log->write("mass_state,%.1f,%.3f,%.3f,%.3f,%.0f,%.0f,%.0f\n", mass_kg, cg_x, cg_y, cg_z, roll_inertia,
                   yaw_inertia, pitch_inertia);
}

void mass_log_piece(int tank, double mass_kg, double x, double y, double z, double roll_inertia, double yaw_inertia,
                    double pitch_inertia)
{
    if (std::abs(mass_kg) < 20.0)
        return;
    if (Log_file* log = open_mass_log())
        log->write("tank_%d,%.1f,%.3f,%.3f,%.3f,%.0f,%.0f,%.0f\n", tank, mass_kg, x, y, z, roll_inertia,
                   yaw_inertia, pitch_inertia);
}

void mass_log_event(const char* name, double value)
{
    if (Log_file* log = open_mass_log())
        log->write("%s,%.1f\n", name, value);
}

void mass_log_release(double fuel_kg)
{
    mass_log_event("release", fuel_kg);
    logged[0] = -1.0;
}
