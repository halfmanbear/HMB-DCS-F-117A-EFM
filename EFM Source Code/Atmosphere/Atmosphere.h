#include "../stdafx.h"

namespace F117
{
	namespace ATMOS
	{
		// Simple atmospheric calculations.
		// Inputs: temperature (K), density (kg/m³), vt (m/s)
		// Outputs: coeff[0] = dynamic pressure (Pa), coeff[1] = Mach (dimensionless)
		void atmos(double temperature, double density, double vt, double *coeff)
		{
			constexpr double kGammaAir = 1.4;
			constexpr double kRAir     = 287.058; // J/(kg·K)

			double mach = vt / sqrt(kGammaAir * kRAir * temperature);
			double qbar = 0.5 * density * vt * vt;

			coeff[0] = qbar;
			coeff[1] = mach;
		}
	}
}