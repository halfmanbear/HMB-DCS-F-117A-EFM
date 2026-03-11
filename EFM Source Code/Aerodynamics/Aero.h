#include "../stdafx.h"
#include "AeroData.h"
/*
Cx is drag
Cd is also drag
Cl is lift
p is atmospheric pressure
q is dynamic pressure, like on the plane
r is turn rate.
*/
namespace F117
{
	namespace AERO
	{		
		double		Cx_total				= 0.0;
		double		Cx						= 0.0;
		double		dXdQ					= 0.0;
		double		Cxq						= 0.0;
		double		Cz_total				= 0.0;
		double		Cz						= 0.0;
		double		dZdQ					= 0.0;
		double		Czq						= 0.0;
		double		Cm_total				= 0.0;
		double		Cm						= 0.0;
		double		eta_el					= 0.0;
		double		dMdQ					= 0.0;
		double		Cmq						= 0.0;
		double		Cm_delta				= 0.0;
		double		Cm_delta_ds				= 0.0;
		double		Cy_total				= 0.0;
		double		Cy						= 0.0;
		double		dYdail					= 0.0;
		double		Cy_delta_r30			= 0.0;
		double		dYdR					= 0.0;
		double		dYdP					= 0.0;
		double		Cy_delta_a20			= 0.0;
		double		Cyr						= 0.0;
		double		Cyp						= 0.0;
		double		Cn_total				= 0.0;
		double		Cn						= 0.0;
		double		dNdail					= 0.0;
		double		Cn_delta_r30			= 0.0;
		double		dNdR					= 0.0;
		double		dNdP					= 0.0;
		double		Cn_delta_beta			= 0.0;
		double		Cn_delta_a20			= 0.0;
		double		Cnr						= 0.0;
		double		Cnp						= 0.0;
		double		Cl_total				= 0.0;
		double		Cl						= 0.0;
		double		dLdail					= 0.0;
		double		Cl_delta_r30			= 0.0;
		double		dLdR					= 0.0;
		double		dLdP					= 0.0;
		double		Cl_delta_beta			= 0.0;
		double		Cl_delta_a20			= 0.0;
		double		Clr						= 0.0;
		double		Clp						= 0.0;
		double		CxWheelFriction = 0.0;
		double		CyWheelFriction = 1.0;

		struct InterpCache
		{
			int flag = 0;
			double **axes = NULL;
			ND_INFO ndinfo = {};
		};

		inline void init_interp_cache(InterpCache &cache, int nDimension, const int *pointCounts, double **axes)
		{
			if (cache.flag == 0)
			{
				cache.flag = 1;
				cache.ndinfo.nDimension = nDimension;
				cache.ndinfo.nPoints = intVector(nDimension);
				cache.axes = (double **)malloc(nDimension * sizeof(double*));
				for (int i = 0; i < nDimension; ++i)
				{
					cache.ndinfo.nPoints[i] = pointCounts[i];
					cache.axes[i] = axes[i];
				}
			}
		}

		inline double lookup_1d(InterpCache &cache, double *data, int points0, double *axis0, double x0)
		{
			int pointCounts[1] = { points0 };
			double *axes[1] = { axis0 };
			double x[1] = { x0 };
			init_interp_cache(cache, 1, pointCounts, axes);
			return interpn(cache.axes, data, x, cache.ndinfo);
		}

		inline double lookup_2d(InterpCache &cache, double *data,
			int points0, double *axis0,
			int points1, double *axis1,
			double x0, double x1)
		{
			int pointCounts[2] = { points0, points1 };
			double *axes[2] = { axis0, axis1 };
			double x[2] = { x0, x1 };
			init_interp_cache(cache, 2, pointCounts, axes);
			return interpn(cache.axes, data, x, cache.ndinfo);
		}

		inline double lookup_3d(InterpCache &cache, double *data,
			int points0, double *axis0,
			int points1, double *axis1,
			int points2, double *axis2,
			double x0, double x1, double x2)
		{
			int pointCounts[3] = { points0, points1, points2 };
			double *axes[3] = { axis0, axis1, axis2 };
			double x[3] = { x0, x1, x2 };
			init_interp_cache(cache, 3, pointCounts, axes);
			return interpn(cache.axes, data, x, cache.ndinfo);
		}

		double _Cx(double alpha, double beta, double dele)
		{
			static InterpCache cache;
			return lookup_3d(cache, _CxData, 20, alpha1, 19, beta1, 5, dh1, alpha, beta, dele);
		}

		double _Cz(double alpha, double beta, double dele)
		{
			static InterpCache cache;
			return lookup_3d(cache, _CzData, 20, alpha1, 19, beta1, 5, dh1, alpha, beta, dele);
		}

		double _Cm(double alpha, double beta, double dele)
		{
			static InterpCache cache;
			return lookup_3d(cache, _CmData, 20, alpha1, 19, beta1, 5, dh1, alpha, beta, dele);
		}

		double _Cy(double alpha, double beta)
		{
			static InterpCache cache;
			// For a symmetric airframe, side-force coefficient is odd in beta.
			const double betaSymmetric = std::abs(beta);
			const double betaSign = (beta < 0.0) ? -1.0 : 1.0;
			return betaSign * lookup_2d(cache, _CyData, 20, alpha1, 19, beta1, alpha, betaSymmetric);
		}

		double _Cn(double alpha, double beta, double dele)
		{
			static InterpCache cache;
			// For a symmetric airframe, yawing-moment coefficient is odd in beta.
			const double betaSymmetric = std::abs(beta);
			const double betaSign = (beta < 0.0) ? -1.0 : 1.0;
			return betaSign * lookup_3d(cache, _CnData, 20, alpha1, 19, beta1, 3, dh2, alpha, betaSymmetric, dele);
		}

		double _Cl(double alpha, double beta, double dele)
		{
			static InterpCache cache;
			// For a symmetric airframe, rolling-moment coefficient is odd in beta.
			const double betaSymmetric = std::abs(beta);
			const double betaSign = (beta < 0.0) ? -1.0 : 1.0;
			return betaSign * lookup_3d(cache, _ClData, 20, alpha1, 19, beta1, 3, dh2, alpha, betaSymmetric, dele);
		}

		double _CXq(double alpha)
		{
			static InterpCache cache;
			return lookup_1d(cache, _CxqData, 20, alpha1, alpha);
		}

		double _CZq(double alpha)
		{
			static InterpCache cache;
			return lookup_1d(cache, _CzqData, 20, alpha1, alpha);
		}

		double _CMq(double alpha)
		{
			static InterpCache cache;
			return lookup_1d(cache, _CmqData, 20, alpha1, alpha);
		}

		double _CYp(double alpha)
		{
			static InterpCache cache;
			return lookup_1d(cache, _CypData, 20, alpha1, alpha);
		}

		double _CYr(double alpha)
		{
			static InterpCache cache;
			return lookup_1d(cache, _CyrData, 20, alpha1, alpha);
		}

		double _CNr(double alpha)
		{
			static InterpCache cache;
			return lookup_1d(cache, _CnrData, 20, alpha1, alpha);
		}

		double _CNp(double alpha)
		{
			static InterpCache cache;
			return lookup_1d(cache, _CnpData, 20, alpha1, alpha);
		}

		double _CLp(double alpha)
		{
			static InterpCache cache;
			return lookup_1d(cache, _ClpData, 20, alpha1, alpha);
		}

		double _CLr(double alpha)
		{
			static InterpCache cache;
			return lookup_1d(cache, _ClrData, 20, alpha1, alpha);
		}

		double _Cy_r30(double alpha, double beta)
		{
			static InterpCache cache;
			return lookup_2d(cache, _Cy_r30Data, 20, alpha1, 19, beta1, alpha, beta);
		}

		double _Cn_r30(double alpha, double beta)
		{
			static InterpCache cache;
			return lookup_2d(cache, _Cn_r30Data, 20, alpha1, 19, beta1, alpha, beta);
		}

		double _Cl_r30(double alpha, double beta)
		{
			static InterpCache cache;
			return lookup_2d(cache, _Cl_r30Data, 20, alpha1, 19, beta1, alpha, beta);
		}

		double _Cy_a20(double alpha, double beta)
		{
			static InterpCache cache;
			return lookup_2d(cache, _Cy_a20Data, 20, alpha1, 19, beta1, alpha, beta);
		}

		double _Cn_a20(double alpha, double beta)
		{
			static InterpCache cache;
			return lookup_2d(cache, _Cn_a20Data, 20, alpha1, 19, beta1, alpha, beta);
		}

		double _Cl_a20(double alpha, double beta)
		{
			static InterpCache cache;
			return lookup_2d(cache, _Cl_a20Data, 20, alpha1, 19, beta1, alpha, beta);
		}

		double _delta_CNbeta(double alpha)
		{
			static InterpCache cache;
			return lookup_1d(cache, _delta_CNbetaData, 20, alpha1, alpha);
		}

		double _delta_CLbeta(double alpha)
		{
			static InterpCache cache;
			return lookup_1d(cache, _delta_CLbetaData, 20, alpha1, alpha);
		}

		double _delta_Cm(double alpha)
		{
			static InterpCache cache;
			return lookup_1d(cache, _delta_CmData, 20, alpha1, alpha);
		}

		double _eta_el(double el)
		{
			static InterpCache cache;
			return lookup_1d(cache, _eta_elData, 5, dh1, el);
		}

		/*
		double _delta_Cm_ds(double alpha, double el){
		...............
		...............
		} End of function(...) */

		void hifi_C(double alpha,double beta,double el,double *retVal){
			retVal[0] = _Cx(alpha,beta,el);
			retVal[1] = _Cz(alpha,beta,el);
			retVal[2] = _Cm(alpha,beta,el);
			retVal[3] = _Cy(alpha,beta);
			retVal[4] = _Cn(alpha,beta,el);
			retVal[5] = _Cl(alpha,beta,el);
		}

		void hifi_damping(double alpha, double *retVal){
			retVal[0] = _CXq(alpha);
			retVal[1] = _CYr(alpha);
			retVal[2] = _CYp(alpha);
			retVal[3] = _CZq(alpha);
			retVal[4] = _CLr(alpha);
			retVal[5] = _CLp(alpha);
			retVal[6] = _CMq(alpha);
			retVal[7] = _CNr(alpha);
			retVal[8] = _CNp(alpha);
		}

		void hifi_rudder(double alpha, double beta, double *retVal){
				// For a symmetric airframe, rudder control derivatives should be even in beta.
				// The current negative-beta rudder tables are asymmetric and drive a one-sided departure,
				// so use the positive-beta half as the canonical source for both sides.
				const double betaSymmetric = std::abs(beta);
				retVal[0] = _Cy_r30(alpha,betaSymmetric) - _Cy(alpha,betaSymmetric);
				retVal[1] = _Cn_r30(alpha,betaSymmetric) - _Cn(alpha,betaSymmetric,0);
				retVal[2] = _Cl_r30(alpha,betaSymmetric) - _Cl(alpha,betaSymmetric,0);
		}

		void hifi_ailerons(double alpha, double beta, double *retVal){
				retVal[0] = _Cy_a20(alpha,beta) - _Cy(alpha,beta);
				retVal[2] = _Cn_a20(alpha,beta) - _Cn(alpha,beta,0);
				retVal[4] = _Cl_a20(alpha,beta) - _Cl(alpha,beta,0);
		}

		void hifi_other_coeffs(double alpha, double el, double *retVal){
				retVal[0] = _delta_CNbeta(alpha);
				retVal[1] = _delta_CLbeta(alpha);
				retVal[2] = _delta_Cm(alpha);
				retVal[3] = _eta_el(el);
				retVal[4] = 0;       /* ignore deep-stall regime, delta_Cm_ds = 0 */
		}
	}
}
