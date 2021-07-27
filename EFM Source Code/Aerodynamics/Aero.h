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

		double _Cx(double alpha,double beta,double dele)
		{
		//CX0120_ALPHA1_BETA1_DH1_201.dat
			static int flag = 0;
	
			static double **X;
			static ND_INFO ndinfo ;

			int FILESIZE;
			int nDimension = 3; 
			double x[3];	
			FILESIZE = 1900;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19; 
				ndinfo.nPoints[2] = 5; 
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
				X[2] = dh1;
			}

			x[0] = alpha;
			x[1] = beta;
			x[2] = dele;

			return interpn(X,_CxData,x,ndinfo);
		}/* End of function(...) */

		double _Cz(double alpha,double beta, double dele){
		//CZ0120_ALPHA1_BETA1_DH1_301.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 3; /* alpha,beta,dele */
			double x[3];	/* Number of dimension */

			FILESIZE = 1900;	/* There are 1900 elements in the 20x19x5 3D array */

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	/* Alpha npoints */
				ndinfo.nPoints[1] = 19; /* Beta npoints  */
				ndinfo.nPoints[2] = 5;  /* dele npoints  */
				X = (double **) malloc(nDimension*sizeof(double*));
				X[0] = alpha1;
				X[1] = beta1;
				X[2] = dh1;
				}
			x[0] = alpha;
			x[1] = beta;
			x[2] = dele;
			return interpn(X,_CzData,x,ndinfo);
		}/* End of function(...) */

		double _Cm(double alpha,double beta,double dele){
		//CM0120_ALPHA1_BETA1_DH1_101.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 3; 
			double x[3];	
			FILESIZE = 1900;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19; 
				ndinfo.nPoints[2] = 5; 
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
				X[2] = dh1;
				}

			x[0] = alpha;
			x[1] = beta;
			x[2] = dele;
			return	interpn(X,_CmData,x,ndinfo);
		}/* End of function(...) */

		double _Cy(double alpha,double beta){
		// CY0320_ALPHA1_BETA1_401.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 2; 
			double x[2];	
			FILESIZE = 380;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */ 
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19; 
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
				}

			x[0] = alpha;
			x[1] = beta;
			return	interpn(X,_CyData,x,ndinfo);
		}/* End of function(...) */

		double _Cn(double alpha, double beta, double dele){
		//CN0120_ALPHA1_BETA1_DH2_501.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 3; 
			double x[3];	
			FILESIZE = 1140;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				ndinfo.nPoints[2] = 3;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
				X[2] = dh2;
				}

			x[0] = alpha;
			x[1] = beta;
			x[2] = dele;
			return (interpn(X,_CnData,x,ndinfo));
		}/* End of function(...) */

		double _Cl(double alpha, double beta,double dele){
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 3; 
			double x[3];	
			FILESIZE = 1140;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				ndinfo.nPoints[2] = 3;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
				X[2] = dh2;
				}

			x[0] = alpha;
			x[1] = beta;
			x[2] = dele;
			return (interpn(X,_ClData,x,ndinfo));
		}/* End of function(...) */

		double _CXq(double alpha){
		//CX1120_ALPHA1_204.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 1; 
			double x[1];	
			FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */ 
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				}

			x[0] = alpha;
			return (interpn(X,_CxqData,x,ndinfo));
		}/* End of function(...) */


		double _CZq(double alpha){
			//CZ1120_ALPHA1_304.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 1; 
			double x[1];	
			FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				}

			x[0] = alpha;
			return (interpn(X,_CzqData,x,ndinfo));
		}/* End of function(...) */


		double _CMq(double alpha){
			//CM1120_ALPHA1_104.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 1; 
			double x[1];	
			FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				}

			x[0] = alpha;
			return (interpn(X,_CmqData,x,ndinfo));
		}/* End of function(...) */


		double _CYp(double alpha){
			//CY1220_ALPHA1_408.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 1; 
			double x[1];	
			FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				}

			x[0] = alpha;
			return (interpn(X,_CypData,x,ndinfo));
		}/* End of function(...) */


		double _CYr(double alpha){
			//CY1320_ALPHA1_406.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 1; 
			double x[1];	
			FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				}

			x[0] = alpha;
			return (interpn(X,_CyrData,x,ndinfo));
		}/* End of function(...) */


		double _CNr(double alpha){
			//CN1320_ALPHA1_506.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 1; 
			double x[1];	
			FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				}

			x[0] = alpha;
			return (interpn(X,_CnrData,x,ndinfo));
		}/* End of function(...) */


		double _CNp(double alpha){
			//CN1220_ALPHA1_508.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 1; 
			double x[1];	
			FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				}

			x[0] = alpha;
			return (interpn(X,_CnpData,x,ndinfo));
		}/* End of function(...) */


		double _CLp(double alpha){
			//CL1220_ALPHA1_608.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 1; 
			double x[1];	
			FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				}

			x[0] = alpha;
			return (interpn(X,_ClpData,x,ndinfo));
		}/* End of function(...) */


		double _CLr(double alpha){
			//CL1320_ALPHA1_606.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 1; 
			double x[1];	
			FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				}

			x[0] = alpha;
			return (interpn(X,_ClrData,x,ndinfo));
		}/* End of function(...) */

		double _Cy_r30(double alpha, double beta){
			//CY0720_ALPHA1_BETA1_405.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 2; 
			double x[2];	
			FILESIZE = 380;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
				}

			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cy_r30Data,x,ndinfo));
		}/* End of function(...) */


		double _Cn_r30(double alpha, double beta){
			//CN0720_ALPHA1_BETA1_503.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 2; 
			double x[2];	
			FILESIZE = 380;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
		
				}

			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cn_r30Data,x,ndinfo));
		}/* End of function(...) */


		double _Cl_r30(double alpha, double beta){
			//CL0720_ALPHA1_BETA1_603.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;

			int FILESIZE;
			int nDimension = 2; 
			double x[2];	
			FILESIZE = 380;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
				}

			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cl_r30Data,x,ndinfo));
		}/* End of function(...) */


		double _Cy_a20(double alpha, double beta){
			//CY0620_ALPHA1_BETA1_403.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 2; 
			double x[2];	
			FILESIZE = 380;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
				}

			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cy_a20Data,x,ndinfo));
		}/* End of function(...) */


		double _Cn_a20(double alpha, double beta){
			//CN0620_ALPHA1_BETA1_504.dat
			static int flag = 0;
			static double *DATA = (double*) NULL;
			static double **X;
			static ND_INFO ndinfo ;	
	
			int FILESIZE;
			int nDimension = 2; 
			double x[2];	
			FILESIZE = 380;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
				}

			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cn_a20Data,x,ndinfo));
		}/* End of function(...) */


		double _Cl_a20(double alpha, double beta){
			//CL0620_ALPHA1_BETA1_604.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
		
			int FILESIZE;
			int nDimension = 2; 
			double x[2];	
			FILESIZE = 380;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
				}
			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cl_a20Data,x,ndinfo));
		}/* End of function(...) */


		double _delta_CNbeta(double alpha){
			//CN9999_ALPHA1_brett.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 1; 
			double x[1];	
			FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				}

			x[0] = alpha;
			return (interpn(X,_delta_CNbetaData,x,ndinfo));
		}/* End of function(...) */


		double _delta_CLbeta(double alpha){
			//CL9999_ALPHA1_brett.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
		
			int FILESIZE;
			int nDimension = 1; 
			double x[1];	
			FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				}

			x[0] = alpha;
			return (interpn(X,_delta_CLbetaData,x,ndinfo));
		}/* End of function(...) */


		double _delta_Cm(double alpha){
			//CM9999_ALPHA1_brett.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int FILESIZE;
			int nDimension = 1; 
			double x[1];	
			FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				}

			x[0] = alpha;
			return (interpn(X,_delta_CmData,x,ndinfo));
		}/* End of function(...) */


		double _eta_el(double el){
			//ETA_DH1_brett.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
		
			int FILESIZE;
			int nDimension = 1; 
			double x[1];	
			FILESIZE = 5;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 5;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = dh1;
				}

			x[0] = el;
			return (interpn(X,_eta_elData,x,ndinfo));
		}/* End of function(...) */


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
				retVal[0] = _Cy_r30(alpha,beta) - _Cy(alpha,beta);
				retVal[1] = _Cn_r30(alpha,beta) - _Cn(alpha,beta,0);
				retVal[2] = _Cl_r30(alpha,beta) - _Cl(alpha,beta,0);
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