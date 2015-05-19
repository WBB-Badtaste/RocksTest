//
// MATLAB Compiler: 4.18 (R2012b)
// Date: Mon May 18 16:26:58 2015
// Arguments: "-B" "macro_default" "-W" "cpplib:DeltaInverseKinVel" "-T"
// "link:lib" "DeltaInverseKinVel.m" 
//

#ifndef __DeltaInverseKinVel_h
#define __DeltaInverseKinVel_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

#if defined(__SUNPRO_CC)
/* Solaris shared libraries use __global, rather than mapfiles
 * to define the API exported from a shared library. __global is
 * only necessary when building the library -- files including
 * this header file to use the library do not need the __global
 * declaration; hence the EXPORTING_<library> logic.
 */

#ifdef EXPORTING_DeltaInverseKinVel
#define PUBLIC_DeltaInverseKinVel_C_API __global
#else
#define PUBLIC_DeltaInverseKinVel_C_API /* No import statement needed. */
#endif

#define LIB_DeltaInverseKinVel_C_API PUBLIC_DeltaInverseKinVel_C_API

#elif defined(_HPUX_SOURCE)

#ifdef EXPORTING_DeltaInverseKinVel
#define PUBLIC_DeltaInverseKinVel_C_API __declspec(dllexport)
#else
#define PUBLIC_DeltaInverseKinVel_C_API __declspec(dllimport)
#endif

#define LIB_DeltaInverseKinVel_C_API PUBLIC_DeltaInverseKinVel_C_API


#else

#define LIB_DeltaInverseKinVel_C_API

#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_DeltaInverseKinVel_C_API 
#define LIB_DeltaInverseKinVel_C_API /* No special import/export declaration */
#endif

extern LIB_DeltaInverseKinVel_C_API 
bool MW_CALL_CONV DeltaInverseKinVelInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_DeltaInverseKinVel_C_API 
bool MW_CALL_CONV DeltaInverseKinVelInitialize(void);

extern LIB_DeltaInverseKinVel_C_API 
void MW_CALL_CONV DeltaInverseKinVelTerminate(void);



extern LIB_DeltaInverseKinVel_C_API 
void MW_CALL_CONV DeltaInverseKinVelPrintStackTrace(void);

extern LIB_DeltaInverseKinVel_C_API 
bool MW_CALL_CONV mlxDeltaInverseKinVel(int nlhs, mxArray *plhs[], int nrhs, mxArray 
                                        *prhs[]);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__BORLANDC__)

#ifdef EXPORTING_DeltaInverseKinVel
#define PUBLIC_DeltaInverseKinVel_CPP_API __declspec(dllexport)
#else
#define PUBLIC_DeltaInverseKinVel_CPP_API __declspec(dllimport)
#endif

#define LIB_DeltaInverseKinVel_CPP_API PUBLIC_DeltaInverseKinVel_CPP_API

#else

#if !defined(LIB_DeltaInverseKinVel_CPP_API)
#if defined(LIB_DeltaInverseKinVel_C_API)
#define LIB_DeltaInverseKinVel_CPP_API LIB_DeltaInverseKinVel_C_API
#else
#define LIB_DeltaInverseKinVel_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_DeltaInverseKinVel_CPP_API void MW_CALL_CONV DeltaInverseKinVel(int nargout, mwArray& vel_theta1, mwArray& vel_theta2, mwArray& vel_theta3, const mwArray& x, const mwArray& y, const mwArray& z, const mwArray& vel_x, const mwArray& vel_y, const mwArray& vel_z, const mwArray& theta1, const mwArray& theta2, const mwArray& theta3, const mwArray& f, const mwArray& rf, const mwArray& e);

#endif
#endif
