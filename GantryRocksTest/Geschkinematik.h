//
// MATLAB Compiler: 4.18 (R2012b)
// Date: Wed May 20 10:53:32 2015
// Arguments: "-B" "macro_default" "-W" "cpplib:Geschkinematik" "-T" "link:lib"
// "Geschkinematik.m" 
//

#ifndef __Geschkinematik_h
#define __Geschkinematik_h 1

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

#ifdef EXPORTING_Geschkinematik
#define PUBLIC_Geschkinematik_C_API __global
#else
#define PUBLIC_Geschkinematik_C_API /* No import statement needed. */
#endif

#define LIB_Geschkinematik_C_API PUBLIC_Geschkinematik_C_API

#elif defined(_HPUX_SOURCE)

#ifdef EXPORTING_Geschkinematik
#define PUBLIC_Geschkinematik_C_API __declspec(dllexport)
#else
#define PUBLIC_Geschkinematik_C_API __declspec(dllimport)
#endif

#define LIB_Geschkinematik_C_API PUBLIC_Geschkinematik_C_API


#else

#define LIB_Geschkinematik_C_API

#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_Geschkinematik_C_API 
#define LIB_Geschkinematik_C_API /* No special import/export declaration */
#endif

extern LIB_Geschkinematik_C_API 
bool MW_CALL_CONV GeschkinematikInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_Geschkinematik_C_API 
bool MW_CALL_CONV GeschkinematikInitialize(void);

extern LIB_Geschkinematik_C_API 
void MW_CALL_CONV GeschkinematikTerminate(void);



extern LIB_Geschkinematik_C_API 
void MW_CALL_CONV GeschkinematikPrintStackTrace(void);

extern LIB_Geschkinematik_C_API 
bool MW_CALL_CONV mlxGeschkinematik(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__BORLANDC__)

#ifdef EXPORTING_Geschkinematik
#define PUBLIC_Geschkinematik_CPP_API __declspec(dllexport)
#else
#define PUBLIC_Geschkinematik_CPP_API __declspec(dllimport)
#endif

#define LIB_Geschkinematik_CPP_API PUBLIC_Geschkinematik_CPP_API

#else

#if !defined(LIB_Geschkinematik_CPP_API)
#if defined(LIB_Geschkinematik_C_API)
#define LIB_Geschkinematik_CPP_API LIB_Geschkinematik_C_API
#else
#define LIB_Geschkinematik_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_Geschkinematik_CPP_API void MW_CALL_CONV Geschkinematik(int nargout, mwArray& Angel_P_1, mwArray& Angel_P_2, mwArray& Angel_P_3, const mwArray& x, const mwArray& y, const mwArray& z, const mwArray& theta1, const mwArray& theta2, const mwArray& theta3, const mwArray& v_x, const mwArray& v_y, const mwArray& v_z, const mwArray& f, const mwArray& e, const mwArray& rf);

#endif
#endif
