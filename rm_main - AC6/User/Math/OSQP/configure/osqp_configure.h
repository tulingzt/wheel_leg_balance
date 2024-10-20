#ifndef OSQP_CONFIGURE_H
# define OSQP_CONFIGURE_H


/* OSQP_ENABLE_DEBUG */
#define OSQP_ENABLE_DEBUG

/* Operating system */
//#define IS_LINUX
//#define IS_MAC
#define IS_WINDOWS

/* Algebra backend in use - Only one of the following is defined at compilation time */
#define OSQP_ALGEBRA_BUILTIN
#define OSQP_ALGEBRA_MKL
#define OSQP_ALGEBRA_CUDA

/* Enable code generation */
#define OSQP_CODEGEN

/* Enable profiler annotations */
#define OSQP_PROFILER_ANNOTATIONS

/* Enable derivative computation in the solver */
#define OSQP_ENABLE_DERIVATIVES

/* OSQP_EMBEDDED_MODE */
//#define OSQP_EMBEDDED_MODE (@OSQP_EMBEDDED_MODE@)
#define OSQP_EMBEDDED_MODE 1

/* Header file containing custom memory allocators */
#define OSQP_CUSTOM_MEMORY "@OSQP_CUSTOM_MEMORY@"

///* OSQP_ENABLE_PRINTING */
//#define OSQP_ENABLE_PRINTING

///* Header file containing custom printing functions */
//#define OSQP_CUSTOM_PRINTING "@OSQP_CUSTOM_PRINTING@"

///* OSQP_ENABLE_PROFILING */
//#define OSQP_ENABLE_PROFILING

///* OSQP_ENABLE_INTERRUPT */
//#define OSQP_ENABLE_INTERRUPT

/* OSQP_USE_FLOAT */
#define OSQP_USE_FLOAT

///* OSQP_USE_LONG */
//#define OSQP_USE_LONG

#endif /* ifndef OSQP_CONFIGURE_H */
