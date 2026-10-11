#ifndef K_INVERSE_INTERNAL_H
#define K_INVERSE_INTERNAL_H
#include <manipulator/inverse.h>
#include "internal.h"
k_status k_internal_dls(k_real *columns, size_t count, const k_real error[3],
                        const k_ik_options *options, k_real *step, k_real gradient_rhs[3],
                        k_real *effective_damping);
k_status k_internal_direction(const k_real *columns, size_t count, const k_real rhs[3],
                              const k_ik_options *options, k_real *step);
#endif
