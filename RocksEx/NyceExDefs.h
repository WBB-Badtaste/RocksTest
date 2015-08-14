#pragma once

#include <nycedefs.h>

#define ROCKS_EXTERN_MASK								30

#define ROCKS_ERR_PU_RATE_ERROR							((NYCE_STATUS)((NYCE_ERROR_MASK)|(SS_ROCKS<<NYCE_SUBSYS_SHIFT)|(ROCKS_EXTERN_MASK + 0)))
#define ROCKS_ERR_DELTA_PARS_ERROR						((NYCE_STATUS)((NYCE_ERROR_MASK)|(SS_ROCKS<<NYCE_SUBSYS_SHIFT)|(ROCKS_EXTERN_MASK + 1)))
#define ROCKS_ERR_DELTA_TAJ_OVER_WORKSPACE				((NYCE_STATUS)((NYCE_ERROR_MASK)|(SS_ROCKS<<NYCE_SUBSYS_SHIFT)|(ROCKS_EXTERN_MASK + 2)))
#define ROCKS_ERR_DELTA_TAJ_VEL_ERROR					((NYCE_STATUS)((NYCE_ERROR_MASK)|(SS_ROCKS<<NYCE_SUBSYS_SHIFT)|(ROCKS_EXTERN_MASK + 3)))
#define ROCKS_ERR_DELTA_JOINT_POS_ERROR					((NYCE_STATUS)((NYCE_ERROR_MASK)|(SS_ROCKS<<NYCE_SUBSYS_SHIFT)|(ROCKS_EXTERN_MASK + 4)))
#define ROCKS_ERR_DELTA_POSTURE_ERROR					((NYCE_STATUS)((NYCE_ERROR_MASK)|(SS_ROCKS<<NYCE_SUBSYS_SHIFT)|(ROCKS_EXTERN_MASK + 5)))


#define ROCKS_MOVE_TYPE_SPIRAL							10