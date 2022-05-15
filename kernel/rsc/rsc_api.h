/*
 * kernel/rsc/rsc_api.h
 *
 * VIVO Resource Control.
 *
 * Common api for other sub moduler.
 *
 */

#ifndef __RSC_API_H__
#define __RSC_API_H__

#include "rsc_internal.h"

extern int rsc_main_update(struct rsc_upper_req *up,
  struct rsc_policy_data *policy);

#endif /*__RSC_API_H__*/
