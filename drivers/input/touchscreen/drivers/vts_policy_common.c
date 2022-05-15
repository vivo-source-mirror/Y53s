#include "vts_policy.h"

struct vts_policy policy_default = {
	.name = "default",
	.expected_mode = vts_policy_expected_mode,
	.modechange_allow = vts_policy_modechange_allow,
	.on_event_down = vts_policy_on_event_down,
	.on_event_up = vts_policy_on_event_up,
	.on_point_down = vts_policy_on_point_down,
	.on_point_up = vts_policy_on_point_up,
	.on_state_changed = vts_policy_on_state_changed,
	.incell_proc = vts_incell_proc,
	.on_post_key = vts_policy_on_post_key,
	.on_aoi_down = vts_policy_on_aoi_down,
	.on_aoi_up = vts_policy_on_aoi_up,
	.get_expect_proc = vts_policy_get_expected_proc,
};


