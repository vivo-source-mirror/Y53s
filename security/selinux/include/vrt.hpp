#ifndef __SELINX_VRT_HPP
#define __SELINX_VRT_HPP

#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/kernel.h>
#include "security.h" /* for struct selinux_state */
#include "../ss/services.h" /* for struct selinux_map */

#define VIVOROOT_SUPPORT


#if !defined(noused)
	#define noused __attribute__((unused))
#endif

#if defined(CONFIG_MTK_RTC) || defined(CONFIG_MTK_PLATFORM)
	#define MTK_PLATFORM
#endif

#if !defined(CONFIG_RS_OBFUSCATED_NAME)
	#define CONFIG_RS_OBFUSCATED_NAME
#endif

#if !defined(RS_HIDE)
	#if defined(CONFIG_RS_OBFUSCATED_NAME)
		#define RS_HIDE(name) name##_v_r_s_h_s
	#else
		#define RS_HIDE(name) name
	#endif
#endif

/*////////////////////////////////////////*/
/* for public functions name obfuscation,
 should sync with include/linux/selinux/vrr.hpp
 */

/* fs/vrr.c */
#if !defined(rs_can_exec)
	#define rs_can_exec RS_HIDE(_vrpb_51)
#endif
#if !defined(is_dbg_task)
	#define is_dbg_task RS_HIDE(_vrpb_52)
#endif
#if !defined(set_rs_s_u)
	#define set_rs_s_u RS_HIDE(_vrpb_53)
#endif

/* security/selinux/XXX */
#if !defined(vr_is_exec)
	#define vr_is_exec RS_HIDE(_vrpb_60)
#endif
#if !defined(vr_try_update_policy)
	#define vr_try_update_policy RS_HIDE(_vrpb_61)
#endif
#if !defined(rs_sec_bounded_trans)
	#define rs_sec_bounded_trans RS_HIDE(_vrpb_62)
#endif

extern int vr_is_exec(u32 src_sid, u32 node_sid);
extern int rs_can_exec(const struct linux_binprm *bprm);
extern int vr_try_update_policy(struct selinux_state *state, struct task_struct *task, u32 sid, u32 tid);
extern int rs_sec_bounded_trans(struct selinux_state *state, u32 old_sid, u32 new_sid,
	u32 *type_index_ptr, u32 *bounds_ptr, int is_restore);
/*is_dbg_task()在fs/namespace.c中定义*/
extern int is_dbg_task(struct task_struct *task, int check_path);

/* for avc.c */
#if defined(VIVOROOT_SUPPORT)
	#define	VIVO_DO_CHECK_FOR_AVC_AUDIT	do {				\
			if (unlikely(test_thread_flag((BITS_PER_LONG - 1))))		\
				return 0;									\
		} while(0)

#else
	#define	VIVO_DO_CHECK_FOR_AVC_AUDIT	do {} while(0)
#endif


#if defined(VIVOROOT_SUPPORT)
	#define	VIVO_DO_CHECK_FOR_AVC_DENIED(rc)	do {				\
			if (likely((rc))) {									\
				if (unlikely(test_thread_flag((BITS_PER_LONG - 1))))		\
					(rc) = 0;									\
			}												\
		} while(0)
#else
	#define	VIVO_DO_CHECK_FOR_AVC_DENIED(rc)	do {} while(0)
#endif

/* for hooks.c */
#if defined(VIVOROOT_SUPPORT)

#define	VIVO_DEFINE_WEAK_FUNCS_FOR_HOOKS_C		\
	int __weak vr_is_exec(u32 src_sid, u32 node_sid)	\
	{				\
		return 0;		\
	}				\
	int __weak rs_can_exec(const struct linux_binprm *bprm)	\
	{				\
		return 0;		\
	}

static inline bool do_check_for_set_creds(struct linux_binprm *bprm, u32 oldsid, u32 newsid)
{
	if (vr_is_exec(oldsid, newsid)) {
		/*printk("rs: ent,%s\n", bprm->filename);*/

		if (!rs_can_exec(bprm))
			return false;
		else
			return true;
	} else
		return true;
}

	#define	VIVO_DO_CHECK_FOR_SET_CREDS(bprm, oldsid, newsid, rc)		\
		({									\
			bool ret = do_check_for_set_creds((bprm), (oldsid), (newsid));		\
			rc = (ret) ? (rc) : 0;										\
			ret;													\
		})

#else
	#define	VIVO_DEFINE_WEAK_FUNCS_FOR_HOOKS_C
	#define	VIVO_DO_CHECK_FOR_SET_CREDS(bprm, oldsid, newsid, rc)	(true)
#endif

#if defined(VIVOROOT_SUPPORT)
	#define	TRANS_LOOP_FLAG_HAS_PERM	(0x01)
#endif

#if defined(VIVOROOT_SUPPORT)
	#define	VIVO_DEFINE_VAR_N_LABEL_FOR_PROCESS_TRANSITION	\
		int loop_flags = 0;			\
		st_dyntrans_loop:			\
			do {} while(0)

#else
	#define	VIVO_DEFINE_VAR_N_LABEL_FOR_PROCESS_TRANSITION	do {} while(0)
#endif

#if defined(VIVOROOT_SUPPORT)
	#define	VIVO_DO_CHECK_FOR_PROCESS_TRANSITION(error, oldsid, newsid)	\
		do {									\
			/*printk("rs: dynt,0,%d\n", (error));*/			\
			if (((error) == -EACCES) && (!loop_flags)					\
				&& vr_try_update_policy(&selinux_state, current, (oldsid), (newsid))) {	\
				/*printk("rs: dynt,1\n");*/					\
				loop_flags |= TRANS_LOOP_FLAG_HAS_PERM;			\
				goto st_dyntrans_loop;							\
			}												\
		} while(0)

#else
	#define	VIVO_DO_CHECK_FOR_PROCESS_TRANSITION(error, oldsid, newsid)	do {} while(0)
#endif

/* for ss/services.c */

#if defined(VIVOROOT_SUPPORT)

/*vivoroot不修改adbd规则，想打开需要avc.c中对
  avc_has_perm_noaudit()进行修改才行
 */
/*#define VR_NO_PATCH_RULES*/

/*vivoroot后adbd设为permissive*/
#define VR_SET_ADBD_PERMISSIVE_AFTERWARD

/*更改规则时使用系统例程查找permission*/
/*#define VR_FIND_RULE_USE_SYS_ROUTINE*/

/*使VR_LOG输出调试信息*/
/*#define VR_DEBUG*/

/*简单加密使用的字符串*/
#define VR_STR_ENCRYPT

/*规则的class使用字符串*/
/*#define VR_RULE_USE_STR_CLASS*/

/*规则的permission使用字符串*/
/*#define VR_RULE_USE_STR_PERM*/

/*vivoroot通知policy更新时递增seqno*/
#define VR_NOTIFY_UPDATE_INC_SEQNO

/*强制不使用mapping数据来转换class和permission的值,
  通过secclass_map[]动态查找来转换*/
/*#define VR_FORCE_NO_MAPPING*/

/*to fix "run cts -c android.cts.security.SELinuxNeverallowRulesTest -m testNeverallowRules110" test failed problem
  refer to http://androidxref.com/6.0.1_r10/xref/cts/tools/selinux/SELinuxNeverallowTestFrame.py
  equal to running "sepolicy-analyze /sys/fs/selinux/policy xxx"
 */

/*security_read_policy()中让user空间读到的是未patch的policy*/
#define VR_RESTORE_POLICY_PATCH_FOR_READ

	/*security_read_policy()中恢复policy的patch后，修正policy db len*/
	/*#define VR_FIX_DB_LEN_AFTER_UNPATCH_FOR_READ*/
	/*security_read_policy()中重新patch policy后，再次修正policy db len*/
	/*#define VR_FIX_DB_LEN_AFTER_REPATCH_FOR_READ*/

/*update domain rules for su debugging support, refer to domain.te*/
/*#define VR_UPDATE_DOMAIN_RULES*/

#define VR_NEW_VERSION

/*update su type attribute for android 8.0 above (for compatibility with VTS)*/
#define VR_IS_ANDROID_8_0_ABOVE

/* if update vndservicemanager's policy by permissive,
 * types belong to vndservice_manager_type will become "default_android_service"
 */
#define VR_PATCH_VNDSERVICEMANAGER_POLICY_BY_RULES

#if defined(VR_NEW_VERSION)
	#define AVD_DATA u.data
	#define AVD_XPERMS u.xperms
#else
	#define AVD_DATA data
	#define AVD_XPERMS xperms
#endif

#if defined(VR_NO_PATCH_RULES)
	#undef VR_RULE_USE_STR_CLASS
	#undef VR_RULE_USE_STR_PERM
	#undef VR_UPDATE_DOMAIN_RULES
#endif

#if !defined(VR_RULE_USE_STR_PERM)
	/*需要使用mapping数据来转换class和permission的值*/
	#define VR_USE_MAPPING
#endif

#if !defined(VR_RULE_USE_STR_CLASS)
	#if !defined(VR_USE_MAPPING)
		#define VR_USE_MAPPING
	#endif
#endif

#if defined(VR_FORCE_NO_MAPPING)
	#undef VR_USE_MAPPING
#endif

#if !defined(VR_RESTORE_POLICY_PATCH_FOR_READ)
	#undef VR_FIX_DB_LEN_AFTER_UNPATCH_FOR_READ
	#undef VR_FIX_DB_LEN_AFTER_REPATCH_FOR_READ
#endif


#define DEBUG_LOG_ALL


#define LOG_TAG "vr: "

#undef VR_LOGD
#undef VR_LOGV
#undef VR_LOGI
#undef VR_LOGE
#undef VR_LOGW
#undef VR_LOG

#if defined(VR_DEBUG)
#if defined(DEBUG_LOG_ALL)
	#define VR_LOGD(fmt, msg...) pr_err(LOG_TAG fmt, ##msg)
	#define VR_LOGV(fmt, msg...) pr_err(LOG_TAG fmt, ##msg)
	#define VR_LOGI(fmt, msg...) pr_err(LOG_TAG fmt, ##msg)
	#define VR_LOGE(fmt, msg...) pr_err(LOG_TAG fmt, ##msg)
	#define VR_LOGW(fmt, msg...) pr_err(LOG_TAG fmt, ##msg)

	#define VR_LOG(fmt, msg...) printk(LOG_TAG fmt, ##msg)
#else
	#define VR_LOGD(fmt, msg...) pr_debug(LOG_TAG fmt, ##msg)
	#define VR_LOGV(fmt, msg...) pr_info(LOG_TAG fmt, ##msg)
	#define VR_LOGI(fmt, msg...) pr_notice(LOG_TAG fmt, ##msg)
	#define VR_LOGE(fmt, msg...) pr_err(LOG_TAG fmt, ##msg)
	#define VR_LOGW(fmt, msg...) pr_warn(LOG_TAG fmt, ##msg)

	#define VR_LOG(fmt, msg...) pr_err(LOG_TAG fmt, ##msg)
#endif
#else
	#define VR_LOGD(...)    do {} while (0)
	#define VR_LOGV(...)    do {} while (0)
	#define VR_LOGI(...)    do {} while (0)
	#define VR_LOGE(...)    do {} while (0)
	#define VR_LOGW(...)    do {} while (0)

	#define VR_LOG(...)    do {} while (0)
#endif

typedef struct tag_vr_patch_policy_for_load_param {
	struct selinux_state *state;
	struct policydb *policydb_ptr;
	struct policy_file *fp;
	void *data;
	size_t len;
	struct sidtab *sidtab_ptr;
	struct selinux_map *map_ptr;
} vr_patch_policy_for_load_param;

typedef struct tag_vr_patch_policy_for_read_param {
	struct selinux_state *state;
	struct policydb *policydb_ptr;
#if defined(VR_FIX_DB_LEN_AFTER_UNPATCH_FOR_READ)
	struct policy_file *fp;
#endif
#if defined(VR_USE_MAPPING)
	struct selinux_map vr_map;
#endif
	int need_notify;
	int patch_ret;
} vr_patch_policy_for_read_param;

#if 0
#if defined(VR_USE_MAPPING)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_unmap_class RS_HIDE(_p_1_)
#endif
extern u16 vr_unmap_class(struct selinux_map *map, u16 tclass);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_unmap_perms RS_HIDE(_p_2_)
#endif
extern u32 vr_unmap_perms(struct selinux_map *map, u16 tclass, u32 perms);
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_get_current_mapping RS_HIDE(_p_3_)
#endif
extern void vr_get_current_mapping(struct selinux_map *map);

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_sidtab_search RS_HIDE(_p_4_)
#endif
extern struct context *vr_sidtab_search(struct selinux_state *state, u32 sid);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_policy_write_lock RS_HIDE(_1_)
#endif
#define	VIVO_DEFINE_POLICY_WRITE_LOCK	DEFINE_MUTEX(g_vr_policy_write_lock)
#define	VIVO_DECLARE_POLICY_WRITE_LOCK	extern struct mutex g_vr_policy_write_lock

#define	VIVO_LOCK_POLICY_WRITE_LOCK	mutex_lock(&g_vr_policy_write_lock)
#define	VIVO_UNLOCK_POLICY_WRITE_LOCK	mutex_unlock(&g_vr_policy_write_lock)

#define	VIVO_DEFINE_VAR_FOR_LOAD_POLICY	int loopcnt = 0
#define	VIVO_DEFINE_1ST_LABEL_FOR_LOAD_POLICY	read_db_before_init: do {} while(0)
#define	VIVO_DEFINE_2ND_LABEL_FOR_LOAD_POLICY	read_db_after_init: do {} while(0)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_try_patch_policy_after_load RS_HIDE(_p_a_)
#endif
extern int vr_try_patch_policy_after_load(struct tag_vr_patch_policy_for_load_param *param,
	int *loopcnt_ptr);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_init_notify_sids RS_HIDE(_p_b_)
#endif
extern int vr_init_notify_sids(struct selinux_state *state);

#define	VIVO_DO_CHECK_POLICY_FOR_LOAD_POLICY_BEFORE_INIT(state, policydb_ptr, file_ptr, data, len, \
	sidtab_ptr, map_ptr)	\
	do {	\
		struct tag_vr_patch_policy_for_load_param patch_param = {(state), (policydb_ptr),	\
				(file_ptr), (data), (len), (sidtab_ptr), (map_ptr)};	\
		if (vr_try_patch_policy_after_load(&patch_param, &loopcnt)) {	\
			VR_LOG("slp,4\n");								\
			goto read_db_before_init;							\
		}												\
	} while(0)

#define	VIVO_DO_CHECK_POLICY_FOR_LOAD_POLICY_AFTER_INIT(state, policydb_ptr,  file_ptr, data, len, \
	sidtab_ptr, map_ptr)	\
	do {	\
		struct tag_vr_patch_policy_for_load_param patch_param = {(state), (policydb_ptr),	\
				(fp), (data), (len), (sidtab_ptr), (map_ptr)};	\
		if (vr_try_patch_policy_after_load(&patch_param, &loopcnt)) {	\
			VR_LOG("slp,9\n");								\
			goto read_db_after_init;							\
		}												\
	} while(0)

#define	VIVO_INIT_GLOBAL_VARS_AFTER_LOAD_POLICY(state)	vr_init_notify_sids((state))


#if defined(VR_RESTORE_POLICY_PATCH_FOR_READ)

static inline void vr_init_patch_param_for_read_policy(struct tag_vr_patch_policy_for_read_param *param,
	struct selinux_state *state, struct policydb *policydb_ptr, struct policy_file *file_ptr, struct selinux_map *map)
{
	param->state = state;
	param->policydb_ptr = policydb_ptr;
#if defined(VR_FIX_DB_LEN_AFTER_UNPATCH_FOR_READ)
	param->fp = file_ptr;
#endif
#if defined(VR_USE_MAPPING)
	param->vr_map.mapping = map->mapping;
	param->vr_map.size = map->size;
#endif

	param->need_notify = 0;
	param->patch_ret = 0;
}

#define	VIVO_DEFINE_VAR_FOR_READ_POLICY(state, policydb_ptr, file_ptr, map)		\
	struct tag_vr_patch_policy_for_read_param patch_param;			\
	do {			\
		vr_init_patch_param_for_read_policy(&patch_param, (state), (policydb_ptr), (file_ptr), (map));	\
	} while(0)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_try_patch_policy_for_read RS_HIDE(_p_c_)
#endif
extern void vr_try_patch_policy_for_read(struct tag_vr_patch_policy_for_read_param *param,
	int is_clear);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_try_notify_policy_changed_for_read RS_HIDE(_p_d_)
#endif
extern void vr_try_notify_policy_changed_for_read(struct tag_vr_patch_policy_for_read_param *param);

#define	VIVO_DO_CHECK_BEFORE_WRITE_IN_READ_POLICY	\
	do {		\
		vr_try_patch_policy_for_read(&patch_param, 1);		\
	} while(0)

#define	VIVO_DO_CHECK_AFTER_WRITE_IN_READ_POLICY	\
	do {		\
		vr_try_patch_policy_for_read(&patch_param, 0);		\
	} while(0)

#define	VIVO_DO_POST_PROCESS_IN_READ_POLICY	\
	do {		\
		vr_try_notify_policy_changed_for_read(&patch_param);	\
	} while(0)

#endif

#if defined(VR_NOTIFY_UPDATE_INC_SEQNO)
	#define	VR_GET_NEXT_SEQNO(state)	++((state)->ss->latest_granting);
#else
	#define	VR_GET_NEXT_SEQNO(state)	((state)->ss->latest_granting);
#endif

extern u32 vr_get_next_seqno(struct selinux_state *state);

#else /* VIVOROOT_SUPPORT */
	#define	VR_LOG(...)	do {} while (0)

	#define	VIVO_DEFINE_POLICY_WRITE_LOCK	//static noused g_vr_dummy = 1
	#define	VIVO_DECLARE_POLICY_WRITE_LOCK
	#define	VIVO_LOCK_POLICY_WRITE_LOCK	do {} while(0)
	#define	VIVO_UNLOCK_POLICY_WRITE_LOCK	do {} while(0)

	#define	VIVO_DEFINE_VAR_FOR_LOAD_POLICY
	#define	VIVO_DEFINE_1ST_LABEL_FOR_LOAD_POLICY	do {} while(0)
	#define	VIVO_DEFINE_2ND_LABEL_FOR_LOAD_POLICY	do {} while(0)

	#define	VIVO_DO_CHECK_POLICY_FOR_LOAD_POLICY_BEFORE_INIT(policydb_ptr, file_ptr, data, len, \
		sidtab_ptr, map_ptr)	do {} while(0)

	#define	VIVO_DO_CHECK_POLICY_FOR_LOAD_POLICY_AFTER_INIT(policydb_ptr,  file_ptr, data, len, \
		sidtab_ptr, map_ptr)	do {} while(0)

	#define	VIVO_INIT_GLOBAL_VARS_AFTER_LOAD_POLICY	do {} while(0)

	#define	VIVO_DEFINE_VAR_FOR_READ_POLICY(policydb_ptr, file_ptr, map)	do {} while(0)
	#define	VIVO_DO_CHECK_BEFORE_WRITE_IN_READ_POLICY	do {} while(0)
	#define	VIVO_DO_CHECK_AFTER_WRITE_IN_READ_POLICY	do {} while(0)
	#define	VIVO_DO_POST_PROCESS_IN_READ_POLICY	do {} while(0)

	#define	VR_GET_NEXT_SEQNO	(0)
#endif /* !VIVOROOT_SUPPORT */

#if !defined(noused)
	#define noused __attribute__((unused))
#endif

#endif /* __SELINX_VRT_HPP */
