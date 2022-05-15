#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/selinux.h>
#include <linux/flex_array.h>
#include <linux/vmalloc.h>

/*begin XieJiYuan@20190923@security*/
#include <linux/vrr.hpp>
/*end*/

#include "flask.h"
#include "avc.h"
#include "avc_ss.h"
#include "security.h"
#include "context.h"
#include "policydb.h"
#include "sidtab.h"
#include "services.h"
#include "conditional.h"
#include "mls.h"
#include "objsec.h"
#include "netlabel.h"
#include "xfrm.h"
#include "ebitmap.h"
#include "audit.h"
#include "vrt.hpp"

#if defined(VIVOROOT_SUPPORT)

VIVO_DEFINE_POLICY_WRITE_LOCK;

/* in case the system.img is userdebug/eng version as VTS testing */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_su_already_permissive RS_HIDE(_1_0)
#endif
static int g_vr_su_already_permissive;


#if defined(VR_IS_ANDROID_8_0_ABOVE)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_policydb_update_typeattribute RS_HIDE(_b_30)
#endif
static int vr_policydb_update_typeattribute(struct policydb *policy,
	const char *type_name, const char *attr_name, int is_clear, int *old_attr_status,
	int *updated_ptr);

extern void set_rs_s_u(void);

void __weak set_rs_s_u(void)
{
}
#endif

/*static int vr_check_access_no_audit(const char *scon,
	const char *tcon, const char *_class, const char *perm);*/

#else
	#define  VR_LOG(...)    do {} while (0)
#endif

/* above before security_load_policy() */
////////////////////////////////////////////////////////

/*begin XieJiYuan@20150706@*/
#if defined(VIVOROOT_SUPPORT)

enum {
	vrseSu,
	vrseAdbd,
#if defined(VR_RULE_USE_STR_CLASS)
	vrseDir,
	vrseFile,
	vrseProcess,
#endif
	vrseSuContext,
	vrseAdbdContext,
#if defined(VR_RULE_USE_STR_PERM)
	vrseSetCurrent,
	vrseDynTransition,
#endif
#if !defined(MTK_PLATFORM)
	vrseSystemDataFile,
	vrseAdbdFile = vrseSystemDataFile,
#else
	vrseMDLogDataFile,
	vrseAdbdFile = vrseMDLogDataFile,
#endif
#if defined(VR_RULE_USE_STR_PERM)
	vrseAdbdDirPerms,
	vrseAdbdFilePerms,
#endif

	vrseShell,
	vrseDumpState,

	vrseInitContext,
	vrseSystemFileContext,

	vrseDomain,
	vrseBinder,
	vrseInit,
	vrseCoreDumpFile,
#if defined(VR_RULE_USE_STR_PERM)
	vrseDomainBinderPerms,
	vrseDomainCoreDumpFilePerms,
#endif

	vrseIncident,
	vrseVndServiceManager,

	vrseCoredomain,
	vrseAppdomain,
	vrseMlstrustedsubject,
	vrseNetdomain,

	vrseHwServiceManager,

	/* add new items before vrseInvalid */
	vrseInvalid,
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vrs_su RS_HIDE(_1)
#define g_vrs_adbd RS_HIDE(_2)
#define g_vrs_dir RS_HIDE(_3)
#define g_vrs_file RS_HIDE(_4)
#define g_vrs_process RS_HIDE(_5)
#define g_vrs_su_context RS_HIDE(_6)
#define g_vrs_adbd_context RS_HIDE(_7)
#define g_vrs_setcurrent RS_HIDE(_8)
#define g_vrs_dyntransition RS_HIDE(_9)
#define g_vrs_system_data_file RS_HIDE(_a)
#define g_vrs_mdlog_data_file RS_HIDE(_a)
#define g_vrs_adbd_dir_perms RS_HIDE(_b)
#define g_vrs_adbd_file_perms RS_HIDE(_c)
#define g_vrs_shell RS_HIDE(_d)
#define g_vrs_dumpstate RS_HIDE(_e)
#define g_vrs_incident RS_HIDE(_f)
#define g_vrs_vndservicemanager RS_HIDE(_g)
#define g_vrs_coredomain RS_HIDE(_h)
#define g_vrs_Appdomain RS_HIDE(_i)
#define g_vrs_Mlstrustedsubject RS_HIDE(_j)
#define g_vrs_Netdomain RS_HIDE(_k)
#define g_vrs_hwservicemanager RS_HIDE(_l)
#endif

#if defined(VR_STR_ENCRYPT)

noused static char g_vrs_su[] = {"\xE4\xEE"};
noused static char g_vrs_adbd[] = {"\xC0\xCC\xC2\xCA"};
#if defined(VR_RULE_USE_STR_CLASS)
noused static char g_vrs_dir[] = {"\xCA\xD6\xE2"};
noused static char g_vrs_file[] = {"\xCE\xD6\xDE\xC8"};
noused static char g_vrs_process[] = {"\xE2\xE0\xD8\xC4\xCE\xE0\xE4"};
#endif
noused static char g_vrs_su_context[] = {"\xE8\x70\xE2\x76\xE2\xEC\x76\xE2\x66"};
noused static char g_vrs_adbd_context[] = {"\xE8\x70\xE2\x76\xC6\xCE\xC6\xCC\x72\xE4\x64"};
#if defined(VR_RULE_USE_STR_PERM)
noused static char g_vrs_setcurrent[] = {"\xE4\xCE\xEE\xC4\xEE\xE2\xE6\xCE\xDA\xEA"};
noused static char g_vrs_dyntransition[] = {"\xCA\xF6\xDA\xEA\xE0\xC4\xDE\xE2\xD4\xEA\xD6\xD8\xDE"};
#endif
#if !defined(MTK_PLATFORM)
noused static char g_vrs_system_data_file[] = {"\xE4\xF6\xE0\xEA\xCE\xDC\xBC\xCC\xC4\xEA\xC6\xB8\xCE\xD6\xDE\xC8"};
#else
noused static char g_vrs_mdlog_data_file[] = {"\xD8\xCC\xDE\xDC\xCA\xB8\xCA\xC6\xEE\xC0\xBA\xCA\xD0\xDC\xCC"};
#endif
#if defined(VR_RULE_USE_STR_PERM)
noused static char g_vrs_adbd_dir_perms[] = {"\xE6\xCE\xC4\xCA\x5C\xE8\xE6\xD6\xEE\xC8\x5C\xC8\xC8\xEC\xC4\xEA\xEC\xE2\x5A\xDA\xE6\xC8\xD8\x5E\xC0\xCC\xCE\xBC\xD8\xC4\xD8\xCE\x5E\xE6\xCE\xDC\xDC\xE8\xCC\xBC\xD8\xC4\xD8\xCE\x5E\xE4\xCE\xC4\xE6\xC2\xD6"};
noused static char g_vrs_adbd_file_perms[] = {"\xE6\xCE\xC4\xCA\x5C\xE8\xE6\xD6\xEE\xC8\x5C\xC0\xE6\xCE\xC4\xEA\xCE\x5E\xCC\xCE\xEE\xC0\xEC\xEE\xE6\x5C\xD8\xE2\xCE\xDA\x5A\xEE\xDA\xDA\xD6\xDA\xD4"};
#endif

noused static char g_vrs_shell[] = {"\xE4\xD4\xCC\xDA\xDC"};
noused static char g_vrs_dumpstate[] = {"\xCA\xEE\xDC\xE2\xE2\xEE\xC0\xEC\xCC"};

noused static char g_vrs_init_context[] = {"\xE8\x70\xE2\x76\xD6\xDA\xD0\xEC\x72\xE4\x64"};
noused static char g_vrs_system_file_context[] = {"\xE8\x70\xD8\xC6\xD0\xCC\xC4\xEC\xB8\xE6\x70\xE0\xF0\xE2\xEE\xC8\xDE\xB8\xCE\xD6\xDE\xC8\x70\xE0\x62"};

noused static char g_vrs_domain[] = {"\xCA\xDA\xDC\xC0\xD6\xDA"};
noused static char g_vrs_binder[] = {"\xC6\xD6\xDA\xCA\xCE\xE2"};
noused static char g_vrs_init[] = {"\xD0\xD8\xD4\xEA"};
noused static char g_vrs_coredump_file[] = {"\xC4\xDA\xE2\xC8\xCC\xEC\xD8\xE4\xB8\xCE\xD6\xDE\xC8"};

#if defined(VR_RULE_USE_STR_PERM)
noused static char g_vrs_domain_binder_perms[] = {"\xC4\xC6\xDE\xDA\x5C\xEE\xE6\xC6\xDA\xE4\xC8\xCC\xE6"};
noused static char g_vrs_domain_coredump_file_perms[] = {"\xC4\xE0\xCC\xC0\xEC\xCC\x5A\xE0\xCC\xDE\xC6\xDC\xC8\x5C\xE0\xC8\xEC\xC4\xEA\xEC\xE2\x5A\xEE\xDA\xDA\xD6\xDA\xD4\x5C\xC8\xC8\xEC\xC4\xEA\xEC\xE2\x5A\xDA\xE6\xC8\xD8\x5E\xE6\xCE\xC4\xCA\x5C\xD4\xDC\xC2\xEE\xDA\x5C\xDE\xDC\xC2\xD0\x5A\xC6\xE6\xE2\xCE\xDA\xCA\x5C\xE8\xE6\xD6\xEE\xC8"};
#endif

noused static char g_vrs_incident[] = {"\xD0\xD8\xC0\xD0\xCC\xCC\xDE\xEC"};
noused static char g_vrs_vndservicemanager[] = {"\xEE\xD8\xCE\xE4\xCE\xE2\xEE\xD6\xC0\xC8\xDE\xC4\xDE\xC6\xC8\xC8\xE0"};

noused static char g_vrs_coredomain[] = {"\xC4\xDA\xE2\xC8\xCC\xD8\xD8\xC6\xD4\xDE"};
noused static char g_vrs_Appdomain[] = {"\xC0\xE4\xE6\xCA\xDA\xDC\xC0\xD6\xDA"};
noused static char g_vrs_Mlstrustedsubject[] = {"\xD8\xDC\xE0\xEA\xE0\xEC\xE4\xEC\xCC\xCA\xE2\xEC\xC6\xD0\xCC\xC4\xEC"};
noused static char g_vrs_Netdomain[] = {"\xDE\xCE\xEE\xCA\xDA\xDC\xC0\xD6\xDA"};

noused static char g_vrs_hwservicemanager[] = {"\xD2\xEA\xE0\xC8\xE0\xEA\xD0\xC2\xCC\xD8\xC6\xDA\xC0\xCA\xCC\xE6"};

#else

noused static const char g_vrs_su[] = {"su"};
noused static const char g_vrs_adbd[] = {"adbd"};
#if defined(VR_RULE_USE_STR_CLASS)
noused static const char g_vrs_dir[] = {"dir"};
noused static const char g_vrs_file[] = {"file"};
noused static const char g_vrs_process[] = {"process"};
#endif
noused static const char g_vrs_su_context[] = {"u:r:su:s0"};
noused static const char g_vrs_adbd_context[] = {"u:r:adbd:s0"};
#if defined(VR_RULE_USE_STR_PERM)
noused static const char g_vrs_setcurrent[] = {"setcurrent"};
noused static const char g_vrs_dyntransition[] = {"dyntransition"};
#endif
#if !defined(MTK_PLATFORM)
noused static const char g_vrs_system_data_file[] = {"system_data_file"};
#else
noused static const char g_vrs_mdlog_data_file[] = {"mdlog_data_file"};
#endif

#if defined(VR_RULE_USE_STR_PERM)
noused static const char g_vrs_adbd_dir_perms[] = {"read,write,getattr,open,add_name,remove_name,search"};
noused static const char g_vrs_adbd_file_perms[] = {"read,write,create,getattr,open,unlink"};
#endif

noused static const char g_vrs_shell[] = {"shell"};
noused static const char g_vrs_dumpstate[] = {"dumpstate"};

noused static const char g_vrs_init_context[] = {"u:r:init:s0"};
noused static const char g_vrs_system_file_context[] = {"u:object_r:system_file:s0"};

noused static const char g_vrs_domain[] = {"domain"};
noused static const char g_vrs_binder[] = {"binder"};
noused static const char g_vrs_init[] = {"init"};
noused static const char g_vrs_coredump_file[] = {"coredump_file"};

#if defined(VR_RULE_USE_STR_PERM)
noused static const char g_vrs_domain_binder_perms[] = {"call,transfer"};
noused static const char g_vrs_domain_coredump_file_perms[] = {"create,rename,setattr,unlink,getattr,open,read,ioctl,lock,append,write"};
#endif

noused static const char g_vrs_incident[] = {"incident"};
noused static const char g_vrs_vndservicemanager[] = {"vndservicemanager"};

noused static const char g_vrs_coredomain[] = {"coredomain"};
noused static const char g_vrs_Appdomain[] = {"appdomain"};
noused static const char g_vrs_Mlstrustedsubject[] = {"mlstrustedsubject"};
noused static const char g_vrs_Netdomain[] = {"netdomain"};

noused static const char g_vrs_hwservicemanager[] = {"hwservicemanager"};

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_strings RS_HIDE(_11)
#endif

noused static
#if !defined(VR_STR_ENCRYPT)
const
#endif
char *g_vr_strings[vrseInvalid] = {
	g_vrs_su,
	g_vrs_adbd,
#if defined(VR_RULE_USE_STR_CLASS)
	g_vrs_dir,
	g_vrs_file,
	g_vrs_process,
#endif
	g_vrs_su_context,
	g_vrs_adbd_context,
#if defined(VR_RULE_USE_STR_PERM)
	g_vrs_setcurrent,
	g_vrs_dyntransition,
#endif
#if !defined(MTK_PLATFORM)
	g_vrs_system_data_file,
#else
	g_vrs_mdlog_data_file,
#endif
#if defined(VR_RULE_USE_STR_PERM)
	g_vrs_adbd_dir_perms,
	g_vrs_adbd_file_perms,
#endif

	g_vrs_shell,
	g_vrs_dumpstate,

	g_vrs_init_context,
	g_vrs_system_file_context,

	g_vrs_domain,
	g_vrs_binder,
	g_vrs_init,
	g_vrs_coredump_file,

#if defined(VR_RULE_USE_STR_PERM)
	g_vrs_domain_binder_perms,
	g_vrs_domain_coredump_file_perms,
#endif
	g_vrs_incident,
	g_vrs_vndservicemanager,

	g_vrs_coredomain,
	g_vrs_Appdomain,
	g_vrs_Mlstrustedsubject,
	g_vrs_Netdomain,

	g_vrs_hwservicemanager,
};

#if defined(VR_STR_ENCRYPT)

#define VR_STR_ENCRYPT_KEY_LEN (3)
#define VR_STR_ENCRYPT_SHIFT_BIT (1)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_str_encrypt RS_HIDE(_21)
#endif
noused notrace static void vr_str_encrypt(char *str)
{
	if (str) {
		unsigned int i = 0;
		unsigned char ch, tmp_ch;

		while ((ch = *str)) {
			if (++i >= (VR_STR_ENCRYPT_KEY_LEN + 1))
				i = 1;

			tmp_ch = ch ^ i;
			if (!tmp_ch)
				tmp_ch = ch;

			ch = ((tmp_ch << VR_STR_ENCRYPT_SHIFT_BIT) | (tmp_ch >> (8 - VR_STR_ENCRYPT_SHIFT_BIT)));

			*str++ = ch;
		}
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_str_decrypt RS_HIDE(_22)
#endif
noused notrace static void vr_str_decrypt(char *str)
{
	if (str) {
		unsigned int i = 0;
		unsigned char ch, tmp_ch;

		while ((ch = *str)) {
			ch = ((ch >> VR_STR_ENCRYPT_SHIFT_BIT) | (ch << (8 - VR_STR_ENCRYPT_SHIFT_BIT)));
			if (++i >= (VR_STR_ENCRYPT_KEY_LEN + 1))
				i = 1;

			tmp_ch = ch ^ i;
			if (!tmp_ch)
				tmp_ch = ch;

			*str++ = tmp_ch;
		}
	}
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_init_strings RS_HIDE(_23)
#endif
noused notrace static void vr_init_strings(void)
{
#if defined(VR_STR_ENCRYPT)
	static int g_vr_strs_inited;
	unsigned int i;

	if (g_vr_strs_inited)
		return;

	g_vr_strs_inited = 1;

	for (i = 0; i < ARRAY_SIZE(g_vr_strings); i++)
		vr_str_decrypt(g_vr_strings[i]);
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_policydb_is_type_permissive RS_HIDE(_23_0)
#endif
noused notrace static int vr_policydb_is_type_permissive(struct policydb *policydb_ptr,
	const char *type_name)
{
	int err = 0;
	struct type_datum *type;

	type = hashtab_search(policydb_ptr->p_types.table, type_name);
	if (type == NULL) {
		VR_LOGE("Type %s doesn't exist\n", type_name);

		err = -ENOENT;
		goto out;
	}

	err = ebitmap_get_bit(&policydb_ptr->permissive_map, type->value);
out:

	return err;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_policydb_is_su_permissive RS_HIDE(_23_1)
#endif
noused notrace static int vr_policydb_is_su_permissive(struct policydb *policydb_ptr)
{
	return vr_policydb_is_type_permissive(policydb_ptr, g_vr_strings[vrseSu]);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_policydb_update_permissive RS_HIDE(_24)
#endif
noused notrace static int vr_policydb_update_permissive(struct policydb *policydb_ptr,
	const char *permissive, int is_clear, int *old_permissive, int *updated_ptr)
{
	int err = 0;
	int old_value, new_value;
	int updated = 0;
	struct type_datum *type;

	type = hashtab_search(policydb_ptr->p_types.table, permissive);
	if (type == NULL) {
		if (old_permissive)
			*old_permissive = 0;

		err = -ENOENT;
		VR_LOG("Type %s doesn't exist\n", permissive);
		goto out;
	}

	old_value = ebitmap_get_bit(&policydb_ptr->permissive_map, type->value);

	if (is_clear) {
		if (old_permissive)
			new_value = *old_permissive;
		else
			new_value = 0;

		VR_LOG("clr,%d\n", new_value);
	} else {
		new_value = 1;
		VR_LOG("set,%d\n", new_value);
	}

	if (old_value == new_value) {
		VR_LOG("skip patch\n");
		goto out;
	}

	if (!is_clear) {
		if (old_permissive)
			*old_permissive = old_value;
	}

	err = ebitmap_set_bit(&policydb_ptr->permissive_map,
		type->value, new_value);
	if (err) {
		VR_LOG("Can't set bit in permissive map\n");
		goto out;
	}

	updated = 1;

out:
	if (updated_ptr)
		*updated_ptr |= updated;

	if (err > 0)
		err = -err;

	return err;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_policydb_update_permissive_by_sid RS_HIDE(_25)
#endif
noused notrace static int vr_policydb_update_permissive_by_sid(struct selinux_state *state,
	struct policydb *policydb_ptr, u32 sid, int is_clear, int *old_permissive, int *updated_ptr)
{
	int err = 0;
	int old_value, new_value;
	int updated = 0;
	struct context *scontext = NULL;

	if (!state->initialized) {
		if (old_permissive)
			*old_permissive = 0;

		err = -EINVAL;
		VR_LOG("Not initialized\n");
		goto out;
	}

	/*read_lock(&state->ss->policy_rwlock);*/
	scontext = vr_sidtab_search(state, sid);
	if (!scontext) {
		if (old_permissive)
			*old_permissive = 0;

		err = -EINVAL;
		VR_LOG("Can't get context by sid\n");
		goto out;
	}

	old_value = ebitmap_get_bit(&policydb_ptr->permissive_map, scontext->type);

	if (is_clear) {
		if (old_permissive)
			new_value = *old_permissive;
		else
			new_value = 0;
	} else {
		new_value = 1;
	}

	if (old_value == new_value)
		goto out;

	if (!is_clear) {
		if (old_permissive)
			*old_permissive = old_value;
	}

	err = ebitmap_set_bit(&policydb_ptr->permissive_map,
		scontext->type, new_value);
	if (err) {
		VR_LOG("Can't set bit in permissive map\n");
		goto out;
	}

	updated = 1;

out:
	/*read_unlock(&state->ss->policy_rwlock);*/

	if (updated_ptr)
		*updated_ptr |= updated;

	if (err > 0)
		err = -err;

	return err;
}

/*#if defined(VR_RULE_USE_STR_PERM)*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define strtok_r_rdonly RS_HIDE(_26)
#endif
noused notrace static const char *strtok_r_rdonly(const char *s,
	const char *delim, const char **last)
{
	const char *spanp;
	int c, sc;
	const char *tok;

	if (s == NULL) {
		s = *last;
		if (s == NULL)
			return NULL;
	}
	/*
	 * Skip (span) leading delimiters (s += strspn(s, delim), sort of).
	 */
cont:
	c = *s++;
	for (spanp = (char *)delim; (sc = *spanp++) != 0;) {
		if (c == sc)
			goto cont;
	}

	if (c == 0) {		/* no non-delimiter characters */
		*last = NULL;
		return NULL;
	}
	tok = s - 1;

	/*
	 * Scan token (scan for delimiters: s += strcspn(s, delim), sort of).
	 * Note that delim must have one NUL; we stop if we see that, too.
	 */
	for (;;) {
		c = *s++;
		spanp = (char *)delim;
		do {
			sc = *spanp++;
			if (sc == c) {
				if (c == 0)
					s = NULL;
				/*else
					s[-1] = 0;
				*/
				*last = s;
				return tok;
			}
		} while (sc != 0);
	}
	/* NOTREACHED */
}
/*#endif*/


#if defined(VR_USE_MAPPING)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_unmap_class RS_HIDE(_27)
#endif
noused notrace static u16 vr_unmap_class(struct selinux_map *map, u16 tclass)
{
	struct selinux_mapping *mapping;

	if (map) {
		mapping = map->mapping;
		if ((mapping) && (tclass < map->size))
			return mapping[tclass].value;
	}

	return tclass;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_unmap_perms RS_HIDE(_28)
#endif
noused notrace static u32 vr_unmap_perms(struct selinux_map *map, u16 tclass, u32 perms)
{
	struct selinux_mapping *mapping;

	if (map) {
		mapping = map->mapping;
		if ((mapping) && (tclass < map->size)) {
			unsigned int i, n = mapping[tclass].num_perms;
			u32 result;

			if (n > ARRAY_SIZE(mapping[0].perms))
				n = ARRAY_SIZE(mapping[0].perms);

			for (i = 0, result = 0; i < n; i++) {
				if (perms & (1 << i))
					result |= mapping[tclass].perms[i];
			}

			return result;
		} else {
			return 0;
		}
	} else {
		return 0;
	}
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_unmap_class_no_map_ex RS_HIDE(_29)
#endif
noused notrace static struct class_datum *vr_unmap_class_no_map_ex(
	struct policydb *policydb_ptr, u16 tclass, u32 *class_value_ptr)
{
	struct class_datum *cls = NULL;
	u32 class_value = 0;
	u32 act_tclass;
	const char *class_name;

	if (tclass <= 0) {
		VR_LOG("Invalid class index %d\n", tclass);
		goto out;
	}

	act_tclass = tclass - 1;

	class_name = secclass_map[act_tclass].name;
	if ((!class_name) || (!class_name[0])) {
		VR_LOG("Invalid class index %d\n", tclass);
		goto out;
	}

	class_value = string_to_security_class(policydb_ptr, class_name);

	if (!class_value || class_value > policydb_ptr->p_classes.nprim) {
		VR_LOG("class [%u]:%s does not exist\n", tclass, class_name);
		goto out;
	}

	cls = policydb_ptr->class_val_to_struct[class_value - 1];

out:
	if (class_value_ptr)
		*class_value_ptr = class_value;

	return cls;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_unmap_perms_no_map_ex RS_HIDE(_2a)
#endif
noused notrace static u32 vr_unmap_perms_no_map_ex(struct policydb *policydb_ptr,
	u16 tclass, u32 perms, u32 *class_value_ptr)
{
#define INLINE_STRING_TO_AV_PERM

	u32 result = 0;
	u32 class_value = 0;
	u32 act_tclass;
	const char *class_name;
	const char **class_perms;
	size_t i, n;
#if defined(INLINE_STRING_TO_AV_PERM)
	struct class_datum *cladatum;
	struct perm_datum *perdatum;
	struct common_datum *comdatum;
	struct hashtab *class_perms_table;
	struct hashtab *common_perms_table;
#endif

	if (tclass <= 0) {
		VR_LOG("Invalid class index %d\n", tclass);
		goto out;
	}

	act_tclass = tclass - 1;

	class_name = secclass_map[act_tclass].name;
	if ((!class_name) || (!class_name[0])) {
		VR_LOG("Invalid class index %d\n", tclass);
		goto out;
	}

	class_value = string_to_security_class(policydb_ptr, class_name);
	if ((!class_value) || (class_value > policydb_ptr->p_classes.nprim)) {
		VR_LOG("Class %s not defined in policy\n", class_name);
		goto out;
	}

#if defined(INLINE_STRING_TO_AV_PERM)
	cladatum = policydb_ptr->class_val_to_struct[class_value - 1];
	if (!cladatum) {
		VR_LOG("Class %s data is NULLL\n", class_name);
		goto out;
	}

	class_perms_table = cladatum->permissions.table;

	comdatum = cladatum->comdatum;
	if (comdatum)
		common_perms_table = comdatum->permissions.table;
	else
		common_perms_table = NULL;
#endif

	class_perms = secclass_map[act_tclass].perms;

	n = ARRAY_SIZE(secclass_map[act_tclass].perms) - 1;

	for (i = 0; i < n; i++) {
		const char *perm_desc;

		if (perms & (1 << i)) {
			perm_desc = class_perms[i];
			if (perm_desc) {
			#if defined(INLINE_STRING_TO_AV_PERM)
				if (common_perms_table)
					perdatum = hashtab_search(common_perms_table, perm_desc);
				else
					perdatum = NULL;

				if (!perdatum)
					perdatum = hashtab_search(class_perms_table, perm_desc);

				if (!perdatum)
					continue;

				result |= (1U << (perdatum->value - 1));
			#else
				result |= string_to_av_perm(policydb_ptr, class_value, perm_desc);
			#endif
			}
		}
	}
out:
	if (class_value_ptr)
		*class_value_ptr = class_value;

	return result;
}

#if defined(VR_RULE_USE_STR_CLASS)
	typedef const char *class_type;
#else
	typedef u32 class_type;
#endif

#if defined(VR_RULE_USE_STR_PERM)
	typedef const char *permissions_type;
#else
	typedef u32 permissions_type;
#endif

/*extern int avtab_insert(struct avtab *h, struct avtab_key *key, struct avtab_datum *datum);*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_policydb_update_rule RS_HIDE(_2b)
#endif
noused notrace static int vr_policydb_update_rule(struct policydb *policydb_ptr,
	const char *source, const char *target,
	class_type _class, permissions_type permissions,
#if defined(VR_USE_MAPPING)
	struct selinux_map *map,
#endif
	u32 rule_type, int is_clear, u32 *old_permissions, int *updated_ptr)
{
	int ret = 0;
	int updated = 0;
	struct type_datum *src, *tgt;
#if defined(VR_RULE_USE_STR_CLASS)
#if defined(VR_FIND_RULE_USE_SYS_ROUTINE)
	u32 tclass;
#else
	struct class_datum *cls;
#endif
#else
	u32 unmapped_class;
	struct class_datum *cls;
#endif
	struct avtab_datum *av;
	struct avtab_key key;
#if defined(VR_RULE_USE_STR_PERM)
	char token_buffer[32];
	u32 perms_value = 0;
	const char *perm_token = NULL;
	const char *perm_saveptr = NULL;

	#define PERMS_VALUE perms_value

	perm_token = strtok_r_rdonly(permissions, ",", &perm_saveptr);
	if (!perm_token) {
		ret = EINVAL;
		VR_LOG("permissions is empty\n");
		goto out;
	}
#else
	#define PERMS_VALUE permissions

	if (!permissions) {
		ret = EINVAL;
		VR_LOG("permissions is empty\n");
		goto out;
	}
#endif

#if !defined(VR_RULE_USE_STR_CLASS)
#if defined(VR_USE_MAPPING)
	unmapped_class = vr_unmap_class(map, _class);
	if (!unmapped_class || unmapped_class > policydb_ptr->p_classes.nprim) {
		ret = EINVAL;
		VR_LOG("class %u does not exist\n", _class);
		goto out;
	}

	cls = policydb_ptr->class_val_to_struct[unmapped_class - 1];
#else
	cls = vr_unmap_class_no_map_ex(policydb_ptr, _class, &unmapped_class);
	if (cls == NULL) {
		ret = EINVAL;
		VR_LOG("class %u does not exist\n", _class);
		goto out;
	}
#endif

	#define CLASS_VALUE cls->value
#endif

	src = hashtab_search(policydb_ptr->p_types.table, source);
	if (src == NULL) {
		ret = EINVAL;
		VR_LOG("source type %s does not exist\n", source);
		goto out;
	}

	tgt = hashtab_search(policydb_ptr->p_types.table, target);
	if (tgt == NULL) {
		ret = EINVAL;
		VR_LOG("target type %s does not exist\n", target);
		goto out;
	}

#if defined(VR_RULE_USE_STR_CLASS)
#if defined(VR_FIND_RULE_USE_SYS_ROUTINE)
	tclass = string_to_security_class(policydb_ptr, _class);
	if (!tclass || tclass > policydb_ptr->p_classes.nprim) {
		ret = EINVAL;
		VR_LOG("class %u does not exist\n", _class);
		goto out;
	}

	#define CLASS_VALUE tclass
#else
	cls = hashtab_search(policydb_ptr->p_classes.table, _class);
	if (cls == NULL) {
		ret = EINVAL;
		VR_LOG("class %s does not exist\n", _class);
		goto out;
	}

	#define CLASS_VALUE cls->value
#endif
#endif

#if defined(VR_RULE_USE_STR_PERM)
	while (perm_token) {
		int len;

		if (perm_saveptr)
			len = perm_saveptr - 1 - perm_token;
		else
			len = strlen(perm_token);

		if (len < (int)sizeof(token_buffer)) {
			u32 v;
		#if !defined(VR_FIND_RULE_USE_SYS_ROUTINE)
			struct perm_datum *perm;
		#endif

			memcpy(token_buffer, perm_token, len);
			token_buffer[len] = '\0';

		#if defined(VR_FIND_RULE_USE_SYS_ROUTINE)
			v = string_to_av_perm(policydb_ptr, tclass, token_buffer);
		#else
			perm = hashtab_search(cls->permissions.table, token_buffer);
			if (perm == NULL) {
				if (cls->comdatum == NULL) {
				#if defined(VR_RULE_USE_STR_CLASS)
					VR_LOG("perm %s does not exist in class %s\n", token_buffer, _class);
				#else
					VR_LOG("perm %s does not exist in class %x\n", token_buffer, _class);
				#endif

					v = 0;
				} else {
					perm = hashtab_search(cls->comdatum->permissions.table, token_buffer);
					if (perm == NULL) {
					#if defined(VR_RULE_USE_STR_CLASS)
						VR_LOG("perm %s does not exist in class %s\n", token_buffer, _class);
					#else
						VR_LOG("perm %s does not exist in class %x\n", token_buffer, _class);
					#endif

						v = 0;
					} else {
						v = (1U << (perm->value - 1));
					}
				}
			} else {
				v = (1U << (perm->value - 1));
			}
		#endif

			perms_value |= v;
		}

		/*if ((perm_saveptr) && (!perm_saveptr[-1]))
			perm_saveptr[-1] = ',';
		*/

		perm_token = strtok_r_rdonly(NULL, ",", &perm_saveptr);
	}

	if (perms_value)
#else
#if defined(VR_USE_MAPPING)
	PERMS_VALUE = vr_unmap_perms(map, _class, PERMS_VALUE);
#else
	PERMS_VALUE = vr_unmap_perms_no_map_ex(policydb_ptr, _class, PERMS_VALUE, NULL);
#endif
#endif
	{
		/* See if there is already a rule */
		key.source_type = src->value;
		key.target_type = tgt->value;
		key.target_class = CLASS_VALUE;
#if 0
#if defined(VR_RULE_USE_STR_CLASS)
	#if defined(VR_FIND_RULE_USE_SYS_ROUTINE)
		key.target_class = tclass;
	#else
		key.target_class = cls->value;
	#endif
#else
		key.target_class = _class;
#endif
#endif
		key.specified = rule_type; /*AVTAB_ALLOWED/AVTAB_AUDITDENY/AVTAB_AUDITALLOW/AVTAB_TRANSITION/AVTAB_CHANGE/AVTAB_MEMBER*/

	#if defined(VR_RULE_USE_STR_CLASS)
		/*printk("perm:2,%s,%s,%s,%x\n", source, target, _class, PERMS_VALUE);*/
	#else
		/*printk("perm:2,%s,%s,%x,%x\n", source, target, _class, PERMS_VALUE);*/
	#endif

		av = avtab_search(&policydb_ptr->te_avtab, &key);

		if (av == NULL) {
			struct avtab_datum local_av;
			struct avtab_node *tmp_node;

			if (old_permissions)
				*old_permissions = 0;

			if (is_clear)
				local_av.AVD_DATA = 0xffffffff & ~PERMS_VALUE;
			else
				local_av.AVD_DATA = PERMS_VALUE;

			/*ret = avtab_insert(&policydb_ptr->te_avtab, &key, &local_av);
			if (ret)*/
			tmp_node = avtab_insert_nonunique(&policydb_ptr->te_avtab, &key, &local_av);
			if (!tmp_node) {
				ret = -ENOMEM;
				VR_LOG("Error inserting into avtab\n");
				goto out;
			} else if (IS_ERR(tmp_node)) {
				ret = PTR_ERR(tmp_node);
				VR_LOG("Error inserting into avtab: %d\n", ret);
				goto out;
			}

			updated = 1;
		} else {
			if (is_clear) {
				if (av->AVD_DATA & PERMS_VALUE) {
					u32 old_perms;

					if (old_permissions) {
						old_perms = *old_permissions;
						if (old_perms)
							av->AVD_DATA = old_perms;
						else
							av->AVD_DATA &= ~PERMS_VALUE;
					} else {
						av->AVD_DATA &= ~PERMS_VALUE;
					}

					updated = 1;
				}
			} else {
				if ((av->AVD_DATA & PERMS_VALUE) != PERMS_VALUE) {
					if (old_permissions)
						*old_permissions = av->AVD_DATA;

					av->AVD_DATA |= PERMS_VALUE;
					updated = 1;
				}
			}
		}
	}
#if defined(VR_RULE_USE_STR_PERM)
	else
		ret = EINVAL;
#endif

out:
	if (updated_ptr)
		*updated_ptr |= updated;

	if (ret > 0)
		ret = -ret;

	return ret;

	#undef CLASS_VALUE
	#undef PERMS_VALUE
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_policydb_update_rule_str RS_HIDE(_2b_0)
#endif
noused notrace static int vr_policydb_update_rule_str(struct policydb *policydb_ptr,
	const char *source, const char *target,
	const char *_class, const char *permissions,
#if defined(VR_USE_MAPPING)
	struct selinux_map *map,
#endif
	u32 rule_type, int is_clear, u32 *old_permissions, int *updated_ptr)
{
	int ret = 0;
	int updated = 0;
	struct type_datum *src, *tgt;
#if defined(VR_FIND_RULE_USE_SYS_ROUTINE)
	u32 tclass;
#else
	struct class_datum *cls;
#endif
	struct avtab_datum *av;
	struct avtab_key key;
	char token_buffer[32];
	u32 perms_value = 0;
	const char *perm_token = NULL;
	const char *perm_saveptr = NULL;

	#define PERMS_VALUE perms_value

	perm_token = strtok_r_rdonly(permissions, ",", &perm_saveptr);
	if (!perm_token) {
		ret = EINVAL;
		VR_LOG("permissions is empty\n");
		goto out;
	}

	src = hashtab_search(policydb_ptr->p_types.table, source);
	if (src == NULL) {
		ret = EINVAL;
		VR_LOG("source type %s does not exist\n", source);
		goto out;
	}

	tgt = hashtab_search(policydb_ptr->p_types.table, target);
	if (tgt == NULL) {
		ret = EINVAL;
		VR_LOG("target type %s does not exist\n", target);
		goto out;
	}

#if defined(VR_FIND_RULE_USE_SYS_ROUTINE)
	tclass = string_to_security_class(policydb_ptr, _class);
	if (!tclass || tclass > policydb_ptr->p_classes.nprim) {
		ret = EINVAL;
		VR_LOG("class %u does not exist\n", _class);
		goto out;
	}

	#define CLASS_VALUE tclass
#else
	cls = hashtab_search(policydb_ptr->p_classes.table, _class);
	if (cls == NULL) {
		ret = EINVAL;
		VR_LOG("class %s does not exist\n", _class);
		goto out;
	}

	#define CLASS_VALUE cls->value
#endif

	while (perm_token) {
		int len;

		if (perm_saveptr)
			len = perm_saveptr - 1 - perm_token;
		else
			len = strlen(perm_token);

		if (len < (int)sizeof(token_buffer)) {
			u32 v;
		#if !defined(VR_FIND_RULE_USE_SYS_ROUTINE)
			struct perm_datum *perm;
		#endif

			memcpy(token_buffer, perm_token, len);
			token_buffer[len] = '\0';

		#if defined(VR_FIND_RULE_USE_SYS_ROUTINE)
			v = string_to_av_perm(policydb_ptr, tclass, token_buffer);
		#else
			perm = hashtab_search(cls->permissions.table, token_buffer);
			if (perm == NULL) {
				if (cls->comdatum == NULL) {
					VR_LOG("perm %s does not exist in class %s\n", token_buffer, _class);

					v = 0;
				} else {
					perm = hashtab_search(cls->comdatum->permissions.table, token_buffer);
					if (perm == NULL) {
						VR_LOG("perm %s does not exist in class %s\n", token_buffer, _class);

						v = 0;
					} else {
						v = (1U << (perm->value - 1));
					}
				}
			} else {
				v = (1U << (perm->value - 1));
			}
		#endif

			perms_value |= v;
		}

		/*if ((perm_saveptr) && (!perm_saveptr[-1]))
			perm_saveptr[-1] = ',';
		*/

		perm_token = strtok_r_rdonly(NULL, ",", &perm_saveptr);
	}

	if (perms_value) {
		/* See if there is already a rule */
		key.source_type = src->value;
		key.target_type = tgt->value;
		key.target_class = CLASS_VALUE;
		key.specified = rule_type; /*AVTAB_ALLOWED/AVTAB_AUDITDENY/AVTAB_AUDITALLOW/AVTAB_TRANSITION/AVTAB_CHANGE/AVTAB_MEMBER*/

		/*printk("perm:2,%s,%s,%s,%x\n", source, target, _class, PERMS_VALUE);*/

		av = avtab_search(&policydb_ptr->te_avtab, &key);

		if (av == NULL) {
			struct avtab_datum local_av;
			struct avtab_node *tmp_node;

			if (old_permissions)
				*old_permissions = 0;

			if (is_clear)
				local_av.AVD_DATA = 0xffffffff & ~PERMS_VALUE;
			else
				local_av.AVD_DATA = PERMS_VALUE;

			/*ret = avtab_insert(&policydb_ptr->te_avtab, &key, &local_av);
			if (ret)*/
			tmp_node = avtab_insert_nonunique(&policydb_ptr->te_avtab, &key, &local_av);
			if (!tmp_node) {
				ret = -ENOMEM;
				VR_LOG("Error inserting into avtab\n");
				goto out;
			} else if (IS_ERR(tmp_node)) {
				ret = PTR_ERR(tmp_node);
				VR_LOG("Error inserting into avtab: %d\n", ret);
				goto out;
			}

			updated = 1;
		} else {
			if (is_clear) {
				if (av->AVD_DATA & PERMS_VALUE) {
					u32 old_perms;

					if (old_permissions) {
						old_perms = *old_permissions;
						if (old_perms)
							av->AVD_DATA = old_perms;
						else
							av->AVD_DATA &= ~PERMS_VALUE;
					} else {
						av->AVD_DATA &= ~PERMS_VALUE;
					}

					updated = 1;
				}
			} else {
				if ((av->AVD_DATA & PERMS_VALUE) != PERMS_VALUE) {
					if (old_permissions)
						*old_permissions = av->AVD_DATA;

					av->AVD_DATA |= PERMS_VALUE;
					updated = 1;
				}
			}
		}
	} else
		ret = EINVAL;

out:
	if (updated_ptr)
		*updated_ptr |= updated;

	if (ret > 0)
		ret = -ret;

	return ret;

	#undef CLASS_VALUE
	#undef PERMS_VALUE
}

#if !defined(VR_NO_PATCH_RULES)

typedef struct tag_se_rule_struct {
	int source;
	int target;
#if defined(VR_RULE_USE_STR_CLASS)
	int _class;
#else
	u32 _class;
#endif
#if defined(VR_RULE_USE_STR_PERM)
	int permissions;
#else
	u32 permissions;
#endif
} se_rule_struct;

typedef struct tag_se_attr_rule_struct {
	int sourceAttr;
	const int *sourceMinuses;
	int target;
#if defined(VR_RULE_USE_STR_CLASS)
	int _class;
#else
	u32 _class;
#endif
#if defined(VR_RULE_USE_STR_PERM)
	int permissions;
#else
	u32 permissions;
#endif
} se_attr_rule_struct;

typedef struct tag_se_str_rule_struct {
	int source;
	int target;
	int _class_str_idx;
	int permissions_str_idx;
} se_str_rule_struct;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_adbd_allow_rules RS_HIDE(_13)
#endif
noused static const se_rule_struct g_vr_adbd_allow_rules[] = {
	{vrseAdbd, vrseAdbdFile,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseFile,
	#else
		SECCLASS_FILE,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		vrseAdbdFilePerms
	#else
		(FILE__READ | FILE__WRITE | FILE__CREATE | FILE__GETATTR | FILE__OPEN | FILE__UNLINK)
	#endif
	},

	{vrseAdbd, vrseAdbdFile,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseDir,
	#else
		SECCLASS_DIR,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		vrseAdbdDirPerms
	#else
		(DIR__READ | DIR__WRITE | DIR__GETATTR | DIR__OPEN | DIR__ADD_NAME | DIR__REMOVE_NAME | DIR__SEARCH)
	#endif
	},

	{vrseAdbd, vrseAdbd,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseProcess,
	#else
		SECCLASS_PROCESS,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		vrseSetCurrent
	#else
		PROCESS__SETCURRENT
	#endif
	},
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_adbd_allow_rules_perms RS_HIDE(_13_1)
#endif
noused static u32 g_vr_adbd_allow_rules_perms[ARRAY_SIZE(g_vr_adbd_allow_rules)];

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_test_allow_perms RS_HIDE(_14)
#endif
noused static const u32 g_test_allow_perms[] = {
	(FILE__READ | FILE__WRITE | FILE__CREATE | FILE__GETATTR | FILE__OPEN | FILE__UNLINK),
	(DIR__READ | DIR__WRITE | DIR__GETATTR | DIR__OPEN | DIR__ADD_NAME | DIR__REMOVE_NAME | DIR__SEARCH),
	PROCESS__SETCURRENT,
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_test_trigger_perm RS_HIDE(_15)
#endif
noused static const u32 g_test_trigger_perm = PROCESS__DYNTRANSITION;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_adbd_trigger_rule RS_HIDE(_16)
#endif
noused static const se_rule_struct g_vr_adbd_trigger_rule = {
	vrseAdbd, vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseProcess,
	#else
		SECCLASS_PROCESS,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		vrseDynTransition
	#else
		PROCESS__DYNTRANSITION
	#endif
		};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_adbd_trigger_rule_perm RS_HIDE(_16_1)
#endif
noused static u32 g_vr_adbd_trigger_rule_perm;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_domain_binder_minus_types RS_HIDE(_16_2)
#endif
noused static const int g_vr_domain_binder_minus_types[] = {
	vrseInit,
	0
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_domain_allow_rules RS_HIDE(_16_3)
#endif
noused static const se_attr_rule_struct g_vr_domain_allow_rules[] = {
	{vrseDomain, g_vr_domain_binder_minus_types,
		vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseBinder,
	#else
		SECCLASS_BINDER,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		vrseDomainBinderPerms
	#else
		(BINDER__CALL | BINDER__TRANSFER)
	#endif
	},

	{vrseDomain, NULL,
		vrseCoreDumpFile,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseFile,
	#else
		SECCLASS_FILE,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		vrseDomainCoreDumpFilePerms
	#else
		(FILE__CREATE | FILE__RENAME | FILE__SETATTR | FILE__UNLINK | FILE__GETATTR
		| FILE__OPEN | FILE__READ | FILE__IOCTL | FILE__LOCK | FILE__APPEND | FILE__WRITE)
	#endif
	},
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_domain_allow_rules_perms RS_HIDE(_16_4)
#endif
noused static u32 g_vr_domain_allow_rules_perms[ARRAY_SIZE(g_vr_domain_allow_rules)];

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_vndservicemanager_allow_rules RS_HIDE(_16_5)
#endif
noused static const se_rule_struct g_vr_vndservicemanager_allow_rules[] = {
	{vrseVndServiceManager, vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseDir,
	#else
		SECCLASS_DIR,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		vrseAdbdDirPerms
	#else
		(DIR__SEARCH)
	#endif
	},

	{vrseVndServiceManager, vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseFile,
	#else
		SECCLASS_FILE,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		vrseAdbdFilePerms
	#else
		(FILE__READ | FILE__OPEN)
	#endif
	},

	{vrseVndServiceManager, vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseProcess,
	#else
		SECCLASS_PROCESS,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		#error need define getattr string
	#else
		PROCESS__GETATTR
	#endif
	},

	{vrseVndServiceManager, vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseBinder,
	#else
		SECCLASS_BINDER,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		vrseDomainBinderPerms
	#else
		(BINDER__CALL | BINDER__TRANSFER)
	#endif
	},
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_vndservicemanager_allow_rules_perms RS_HIDE(_16_6)
#endif
noused static u32 g_vr_vndservicemanager_allow_rules_perms[ARRAY_SIZE(g_vr_vndservicemanager_allow_rules)];

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_hwservicemanager_allow_rules RS_HIDE(_16_7)
#endif
noused static const se_rule_struct g_vr_hwservicemanager_allow_rules[] = {
	{vrseHwServiceManager, vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseDir,
	#else
		SECCLASS_DIR,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		vrseAdbdDirPerms
	#else
		(DIR__SEARCH)
	#endif
	},

	{vrseHwServiceManager, vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseFile,
	#else
		SECCLASS_FILE,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		vrseAdbdFilePerms
	#else
		(FILE__READ | FILE__OPEN)
	#endif
	},

	{vrseHwServiceManager, vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseProcess,
	#else
		SECCLASS_PROCESS,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		#error need define getattr string
	#else
		PROCESS__GETATTR
	#endif
	},

	{vrseHwServiceManager, vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseBinder,
	#else
		SECCLASS_BINDER,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		vrseDomainBinderPerms
	#else
		(BINDER__CALL | BINDER__TRANSFER)
	#endif
	},
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_hwservicemanager_allow_rules_perms RS_HIDE(_16_8)
#endif
noused static u32 g_vr_hwservicemanager_allow_rules_perms[ARRAY_SIZE(g_vr_hwservicemanager_allow_rules)];

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_domain_allow_rules_8_x RS_HIDE(_16_9)
#endif
noused static const se_rule_struct g_vr_domain_allow_rules_8_x[] = {
	{vrseDomain, vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		#error need define fd string
	#else
		SECCLASS_FD,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		#error need define fd permissions string
	#else
		(FD__USE)
	#endif
	},

	{vrseDomain, vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		#error need define fifo_file string
	#else
		SECCLASS_FIFO_FILE,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		#error need define fifo_file permissions string
	#else
		(FIFO_FILE__OPEN | FIFO_FILE__READ | FIFO_FILE__WRITE | FIFO_FILE__GETATTR
			| FIFO_FILE__APPEND)
	#endif
	},

	{vrseDomain, vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseProcess,
	#else
		SECCLASS_PROCESS,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		#error need define process permissions string
	#else
		(PROCESS__GETATTR | PROCESS__SIGCHLD)
	#endif
	},

	{vrseDomain, vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		vrseBinder,
	#else
		SECCLASS_BINDER,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		vrseDomainBinderPerms
	#else
		(BINDER__CALL | BINDER__TRANSFER)
	#endif
	},

	{vrseDomain, vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		#error need define unix_stream_socket string
	#else
		SECCLASS_UNIX_STREAM_SOCKET,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		#error need define unix_stream_socket permissions string
	#else
		(UNIX_STREAM_SOCKET__CONNECTTO | UNIX_STREAM_SOCKET__GETATTR | UNIX_STREAM_SOCKET__GETOPT
			| UNIX_STREAM_SOCKET__READ | UNIX_STREAM_SOCKET__WRITE | UNIX_STREAM_SOCKET__SHUTDOWN)
	#endif
	},

	{vrseDomain, vrseSu,
	#if defined(VR_RULE_USE_STR_CLASS)
		#error need define unix_dgram_socket string
	#else
		SECCLASS_UNIX_DGRAM_SOCKET,
	#endif
	#if defined(VR_RULE_USE_STR_PERM)
		#error need define unix_dgram_socket permissions string
	#else
		(UNIX_DGRAM_SOCKET__SENDTO)
	#endif
	},
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_domain_allow_rules_perms_8_x RS_HIDE(_16_10)
#endif
noused static u32 g_vr_domain_allow_rules_perms_8_x[ARRAY_SIZE(g_vr_domain_allow_rules_8_x)];


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_vr_vivorooted RS_HIDE(_17)
#define g_vr_adbd_sid RS_HIDE(_18)
#define g_vr_su_sid RS_HIDE(_19)
#define g_vr_adbd_permissive RS_HIDE(_19_1)
#define g_vr_su_permissive RS_HIDE(_19_2)
#define g_vr_otherrooted RS_HIDE(_1a)
#define g_vr_other_sid RS_HIDE(_1b)
#define g_vr_other_sid_permissive RS_HIDE(_1b_1)

#define g_vr_init_sid RS_HIDE(_1c)
#define g_vr_system_file_sid RS_HIDE(_1d)

#define g_su_patched RS_HIDE(_1e)
#if !defined(VR_NO_PATCH_RULES)
#define g_adb_trigger_rule_patched RS_HIDE(_1f)
#define g_adb_allow_rules_patched RS_HIDE(_1g)
#define g_domain_allow_rules_patched RS_HIDE(_1g_1)
#endif

#define g_vr_shell_permissive RS_HIDE(_1h)
#define g_vr_dumpstate_permissive RS_HIDE(_1i)

#define g_vr_shell_permissive_patched RS_HIDE(_1j)
#define g_vr_dumpstate_permissive_patched RS_HIDE(_1k)

#define g_vr_incident_permissive RS_HIDE(_1l)
#if !defined(VR_PATCH_VNDSERVICEMANAGER_POLICY_BY_RULES)
#define g_vr_vndservicemanager_permissive RS_HIDE(_1m)
#endif

#define g_vr_incident_permissive_patched RS_HIDE(_1n)
#if defined(VR_PATCH_VNDSERVICEMANAGER_POLICY_BY_RULES)
#define g_vr_vndservicemanager_rules_patched RS_HIDE(_1o)
#else
#define g_vr_vndservicemanager_permissive_patched RS_HIDE(_1o)
#endif

#define g_vr_su_attr_coredomain RS_HIDE(_1p)
#define g_vr_su_attr_appdomain RS_HIDE(_1q)
#define g_vr_su_attr_mlstrustedsubject RS_HIDE(_1r)
#define g_vr_su_attr_netdomain RS_HIDE(_1s)

#define g_vr_su_attr_coredomain_patched RS_HIDE(_1t)
#define g_vr_su_attr_appdomain_patched RS_HIDE(_1u)
#define g_vr_su_attr_mlstrustedsubject_patched RS_HIDE(_1v)
#define g_vr_su_attr_netdomain_patched RS_HIDE(_1w)

#define g_vr_hwservicemanager_rules_patched RS_HIDE(_1x)

#define g_vr_domain_rules_8_x_patched RS_HIDE(_1y)

#endif

noused static int g_vr_vivorooted;
noused static u32 g_vr_adbd_sid;
noused static u32 g_vr_su_sid;
noused static int g_vr_adbd_permissive;
noused static int g_vr_su_permissive;

noused static int g_vr_otherrooted;
noused static u32 g_vr_other_sid;
noused static int g_vr_other_permissive;

noused static u32 g_vr_init_sid;
noused static u32 g_vr_system_file_sid;

noused static int g_su_patched;
#if !defined(VR_NO_PATCH_RULES)
noused static int g_adb_trigger_rule_patched;
noused static int g_adb_allow_rules_patched;
noused static int g_domain_allow_rules_patched;
#endif

noused static int g_vr_shell_permissive;
noused static int g_vr_dumpstate_permissive;

noused static int g_vr_shell_permissive_patched;
noused static int g_vr_dumpstate_permissive_patched;

noused static int g_vr_incident_permissive;
#if !defined(VR_PATCH_VNDSERVICEMANAGER_POLICY_BY_RULES)
noused static int g_vr_vndservicemanager_permissive;
#endif

noused static int g_vr_incident_permissive_patched;
#if defined(VR_PATCH_VNDSERVICEMANAGER_POLICY_BY_RULES)
noused static int g_vr_vndservicemanager_rules_patched;
#else
noused static int g_vr_vndservicemanager_permissive_patched;
#endif

noused static int g_vr_su_attr_coredomain;
noused static int g_vr_su_attr_appdomain;
noused static int g_vr_su_attr_mlstrustedsubject;
noused static int g_vr_su_attr_netdomain;

noused static int g_vr_su_attr_coredomain_patched;
noused static int g_vr_su_attr_appdomain_patched;
noused static int g_vr_su_attr_mlstrustedsubject_patched;
noused static int g_vr_su_attr_netdomain_patched;

noused static int g_vr_hwservicemanager_rules_patched;

noused static int g_vr_domain_rules_8_x_patched;

#if !defined(VR_NO_PATCH_RULES)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_patch_sepolicy_for_adbd_allow_rules RS_HIDE(_31)
#endif
noused notrace static int vr_patch_sepolicy_for_adbd_allow_rules(struct policydb *policydb_ptr,
#if defined(VR_USE_MAPPING)
	struct selinux_map *map,
#endif
	int *patched_ptr)
{
	int ret = 0;
	int patched = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(g_vr_adbd_allow_rules); i++) {
		const se_rule_struct *rule = &g_vr_adbd_allow_rules[i];

	#if defined(VR_USE_MAPPING)
		/*printk("perm:0,%x,%x,%x\n", g_test_allow_perms[i], vr_unmap_perms(map, rule->_class, g_test_allow_perms[i]),
			vr_unmap_perms_no_map_ex(policydb_ptr, rule->_class, g_test_allow_perms[i], NULL));
		*/
	#else
		/*printk("perm:0,%x,%x\n", g_test_allow_perms[i], vr_unmap_perms_no_map_ex(policydb_ptr, rule->_class, g_test_allow_perms[i], NULL));*/
	#endif

		ret = vr_policydb_update_rule(policydb_ptr, g_vr_strings[rule->source],
			g_vr_strings[rule->target],
		#if defined(VR_RULE_USE_STR_CLASS)
			g_vr_strings[rule->_class],
		#else
			rule->_class,
		#endif
		#if defined(VR_RULE_USE_STR_PERM)
			g_vr_strings[rule->permissions],
		#else
			rule->permissions,
		#endif
		#if defined(VR_USE_MAPPING)
			map,
		#endif
			AVTAB_ALLOWED, 0, &g_vr_adbd_allow_rules_perms[i], &patched);

		if (ret)
			break;
	}

	if (patched_ptr)
		*patched_ptr |= patched;

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_patch_sepolicy_for_adbd_allow_rules_for_read RS_HIDE(_31_)
#endif
noused notrace static int vr_patch_sepolicy_for_adbd_allow_rules_for_read(struct policydb *policydb_ptr,
#if defined(VR_USE_MAPPING)
	struct selinux_map *map,
#endif
	int *patched_ptr,
	int is_clear)
{
	int ret = 0;
	int patched = 0;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(g_vr_adbd_allow_rules); i++) {
		const se_rule_struct *rule = &g_vr_adbd_allow_rules[i];

	#if defined(VR_USE_MAPPING)
		/*printk("perm:0,%x,%x,%x\n", g_test_allow_perms[i], vr_unmap_perms(map, rule->_class, g_test_allow_perms[i]),
			vr_unmap_perms_no_map_ex(policydb_ptr, rule->_class, g_test_allow_perms[i], NULL));
		*/
	#else
		/*printk("perm:0,%x,%x\n", g_test_allow_perms[i], vr_unmap_perms_no_map_ex(policydb_ptr, rule->_class, g_test_allow_perms[i], NULL));*/
	#endif
		patched = 0;

		ret = vr_policydb_update_rule(policydb_ptr, g_vr_strings[rule->source],
			g_vr_strings[rule->target],
		#if defined(VR_RULE_USE_STR_CLASS)
			g_vr_strings[rule->_class],
		#else
			rule->_class,
		#endif
		#if defined(VR_RULE_USE_STR_PERM)
			g_vr_strings[rule->permissions],
		#else
			rule->permissions,
		#endif
		#if defined(VR_USE_MAPPING)
			map,
		#endif
			AVTAB_ALLOWED, is_clear, &g_vr_adbd_allow_rules_perms[i], &patched);

		if (patched_ptr)
			*patched_ptr |= patched;

		if ((!is_clear) && (ret))
			break;
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_patch_sepolicy_for_adbd_trigger_rule RS_HIDE(_32)
#endif
noused notrace static int vr_patch_sepolicy_for_adbd_trigger_rule(struct policydb *policydb_ptr,
#if defined(VR_USE_MAPPING)
	struct selinux_map *map,
#endif
	int *patched_ptr)
{
	int ret;
	int patched = 0;

	const se_rule_struct *rule = &g_vr_adbd_trigger_rule;

#if defined(VR_USE_MAPPING)
	/*printk("perm:1,%x,%x,%x\n", g_test_trigger_perm, vr_unmap_perms(map, SECCLASS_PROCESS, g_test_trigger_perm),
		vr_unmap_perms_no_map_ex(policydb_ptr, SECCLASS_PROCESS, g_test_trigger_perm, NULL));
	*/
#else
	/*printk("perm:1,%x,%x\n", g_test_trigger_perm, vr_unmap_perms_no_map_ex(policydb_ptr, SECCLASS_PROCESS, g_test_trigger_perm, NULL));*/
#endif

	if (g_vr_vivorooted) {
	#if !defined(VR_SET_ADBD_PERMISSIVE_AFTERWARD)
		/*allow trigger rule*/
		ret = vr_policydb_update_rule(policydb_ptr, g_vr_strings[rule->source],
			g_vr_strings[rule->target],
		#if defined(VR_RULE_USE_STR_CLASS)
			g_vr_strings[rule->_class],
		#else
			rule->_class,
		#endif
		#if defined(VR_RULE_USE_STR_PERM)
			g_vr_strings[rule->permissions],
		#else
			rule->permissions,
		#endif
		#if defined(VR_USE_MAPPING)
			map,
		#endif
			AVTAB_ALLOWED, 0, &g_vr_adbd_trigger_rule_perm, &patched);
	#else
		ret = 0;
	#endif
	} else {
		/*do not audit deny of trigger rule*/
		ret = vr_policydb_update_rule(policydb_ptr, g_vr_strings[rule->source],
			g_vr_strings[rule->target],
		#if defined(VR_RULE_USE_STR_CLASS)
			g_vr_strings[rule->_class],
		#else
			rule->_class,
		#endif
		#if defined(VR_RULE_USE_STR_PERM)
			g_vr_strings[rule->permissions],
		#else
			rule->permissions,
		#endif
		#if defined(VR_USE_MAPPING)
			map,
		#endif
			AVTAB_AUDITDENY, 1, &g_vr_adbd_trigger_rule_perm, &patched);
	}

	if (patched_ptr)
		*patched_ptr |= patched;

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_patch_sepolicy_for_adbd_trigger_rule_for_read RS_HIDE(_32_)
#endif
noused notrace static int vr_patch_sepolicy_for_adbd_trigger_rule_for_read(struct policydb *policydb_ptr,
#if defined(VR_USE_MAPPING)
	struct selinux_map *map,
#endif
	int *patched_ptr,
	int is_clear)
{
	int ret;
#if !defined(VR_SET_ADBD_PERMISSIVE_AFTERWARD)
	noused int patched = 0;
#endif
	noused const se_rule_struct *rule = &g_vr_adbd_trigger_rule;

#if defined(VR_USE_MAPPING)
	/*printk("perm:1,%x,%x,%x\n", g_test_trigger_perm, vr_unmap_perms(map, SECCLASS_PROCESS, g_test_trigger_perm),
		//vr_unmap_perms_no_map_ex(policydb_ptr, SECCLASS_PROCESS, g_test_trigger_perm, NULL));
	*/
#else
	/*printk("perm:1,%x,%x\n", g_test_trigger_perm, vr_unmap_perms_no_map_ex(policydb_ptr, SECCLASS_PROCESS, g_test_trigger_perm, NULL));*/
#endif

	/*if (g_vr_vivorooted)*/
	{
	#if !defined(VR_SET_ADBD_PERMISSIVE_AFTERWARD)
		/*allow trigger rule*/
		ret = vr_policydb_update_rule(policydb_ptr, g_vr_strings[rule->source],
			g_vr_strings[rule->target],
		#if defined(VR_RULE_USE_STR_CLASS)
			g_vr_strings[rule->_class],
		#else
			rule->_class,
		#endif
		#if defined(VR_RULE_USE_STR_PERM)
			g_vr_strings[rule->permissions],
		#else
			rule->permissions,
		#endif
		#if defined(VR_USE_MAPPING)
			map,
		#endif
			AVTAB_ALLOWED, is_clear, &patched);

		if (patched_ptr)
			*patched_ptr |= patched;
	#else
		/*if (patched_ptr) {
			*patched_ptr |= 0;
		}
		*/
		ret = 0;
	#endif
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_policydb_update_typerule RS_HIDE(_c_1)
#endif
noused notrace static int vr_policydb_update_typerule(struct policydb *policydb_ptr, const char *sourceAttribute,
	const int *source_minuses,
	const char *target, class_type _class,
	permissions_type permissions,
#if defined(VR_USE_MAPPING)
	struct selinux_map *map,
#endif
	u32 rule_type, int is_clear, int *updated_ptr);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_patch_sepolicy_for_domain_allow_rules RS_HIDE(_32_1)
#endif
noused notrace static int vr_patch_sepolicy_for_domain_allow_rules(struct policydb *policydb_ptr,
#if defined(VR_USE_MAPPING)
	struct selinux_map *map,
#endif
	int *patched_ptr)
{
	int ret = 0;
	int patched = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(g_vr_domain_allow_rules); i++) {
		const se_attr_rule_struct *rule = &g_vr_domain_allow_rules[i];

		ret = vr_policydb_update_typerule(policydb_ptr, g_vr_strings[rule->sourceAttr],
			rule->sourceMinuses,
			g_vr_strings[rule->target],
		#if defined(VR_RULE_USE_STR_CLASS)
			g_vr_strings[rule->_class],
		#else
			rule->_class,
		#endif
		#if defined(VR_RULE_USE_STR_PERM)
			g_vr_strings[rule->permissions],
		#else
			rule->permissions,
		#endif
		#if defined(VR_USE_MAPPING)
			map,
		#endif
			AVTAB_ALLOWED, 0, &patched);

		if (ret)
			break;
	}

	if (patched_ptr)
		*patched_ptr |= patched;

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_patch_sepolicy_for_domain_allow_rules_for_read RS_HIDE(_32_2)
#endif
noused notrace static int vr_patch_sepolicy_for_domain_allow_rules_for_read(struct policydb *policydb_ptr,
#if defined(VR_USE_MAPPING)
	struct selinux_map *map,
#endif
	int *patched_ptr,
	int is_clear)
{
	int ret = 0;
	int patched = 0;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(g_vr_domain_allow_rules); i++) {
		const se_attr_rule_struct *rule = &g_vr_domain_allow_rules[i];

		patched = 0;

		ret = vr_policydb_update_typerule(policydb_ptr, g_vr_strings[rule->sourceAttr],
			rule->sourceMinuses,
			g_vr_strings[rule->target],
		#if defined(VR_RULE_USE_STR_CLASS)
			g_vr_strings[rule->_class],
		#else
			rule->_class,
		#endif
		#if defined(VR_RULE_USE_STR_PERM)
			g_vr_strings[rule->permissions],
		#else
			rule->permissions,
		#endif
		#if defined(VR_USE_MAPPING)
			map,
		#endif
			AVTAB_ALLOWED, is_clear, &patched);

		if (patched_ptr)
			*patched_ptr |= patched;

		if ((!is_clear) && (ret))
			break;
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_fix_policy_db_len RS_HIDE(_33)
#endif
noused notrace static int vr_fix_policy_db_len(struct policydb *policydb_ptr)
{
	#define VR_POLICYDB_INCREASE_SIZE_DIVIDEND (32) /*"/sepolicy" file at least 180KB*/
	#define VR_MIN_POLICYDB_INCREASE_SIZE (PAGE_SIZE)
	#define VR_MAX_POLICYDB_INCREASE_SIZE (PAGE_SIZE * 8)
	#define VR_POLICYDB_MAX_SIZE (64 * 1024 * 1024) /*refer to sel_write_load() in selinuxfs.c*/

	int ret;
	void *data = NULL;
	size_t len, increase_len;
	struct policy_file fp;

	/*if (!state->initialized) {
		ret = -EINVAL;
		goto out;
	}
	*/

	len = policydb_ptr->len; /*security_policydb_len();*/
	increase_len = len / VR_POLICYDB_INCREASE_SIZE_DIVIDEND;
	if (increase_len < VR_MIN_POLICYDB_INCREASE_SIZE)
		increase_len = VR_MIN_POLICYDB_INCREASE_SIZE;
	else if (increase_len > VR_MAX_POLICYDB_INCREASE_SIZE)
		increase_len = VR_MAX_POLICYDB_INCREASE_SIZE;

	len += increase_len;

	len = PAGE_ALIGN(len);

	if (len > VR_POLICYDB_MAX_SIZE) {
		ret = -EFBIG;
		goto out;
	}

	data = vmalloc(len);
	if (!data) {
		ret = -ENOMEM;
		goto out;
	}

	fp.data = data;
	fp.len = len;

	/*read_lock(&state->ss->policy_rwlock);*/
	ret = policydb_write(policydb_ptr, &fp);
	/*read_unlock(&state->ss->policy_rwlock);*/

	if (ret)
		goto out;

	len = (unsigned long)fp.data - (unsigned long)data;

	/*write_lock(&state->ss->policy_rwlock);*/
	policydb_ptr->len = len;
	/*write_unlock(&state->ss->policy_rwlock);*/

out:
	if (data)
		vfree(data);

	if (ret > 0)
		ret = -ret;

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_fix_policy_db_len_for_read RS_HIDE(_33_)
#endif
noused notrace static int vr_fix_policy_db_len_for_read(
	struct policydb *policydb_ptr, struct policy_file *fp)
{
	int ret;
	void *data;
	size_t len;

	if ((!fp) || (!fp->data) || (!fp->len)) {
		ret = -EINVAL;
		goto out;
	}

	len = policydb_ptr->len; /*security_policydb_len();*/

	if (len > fp->len) {
		ret = -EFBIG;
		goto out;
	}

	data = fp->data;
	/*read_lock(&state->ss->policy_rwlock);*/
	ret = policydb_write(policydb_ptr, fp);
	/*read_unlock(&state->ss->policy_rwlock);*/

	if (ret) {
		fp->data = data;
		goto out;
	}

	len = (unsigned long)fp->data - (unsigned long)data;

	fp->data = data;
	/*fp->len = len; //not needed, for len should <= fp->len*/

	/*write_lock(&state->ss->policy_rwlock);*/
	policydb_ptr->len = len;
	/*write_unlock(&state->ss->policy_rwlock);*/

out:

	if (ret > 0)
		ret = -ret;

	return ret;
}

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_patch_sepolicy_for_vndservicemanager_allow_rules RS_HIDE(_33_10)
#endif
noused notrace static int vr_patch_sepolicy_for_vndservicemanager_allow_rules(struct policydb *policydb_ptr,
#if defined(VR_USE_MAPPING)
	struct selinux_map *map,
#endif
	int *patched_ptr,
	int is_clear)
{
	int ret = 0;
	int patched = 0;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(g_vr_vndservicemanager_allow_rules); i++) {
		const se_rule_struct *rule = &g_vr_vndservicemanager_allow_rules[i];

	#if defined(VR_USE_MAPPING)
		/*printk("perm:0,%x,%x,%x\n", g_test_allow_perms[i], vr_unmap_perms(map, rule->_class, g_test_allow_perms[i]),
			vr_unmap_perms_no_map_ex(policydb_ptr, rule->_class, g_test_allow_perms[i], NULL));
		*/
	#else
		/*printk("perm:0,%x,%x\n", g_test_allow_perms[i], vr_unmap_perms_no_map_ex(policydb_ptr, rule->_class, g_test_allow_perms[i], NULL));*/
	#endif
		patched = 0;

		ret = vr_policydb_update_rule(policydb_ptr, g_vr_strings[rule->source],
			g_vr_strings[rule->target],
		#if defined(VR_RULE_USE_STR_CLASS)
			g_vr_strings[rule->_class],
		#else
			rule->_class,
		#endif
		#if defined(VR_RULE_USE_STR_PERM)
			g_vr_strings[rule->permissions],
		#else
			rule->permissions,
		#endif
		#if defined(VR_USE_MAPPING)
			map,
		#endif
			AVTAB_ALLOWED, is_clear, &g_vr_vndservicemanager_allow_rules_perms[i], &patched);

		if (patched_ptr)
			*patched_ptr |= patched;

		if ((!is_clear) && (ret))
			break;
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_patch_sepolicy_for_hwservicemanager_allow_rules RS_HIDE(_33_11)
#endif
noused notrace static int vr_patch_sepolicy_for_hwservicemanager_allow_rules(struct policydb *policydb_ptr,
#if defined(VR_USE_MAPPING)
	struct selinux_map *map,
#endif
	int *patched_ptr,
	int is_clear)
{
	int ret = 0;
	int patched = 0;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(g_vr_hwservicemanager_allow_rules); i++) {
		const se_rule_struct *rule = &g_vr_hwservicemanager_allow_rules[i];

	#if defined(VR_USE_MAPPING)
		/*printk("perm:0,%x,%x,%x\n", g_test_allow_perms[i], vr_unmap_perms(map, rule->_class, g_test_allow_perms[i]),
			vr_unmap_perms_no_map_ex(policydb_ptr, rule->_class, g_test_allow_perms[i], NULL));
		*/
	#else
		/*printk("perm:0,%x,%x\n", g_test_allow_perms[i], vr_unmap_perms_no_map_ex(policydb_ptr, rule->_class, g_test_allow_perms[i], NULL));*/
	#endif
		patched = 0;
		ret = vr_policydb_update_rule(policydb_ptr, g_vr_strings[rule->source],
			g_vr_strings[rule->target],
		#if defined(VR_RULE_USE_STR_CLASS)
			g_vr_strings[rule->_class],
		#else
			rule->_class,
		#endif
		#if defined(VR_RULE_USE_STR_PERM)
			g_vr_strings[rule->permissions],
		#else
			rule->permissions,
		#endif
		#if defined(VR_USE_MAPPING)
			map,
		#endif
			AVTAB_ALLOWED, is_clear, &g_vr_hwservicemanager_allow_rules_perms[i], &patched);

		if (patched_ptr)
			*patched_ptr |= patched;

		if ((!is_clear) && (ret))
			break;
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_patch_sepolicy_for_domain_allow_rules_8_x RS_HIDE(_33_12)
#endif
noused notrace static int vr_patch_sepolicy_for_domain_allow_rules_8_x(struct policydb *policydb_ptr,
#if defined(VR_USE_MAPPING)
	struct selinux_map *map,
#endif
	int *patched_ptr,
	int is_clear)
{
	int ret = 0;
	int patched = 0;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(g_vr_domain_allow_rules_8_x); i++) {
		const se_rule_struct *rule = &g_vr_domain_allow_rules_8_x[i];

		patched = 0;
		ret = vr_policydb_update_rule(policydb_ptr, g_vr_strings[rule->source],
			g_vr_strings[rule->target],
		#if defined(VR_RULE_USE_STR_CLASS)
			g_vr_strings[rule->_class],
		#else
			rule->_class,
		#endif
		#if defined(VR_RULE_USE_STR_PERM)
			g_vr_strings[rule->permissions],
		#else
			rule->permissions,
		#endif
		#if defined(VR_USE_MAPPING)
			map,
		#endif
			AVTAB_ALLOWED, is_clear, &g_vr_domain_allow_rules_perms_8_x[i], &patched);

		if (patched_ptr)
			*patched_ptr |= patched;

		if ((!is_clear) && (ret))
			break;
	}

	return ret;
}

/*noused notrace static int update_rules_for_8_0_above(struct policydb *policy);*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_patch_sepolicy_internal RS_HIDE(_34)
#endif
noused notrace static int vr_patch_sepolicy_internal(struct selinux_state *state,
	struct policydb *policydb_ptr,
#if defined(VR_USE_MAPPING)
	struct selinux_map *map,
#endif
	int *patched_ptr)
{
	int ret;
	int su_patched = 0;
	int real_patched = 0;
	int patched = 0;

	if ((g_vr_otherrooted) && (g_vr_other_sid)) {
		ret = vr_policydb_update_permissive_by_sid(state, policydb_ptr, g_vr_other_sid, 0, &g_vr_other_permissive, &patched);
		if (!ret) {
			su_patched = 1;

			real_patched |= patched;

			patched = 0;
			ret = vr_policydb_update_permissive(policydb_ptr, g_vr_strings[vrseSu], 0, &g_vr_su_permissive, &patched);
			/*ret = vr_policydb_update_permissive_by_sid(state, &policydb, g_vr_su_sid, 0, &g_vr_su_permissive, &patched);*/

			g_su_patched = patched;

			real_patched |= patched;

			VR_LOG("psi,su[0],%d,%d,%d\n", ret, patched, g_vr_su_permissive);
		}

		if (ret)
			goto out;
	}

#if !defined(VR_NO_PATCH_RULES)
	patched = 0;
	ret = vr_patch_sepolicy_for_adbd_trigger_rule(policydb_ptr,
	#if defined(VR_USE_MAPPING)
		map,
	#endif
		&patched);

	VR_LOG("psi,2,%d,%d\n", ret, patched);

	if (!ret)
#endif
	{
	#if !defined(VR_NO_PATCH_RULES)
		real_patched |= patched;
		g_adb_trigger_rule_patched = patched;
	#endif

		if (g_vr_vivorooted) {
		#if defined(VR_SET_ADBD_PERMISSIVE_AFTERWARD)
			/*extern int avc_allow_trans(void);*/
			/*avc_allow_trans(); //allow further dyntransition in avc_has_perm_noaudit()*/

			patched = 0;

			ret = vr_policydb_update_permissive(policydb_ptr, g_vr_strings[vrseAdbd], 0, &g_vr_adbd_permissive, &patched);

			real_patched |= patched;

			VR_LOG("psi,3,%d,%d\n", ret, patched);
		#endif

			if (!su_patched) {

				patched = 0;
				ret = vr_policydb_update_permissive(policydb_ptr, g_vr_strings[vrseSu], 0, &g_vr_su_permissive, &patched);

				real_patched |= patched;
				g_su_patched = patched;

				VR_LOG("psi,su[1],%d,%d,%d\n", ret, patched, g_vr_su_permissive);
			}
		} else {
		#if !defined(VR_NO_PATCH_RULES)
			patched = 0;

			ret = vr_patch_sepolicy_for_adbd_allow_rules(policydb_ptr,
			#if defined(VR_USE_MAPPING)
				map,
			#endif
				&patched);

			real_patched |= patched;
			g_adb_allow_rules_patched = patched;

			VR_LOG("psi,5,%d,%d\n", ret, patched);

		#if defined(VR_UPDATE_DOMAIN_RULES)
			patched = 0;

			ret = vr_patch_sepolicy_for_domain_allow_rules(policydb_ptr,
			#if defined(VR_USE_MAPPING)
				map,
			#endif
				&patched);

			real_patched |= patched;

			VR_LOG("psi,5.1,%d,%d\n", ret, patched);
		#endif

		#endif
		}
	}

	if ((!ret) && (g_su_patched) && (g_vr_vivorooted)) {
		patched = 0;
		ret = vr_policydb_update_permissive(policydb_ptr,
			g_vr_strings[vrseShell], 0, &g_vr_shell_permissive, &patched);
		g_vr_shell_permissive_patched = patched;
		real_patched |= patched;

		VR_LOG("psi,6,%d\n", patched);

		patched = 0;
		ret = vr_policydb_update_permissive(policydb_ptr,
			g_vr_strings[vrseDumpState], 0,
			&g_vr_dumpstate_permissive, &patched);

		g_vr_dumpstate_permissive_patched = patched;
		real_patched |= patched;

		VR_LOG("psi,7,%d,%d\n", ret, patched);

		patched = 0;
		ret = vr_policydb_update_permissive(policydb_ptr,
			g_vr_strings[vrseIncident], 0,
			&g_vr_incident_permissive, &patched);
		g_vr_incident_permissive_patched = patched;
		real_patched |= patched;

		VR_LOG("psi,8,%d\n", patched);

#if defined(VR_PATCH_VNDSERVICEMANAGER_POLICY_BY_RULES)
		patched = 0;
		vr_patch_sepolicy_for_vndservicemanager_allow_rules(policydb_ptr,
			#if defined(VR_USE_MAPPING)
				map,
			#endif
				&patched, 0);

		g_vr_vndservicemanager_rules_patched = patched;
		real_patched |= patched;
#else
		patched = 0;
		vr_policydb_update_permissive(policydb_ptr,
			g_vr_strings[vrseVndServiceManager], 0,
			&g_vr_vndservicemanager_permissive, &patched);

		g_vr_vndservicemanager_permissive_patched = patched;
		real_patched |= patched;
#endif

		VR_LOG("psi,9,%d,%d\n", ret, patched);

	#if defined(VR_IS_ANDROID_8_0_ABOVE)
		patched = 0;
		vr_patch_sepolicy_for_hwservicemanager_allow_rules(policydb_ptr,
			#if defined(VR_USE_MAPPING)
				map,
			#endif
				&patched, 0);

		g_vr_hwservicemanager_rules_patched = patched;
		real_patched |= patched;

		VR_LOG("psi,10,%d,%d\n", ret, patched);

		patched = 0;
		vr_policydb_update_typeattribute(policydb_ptr,
			g_vr_strings[vrseSu], g_vr_strings[vrseAppdomain], 0,
			&g_vr_su_attr_appdomain, &patched);

		g_vr_su_attr_appdomain_patched = patched;
		real_patched |= patched;

		VR_LOG("psi,11,%d,%d\n", ret, patched);

		patched = 0;
		vr_policydb_update_typeattribute(policydb_ptr,
			g_vr_strings[vrseSu], g_vr_strings[vrseCoredomain], 0,
			&g_vr_su_attr_coredomain, &patched);

		g_vr_su_attr_coredomain_patched = patched;
		real_patched |= patched;

		VR_LOG("psi,12,%d,%d\n", ret, patched);

		patched = 0;
		vr_policydb_update_typeattribute(policydb_ptr,
			g_vr_strings[vrseSu], g_vr_strings[vrseMlstrustedsubject], 0,
			&g_vr_su_attr_mlstrustedsubject, &patched);

		g_vr_su_attr_mlstrustedsubject_patched = patched;
		real_patched |= patched;

		VR_LOG("psi,13,%d,%d\n", ret, patched);

		patched = 0;
		vr_policydb_update_typeattribute(policydb_ptr,
			g_vr_strings[vrseSu], g_vr_strings[vrseNetdomain], 0,
			&g_vr_su_attr_netdomain, &patched);

		g_vr_su_attr_netdomain_patched = patched;
		real_patched |= patched;

		VR_LOG("psi,14,%d,%d\n", ret, patched);

		patched = 0;
		vr_patch_sepolicy_for_domain_allow_rules_8_x(policydb_ptr,
			#if defined(VR_USE_MAPPING)
				map,
			#endif
				&patched, 0);

		g_vr_domain_rules_8_x_patched = patched;
		real_patched |= patched;

		VR_LOG("psi,15,%d,%d\n", ret, patched);
	#endif
	}

#if !defined(VR_NO_PATCH_RULES)
	if ((!ret) && (real_patched)) {
		ret = vr_fix_policy_db_len(policydb_ptr);
		VR_LOG("psi,10,%d\n", ret);
	}
#endif

out:
	if (patched_ptr)
		*patched_ptr |= real_patched;

	return ret;
}


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_patch_sepolicy_for_read RS_HIDE(_34_)
#endif
noused notrace static int vr_patch_sepolicy_for_read(struct selinux_state *state,
	struct policydb *policydb_ptr,
#if defined(VR_USE_MAPPING)
	struct selinux_map *map,
#endif
#if defined(VR_FIX_DB_LEN_AFTER_UNPATCH_FOR_READ)
	struct policy_file *fp,
#endif
	int *need_notify_ptr,
	int is_clear)
{
	int ret = -EINVAL;
	int patched = 0;
	int real_patched = 0;
	int need_notify = 0;

	if ((g_vr_otherrooted) && (g_vr_other_sid)) {
		ret = vr_policydb_update_permissive_by_sid(state, policydb_ptr,
			g_vr_other_sid, is_clear, &g_vr_other_permissive, &patched);

		if ((!ret) && (!patched)) {
			VR_LOG("ps4r,1\n");
			need_notify = 1;
		}

		real_patched |= patched;
	}

	if (g_su_patched) {
		patched = 0;

		ret = vr_policydb_update_permissive(policydb_ptr,
			g_vr_strings[vrseSu], is_clear, &g_vr_su_permissive, &patched);

		if ((!ret) && (!patched)) {
			VR_LOG("ps4r,2\n");
			need_notify = 1;
		}

		real_patched |= patched;

		VR_LOG("ps4r,su,%d,%d,%d,%d\n", is_clear, ret, patched, g_vr_su_permissive);
	}
#if !defined(VR_NO_PATCH_RULES)
	if (g_adb_trigger_rule_patched) {
		patched = 0;

		ret = vr_patch_sepolicy_for_adbd_trigger_rule_for_read(policydb_ptr,
		#if defined(VR_USE_MAPPING)
			map,
		#endif
			&patched,
			is_clear);

		if ((!ret) && (!patched)) {
			VR_LOG("ps4r,3\n");
			need_notify = 1;
		}

		real_patched |= patched;
	}
#endif

	if (g_vr_vivorooted) {
	#if defined(VR_SET_ADBD_PERMISSIVE_AFTERWARD)
		/*extern int avc_allow_trans(void);*/
		/*avc_allow_trans(); //allow further dyntransition in avc_has_perm_noaudit()*/
		patched = 0;
		ret = vr_policydb_update_permissive(policydb_ptr,
			g_vr_strings[vrseAdbd], is_clear,
			&g_vr_adbd_permissive, &patched);

		if ((!ret) && (!patched)) {
			VR_LOG("ps4r,4\n");
			need_notify = 1;
		}

		real_patched |= patched;
	#endif
	}
	/*else*/
	{
	#if !defined(VR_NO_PATCH_RULES)
		if (g_adb_allow_rules_patched) {
			patched = 0;

			ret = vr_patch_sepolicy_for_adbd_allow_rules_for_read(policydb_ptr,
			#if defined(VR_USE_MAPPING)
				map,
			#endif
				&patched,
				is_clear);

			if ((!ret) && (!patched)) {
				VR_LOG("ps4r,5\n");
				need_notify = 1;
			}

			real_patched |= patched;
		}

	#if defined(VR_UPDATE_DOMAIN_RULES)
		if (g_domain_allow_rules_patched) {
			patched = 0;

			ret = vr_patch_sepolicy_for_domain_allow_rules_for_read(policydb_ptr,
			#if defined(VR_USE_MAPPING)
				map,
			#endif
				&patched,
				is_clear);

			if ((!ret) && (!patched)) {
				VR_LOG("ps4r,5.1\n");
				need_notify = 1;
			}

			real_patched |= patched;
		}
	#endif

	#endif
	}

	if (g_vr_shell_permissive_patched) {
		patched = 0;
		ret = vr_policydb_update_permissive(policydb_ptr,
			g_vr_strings[vrseShell], is_clear,
			&g_vr_shell_permissive, &patched);

		if ((!ret) && (!patched)) {
			VR_LOG("ps4r,6\n");
			need_notify = 1;
		}

		real_patched |= patched;
	}

	if (g_vr_dumpstate_permissive_patched) {
		patched = 0;
		ret = vr_policydb_update_permissive(policydb_ptr,
			g_vr_strings[vrseDumpState], is_clear,
			&g_vr_dumpstate_permissive, &patched);

		if ((!ret) && (!patched)) {
			VR_LOG("ps4r,7\n");
			need_notify = 1;
		}

		real_patched |= patched;
	}

	if (g_vr_incident_permissive_patched) {
		patched = 0;
		ret = vr_policydb_update_permissive(policydb_ptr,
			g_vr_strings[vrseIncident], is_clear,
			&g_vr_incident_permissive, &patched);

		if ((!ret) && (!patched)) {
			VR_LOG("ps4r,8\n");
			need_notify = 1;
		}

		real_patched |= patched;
	}

#if defined(VR_PATCH_VNDSERVICEMANAGER_POLICY_BY_RULES)
	if (g_vr_vndservicemanager_rules_patched) {
		patched = 0;
		ret = vr_patch_sepolicy_for_vndservicemanager_allow_rules(policydb_ptr,
			#if defined(VR_USE_MAPPING)
				map,
			#endif
				&patched, is_clear);

		if ((!ret) && (!patched)) {
			VR_LOG("ps4r,9\n");
			need_notify = 1;
		}

		real_patched |= patched;
	}
#else
	if (g_vr_vndservicemanager_permissive_patched) {
		patched = 0;
		ret = vr_policydb_update_permissive(policydb_ptr,
			g_vr_strings[vrseVndServiceManager], is_clear,
			&g_vr_vndservicemanager_permissive, &patched);

		if ((!ret) && (!patched)) {
			VR_LOG("ps4r,9\n");
			need_notify = 1;
		}

		real_patched |= patched;
	}
#endif

#if defined(VR_IS_ANDROID_8_0_ABOVE)
	if (g_vr_hwservicemanager_rules_patched) {
		patched = 0;
		ret = vr_patch_sepolicy_for_hwservicemanager_allow_rules(policydb_ptr,
			#if defined(VR_USE_MAPPING)
				map,
			#endif
				&patched, is_clear);

		if ((!ret) && (!patched)) {
			VR_LOG("ps4r,10\n");
			need_notify = 1;
		}

		real_patched |= patched;
	}

	if (g_vr_su_attr_appdomain_patched) {
		patched = 0;
		ret = vr_policydb_update_typeattribute(policydb_ptr,
			g_vr_strings[vrseSu], g_vr_strings[vrseAppdomain], is_clear,
			&g_vr_su_attr_appdomain, &patched);

		if ((!ret) && (!patched)) {
			VR_LOG("ps4r,11\n");
			need_notify = 1;
		}

		real_patched |= patched;
	}

	if (g_vr_su_attr_coredomain_patched) {
		patched = 0;
		ret = vr_policydb_update_typeattribute(policydb_ptr,
			g_vr_strings[vrseSu], g_vr_strings[vrseCoredomain], is_clear,
			&g_vr_su_attr_coredomain, &patched);

		if ((!ret) && (!patched)) {
			VR_LOG("ps4r,12\n");
			need_notify = 1;
		}

		real_patched |= patched;
	}

	if (g_vr_su_attr_mlstrustedsubject_patched) {
		patched = 0;
		ret = vr_policydb_update_typeattribute(policydb_ptr,
			g_vr_strings[vrseSu], g_vr_strings[vrseMlstrustedsubject], is_clear,
			&g_vr_su_attr_mlstrustedsubject, &patched);

		if ((!ret) && (!patched)) {
			VR_LOG("ps4r,13\n");
			need_notify = 1;
		}

		real_patched |= patched;
	}

	if (g_vr_su_attr_netdomain_patched) {
		patched = 0;
		ret = vr_policydb_update_typeattribute(policydb_ptr,
			g_vr_strings[vrseSu], g_vr_strings[vrseNetdomain], is_clear,
			&g_vr_su_attr_netdomain, &patched);

		if ((!ret) && (!patched)) {
			VR_LOG("ps4r,14\n");
			need_notify = 1;
		}

		real_patched |= patched;
	}

	if (g_vr_domain_rules_8_x_patched) {
		patched = 0;
		ret = vr_patch_sepolicy_for_domain_allow_rules_8_x(policydb_ptr,
			#if defined(VR_USE_MAPPING)
				map,
			#endif
				&patched, is_clear);

		if ((!ret) && (!patched)) {
			VR_LOG("ps4r,15\n");
			need_notify = 1;
		}

		real_patched |= patched;
	}
#endif

#if !defined(VR_NO_PATCH_RULES)
	if ((!ret) && (real_patched)) {
		VR_LOG("ps4r,f,%d\n", is_clear);
		if (is_clear) {
			/* data size after restore patch should < data size before restore patch */
		#if defined(VR_FIX_DB_LEN_AFTER_UNPATCH_FOR_READ)
			if (fp)
				ret = vr_fix_policy_db_len_for_read(policydb_ptr, fp);
		#endif
		} else {
			/* data size after re-patch should == data size before restore patch */
		#if defined(VR_FIX_DB_LEN_AFTER_REPATCH_FOR_READ)
			ret = vr_fix_policy_db_len(policydb_ptr);
		#endif
		}
	}
#endif

/*out:*/
	if (need_notify_ptr)
		*need_notify_ptr = need_notify;

	VR_LOG("ps4r,e,%d,%d\n", is_clear, ret);
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_patch_sepolicy RS_HIDE(_35)
#endif
noused notrace static int vr_patch_sepolicy(struct selinux_state *state,
	struct policydb *policydb_ptr
#if defined(VR_USE_MAPPING)
	, struct selinux_map *map
#endif
	)
{
	noused int patched = 0;

	return vr_patch_sepolicy_internal(state, policydb_ptr,
	#if defined(VR_USE_MAPPING)
		map,
	#endif
		&patched);
}

#if 0
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_patch_sepolicy_2 RS_HIDE(_36)
#endif
noused notrace static int vr_patch_sepolicy_2(struct policydb *policydb_ptr)
{
	int patched = 0;

	if (g_vr_vivorooted) {
		vr_policydb_update_permissive(policydb_ptr,
			g_vr_strings[vrseAdbd], 0,
			g_vr_adbd_permissive, &patched);
		vr_policydb_update_permissive(policydb_ptr,
			g_vr_strings[vrseSu], 0,
			&g_vr_su_permissive, &patched);
	} else {
		vr_patch_sepolicy_for_adbd_allow_rules(policydb_ptr, &patched);
	}

	return 0;
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_context_to_sid RS_HIDE(_37)
#endif
static int noused vr_context_to_sid(struct selinux_state *state, const char *ctx, u32 *sid_ptr)
{
	int ret;
	int len;

	if ((!state) || (!sid_ptr))
		return -EINVAL;

	if ((!ctx) || (!ctx[0])) {
		ret = -EINVAL;
		goto out;
	}

	len = strlen(ctx);

#if defined(VR_NEW_VERSION)
	ret = security_context_to_sid(state, ctx, len, sid_ptr, GFP_ATOMIC);
#else
	ret = security_context_to_sid(ctx, len, sid_ptr);
#endif
	if (ret == -EINVAL)
#if defined(VR_NEW_VERSION)
		/* copied from security_context_to_sid_force(), but use GFP_ATOMIC instead */
		ret = security_context_to_sid_default(state, ctx, len,
						sid_ptr, SECSID_NULL, GFP_ATOMIC);
#else
		ret = security_context_to_sid_force(ctx, len, sid_ptr);
#endif

out:
	if (ret) {
		*sid_ptr = 0;

		VR_LOG("Can't get sid for %s\n", ctx);
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
/*#define vr_init_notify_sids RS_HIDE(_38)*/
#endif
int vr_init_notify_sids(struct selinux_state *state)
{
	int ret;

	if (!state->initialized) {
		ret = -EINVAL;
		goto out;
	}

	ret = vr_context_to_sid(state, g_vr_strings[vrseAdbdContext], &g_vr_adbd_sid);
	ret = vr_context_to_sid(state, g_vr_strings[vrseSuContext], &g_vr_su_sid);

	ret = vr_context_to_sid(state, g_vr_strings[vrseInitContext], &g_vr_init_sid);
	ret = vr_context_to_sid(state, g_vr_strings[vrseSystemFileContext], &g_vr_system_file_sid);

out:
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_check_access_no_audit RS_HIDE(_39)
#endif
noused notrace static int vr_check_access_no_audit(struct selinux_state *state,
	const char *scon, const char *tcon, const char *_class, const char *perm)
{
	int rc = -1;
	u32 scon_id;
	u32 tcon_id;
	u32 sclass;
	u32 request;
	struct av_decision avd;

	if (!state->initialized)
		return -EINVAL;

	rc = vr_context_to_sid(state, scon, &scon_id);
	if (rc < 0)
		return rc;

	rc = vr_context_to_sid(state, tcon, &tcon_id);
	if (rc < 0)
		return rc;

	sclass = string_to_security_class(&state->ss->policydb, _class);
	if (sclass == 0)
		return -EINVAL;

	request = string_to_av_perm(&state->ss->policydb, sclass, perm);
	if (request == 0)
		return -EINVAL;

	memset(&avd, 0, sizeof(avd));

	return avc_has_perm_noaudit(state, scon_id, tcon_id, sclass, request, 0, &avd);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_notify_sepolicy_update_4_adbd RS_HIDE(_3a)
#endif
noused notrace static int vr_notify_sepolicy_update_4_adbd(struct selinux_state *state)
{
	int ret;
	int seqno = 0;
	int patched = 0;
	struct policydb *policydb = &state->ss->policydb;
#if defined(VR_USE_MAPPING)
	struct selinux_map map = state->ss->map;
#endif

	g_vr_vivorooted = 1;

	/*write_lock_irq(&state->ss->policy_rwlock);*/
	mutex_lock(&g_vr_policy_write_lock);

	ret = vr_patch_sepolicy_internal(state, policydb,
	#if defined(VR_USE_MAPPING)
		&map,
	#endif
		&patched);

	VR_LOG("nsu4a,1,%d,%d\n", ret, patched);

	if ((!ret) && (patched)) {
		seqno = vr_get_next_seqno(state);
	} else {
		ret = -EINVAL;
	}

	mutex_unlock(&g_vr_policy_write_lock);
	/*write_unlock_irq(&state->ss->policy_rwlock);*/

	if (!ret) {
		avc_ss_reset(state->avc, seqno);
		selnl_notify_policyload(seqno);
		selinux_status_update_policyload(state, seqno);
		selinux_netlbl_cache_invalidate();
		selinux_xfrm_notify_policyload();
	}
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_notify_sepolicy_update_4_other RS_HIDE(_3b)
#endif
noused notrace static int vr_notify_sepolicy_update_4_other(struct selinux_state *state, u32 sid)
{
	int ret;
	int seqno = 0;
	int patched = 0;
	struct policydb *policydb = &state->ss->policydb;

	g_vr_otherrooted = 1;
	g_vr_other_sid = sid;

	/*write_lock_irq(&state->ss->policy_rwlock);*/
	mutex_lock(&g_vr_policy_write_lock);

	ret = vr_policydb_update_permissive_by_sid(state, policydb, sid,
		0, &g_vr_other_permissive, &patched);
	if (!ret) {
		ret = vr_policydb_update_permissive(policydb,
			g_vr_strings[vrseSu], 0,
			&g_vr_su_permissive, &patched);
		VR_LOG("ps4o,su,%d,%d,%d\n", ret, patched, g_vr_su_permissive);
	}

	if ((!ret) && (patched)) {
		seqno = vr_get_next_seqno(state);
	} else {
		ret = -EINVAL;
	}

	mutex_unlock(&g_vr_policy_write_lock);
	/*write_unlock_irq(&state->ss->policy_rwlock);*/

	if (!ret) {
		avc_ss_reset(state->avc, seqno);
		selnl_notify_policyload(seqno);
		selinux_status_update_policyload(state, seqno);
		selinux_netlbl_cache_invalidate();
		selinux_xfrm_notify_policyload();
	}
	return ret;
}

int __weak is_dbg_task(struct task_struct *task, int check_path)
{
	return 1;
}

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
/*#define vr_try_update_policy RS_HIDE(_3c)*/
#endif
int vr_try_update_policy(struct selinux_state *state, struct task_struct *task, u32 sid, u32 tid)
{
#if defined(VIVOROOT_SUPPORT)
	int ret = 0;

	if ((g_vr_su_already_permissive == 0) && (tid == g_vr_su_sid) && (is_dbg_task(task, 1) > 0)) {
		if (sid == g_vr_adbd_sid) {
			if (!vr_notify_sepolicy_update_4_adbd(state))
				ret = 1;
		} else if (state->initialized) {
			if (!vr_notify_sepolicy_update_4_other(state, sid))
				ret = 1;
		}
	}

	return ret;
#else
	return 0;
#endif
}

#if defined(VIVOROOT_SUPPORT)
noused notrace static void vr_reset_load_policy_data(struct tag_vr_patch_policy_for_load_param *param)
{
	param->fp->data = param->data;
	param->fp->len = param->len;

	policydb_destroy(param->policydb_ptr);
	memset(param->policydb_ptr, 0, sizeof(*(param->policydb_ptr)));
	sidtab_destroy(param->sidtab_ptr);
	memset(param->sidtab_ptr, 0, sizeof(*(param->sidtab_ptr)));
	kfree(param->map_ptr->mapping);
	param->map_ptr->mapping = NULL;
	param->map_ptr->size = 0;
}

int vr_try_patch_policy_after_load(struct tag_vr_patch_policy_for_load_param *param,
	int *loopcnt_ptr)
{
	vr_init_strings();

	if (!(*loopcnt_ptr)) {
		g_vr_su_already_permissive = vr_policydb_is_su_permissive(param->policydb_ptr);

		if (g_vr_su_already_permissive == 0) {
			(*loopcnt_ptr)++;

			if (vr_patch_sepolicy(param->state, param->policydb_ptr
			#if defined(VR_USE_MAPPING)
				, param->map_ptr
			#endif
				)) {
				vr_reset_load_policy_data(param);

				return -EINVAL;
			}
		} else {
		#if defined(VR_IS_ANDROID_8_0_ABOVE)
			VR_LOG("slp,4.1\n");
			set_rs_s_u();
		#endif
		}
	}

	return 0;
}
#endif

#if defined(VR_RESTORE_POLICY_PATCH_FOR_READ)

void vr_try_patch_policy_for_read(struct tag_vr_patch_policy_for_read_param *param,
	int is_clear)
{
	if (is_clear) {
		param->need_notify = 0;

		/*write_lock_irq(&param->state->ss->policy_rwlock);*/
		mutex_lock(&g_vr_policy_write_lock);

		if (g_vr_su_already_permissive == 0) {
			param->patch_ret = vr_patch_sepolicy_for_read(param->state, param->policydb_ptr,
			#if defined(VR_USE_MAPPING)
				&(param->vr_map),
			#endif
			#if defined(VR_FIX_DB_LEN_AFTER_UNPATCH_FOR_READ)
				param->fp,
			#endif
				&(param->need_notify),
				1);
		} else {
			param->patch_ret = -EINVAL;
		}
	} else {
		if (g_vr_su_already_permissive == 0) {
			if (!param->patch_ret) {
				vr_patch_sepolicy_for_read(param->state, param->policydb_ptr,
				#if defined(VR_USE_MAPPING)
					&param->vr_map,
				#endif
				#if defined(VR_FIX_DB_LEN_AFTER_UNPATCH_FOR_READ)
					NULL,
				#endif
					&param->need_notify,
					0);
			}
		}

		mutex_unlock(&g_vr_policy_write_lock);
		/*write_unlock_irq(&param->state->ss->policy_rwlock);*/
	}
}

void vr_try_notify_policy_changed_for_read(struct tag_vr_patch_policy_for_read_param *param)
{
	if ((g_vr_su_already_permissive == 0) && (!param->patch_ret) && (param->need_notify)) {
		u32 seqno;

		VR_LOG("srp,2\n");

		seqno = vr_get_next_seqno(param->state);

		avc_ss_reset(param->state->avc, seqno);
		selnl_notify_policyload(seqno);
		selinux_status_update_policyload(param->state, seqno);
		selinux_netlbl_cache_invalidate();
		selinux_xfrm_notify_policyload();
	}
}
#endif


#if defined(CONFIG_RS_OBFUSCATED_NAME)
/*#define rs_sec_bounded_trans RS_HIDE(_3d)*/
#endif
int rs_sec_bounded_trans(struct selinux_state *state, u32 old_sid, u32 new_sid,
	u32 *type_index_ptr, u32 *bounds_ptr, int is_restore)
{
#if defined(VIVOROOT_SUPPORT)
	struct policydb *policydb;

	struct context *old_context, *new_context;
	struct type_datum *type;
	int index;
	int rc;

	if ((!state) || (!type_index_ptr) || (!bounds_ptr))
		return -EINVAL;

	if (!is_restore) {
		*type_index_ptr = 0;
		*bounds_ptr = 0;
	}

	policydb = &state->ss->policydb;

	read_lock(&state->ss->policy_rwlock);
	/*mutex_lock(&g_vr_policy_write_lock);*/

	rc = -EINVAL;
	old_context = vr_sidtab_search(state, old_sid);
	if (!old_context) {
		/*printk(KERN_ERR "SELinux: %s: unrecognized SID %u\n",
		       __func__, old_sid);*/
		goto out;
	}

	rc = -EINVAL;
	new_context = vr_sidtab_search(state, new_sid);
	if (!new_context) {
		/*printk(KERN_ERR "SELinux: %s: unrecognized SID %u\n",
		       __func__, new_sid);*/
		goto out;
	}

	rc = 0;
	/* type/domain unchanged */
	if (old_context->type == new_context->type)
		goto out;

	index = new_context->type;
	while (true) {
		type = flex_array_get_ptr(policydb->type_val_to_struct_array,
						index - 1);
		WARN_ON(!type); /*BUG_ON(!type);*/

		rc = -EPERM;
		if (!type) {
			break;
		}

		if (is_restore) {
			if (*type_index_ptr == (u32)index) {
				type->bounds = *bounds_ptr;
				rc = 0;
				break;
			}
		} else {
			if (!type->bounds) {
				*type_index_ptr = (u32)index;
				*bounds_ptr = type->bounds;
				type->bounds = old_context->type;

				rc = 0;
				break;
			} else if (type->bounds == old_context->type) {
				*type_index_ptr = (u32)index;
				*bounds_ptr = type->bounds;

				rc = 0;
				break;
			}
		}

		index = type->bounds;
		if (!index)
			break;
	}
#if 0
	if (rc) {
		char *old_name = NULL;
		char *new_name = NULL;
		u32 length;

		if (!context_struct_to_string(old_context,
					      &old_name, &length) &&
		    !context_struct_to_string(new_context,
					      &new_name, &length)) {
			audit_log(current->audit_context,
				  GFP_ATOMIC, AUDIT_SELINUX_ERR,
				  "op=security_bounded_transition "
				  "result=denied "
				  "oldcontext=%s newcontext=%s",
				  old_name, new_name);
		}
		kfree(new_name);
		kfree(old_name);
	}
#endif
out:
	/*mutex_unlock(&g_vr_policy_write_lock);*/
	read_unlock(&state->ss->policy_rwlock);

	return rc;
#else
	return -EPERM;
#endif
}

#if defined(VIVOROOT_SUPPORT)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
/*#define vr_is_exec RS_HIDE(_3e)*/
#endif
int vr_is_exec(u32 src_sid, u32 node_sid)
{
	if ((src_sid == g_vr_init_sid) && (node_sid == g_vr_system_file_sid))
		return 1;
	else
		return 0;
}

#endif
/*end*/


#if defined(VIVOROOT_SUPPORT)

#if 0
static int get_attr(const char *type, u32 value, struct policydb *policy)
{
	struct type_datum *attr = hashtab_search(policy->p_types.table, type);
	if (!attr)
		return 1;

	if (attr->flavor != TYPE_ATTRIB)
		return 1;

	return !!ebitmap_get_bit(&policydb->attr_type_map[attr->value - 1], value - 1);
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_attr RS_HIDE(_b_1)
#endif
noused notrace static struct type_datum *get_attr(const char *type, struct policydb *policy)
{
	struct type_datum *attr;

	attr = hashtab_search(policy->p_types.table, type);
	if (!attr)
		return NULL;

	/*if (!attr->attribute) //if (attr->flavor != TYPE_ATTRIB)
		return NULL;
	*/

	return attr;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_attr_id RS_HIDE(_b_2)
#endif
noused notrace static u32 get_attr_id(const char *type, struct policydb *policy)
{
	struct type_datum *attr = get_attr(type, policy);

	return attr ? attr->value : 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_attr RS_HIDE(_b_3)
#endif
noused notrace static int set_attr(const char *type, const u32 value, struct policydb *policy)
{
	struct ebitmap *e;
	struct type_datum *attr = get_attr(type, policy);

	if (!attr)
		return -EINVAL;

	/*if (attr->flavor != TYPE_ATTRIB)
		return -EINVAL;
	*/

	e = flex_array_get(policy->type_attr_map_array, value - 1);
	if (!e)
		return -EINVAL;

	if (ebitmap_set_bit(e, attr->value - 1, 1))
		return -EINVAL;

	/*if (ebitmap_set_bit(&policy->type_attr_map[value-1], attr->value - 1, 1))
		return -EINVAL;
	*/

	/*if (ebitmap_set_bit(&policy->attr_type_map[attr->value-1], value - 1, 1))
		return -EINVAL;
	*/

	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define attr_is_set RS_HIDE(_b_3_)
#endif
noused notrace static int attr_is_set(const char *type, const u32 value, struct policydb *policy)
{
	struct ebitmap *e;
	struct type_datum *attr = get_attr(type, policy);

	if (!attr)
		return -EINVAL;

	/*if (attr->flavor != TYPE_ATTRIB)
		return -EINVAL;
	*/

	e = flex_array_get(policy->type_attr_map_array, value - 1);
	if (!e)
		return -EINVAL;

	if (ebitmap_get_bit(e, attr->value - 1))
		return 1;

	/*if (ebitmap_set_bit(&policy->type_attr_map[value-1], attr->value - 1, 1))
		return -EINVAL;
	*/

	/*if (ebitmap_set_bit(&policy->attr_type_map[attr->value-1], value - 1, 1))
		return -EINVAL;
	*/

	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define clear_attr RS_HIDE(_b_4)
#endif
noused notrace static int clear_attr(const char *type, u32 value, struct policydb *policy)
{
	struct ebitmap *e;
	struct type_datum *attr = hashtab_search(policy->p_types.table, type);

	if (!attr)
		return -EINVAL;

	/*if (attr->flavor != TYPE_ATTRIB)
		return -EINVAL;
	*/

	e = flex_array_get(policy->type_attr_map_array, value - 1);
	if (!e)
		return -EINVAL;

	if (ebitmap_set_bit(e, attr->value - 1, 0))
		return -EINVAL;

	/*if(ebitmap_set_bit(&policy->type_attr_map[value-1], attr->value-1, 0))
		return -EINVAL;
	*/

	/*if(ebitmap_set_bit(&policy->attr_type_map[attr->value-1], value-1, 0))
		return -EINVAL;
	*/

	return 0;
}


#if 0

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define type_datum_init RS_HIDE(_b_10)
#endif
noused notrace static void type_datum_init(struct type_datum *x)
{
	memset(x, 0, sizeof(*x));
	ebitmap_init(&x->types);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define create_domain RS_HIDE(_b_12)
#endif
noused notrace static struct type_datum *create_domain(const char *d, struct policydb *policy)
{
	uint32_t value = 0;
	unsigned int i;
	int r;
	struct type_datum *src;

	src = hashtab_search(policy->p_types.table, d);
	if (src) {
		VR_LOGI("Domain '%s' already exists\n", d);
		return src;
	}

	src = kzalloc(sizeof(type_datum), GFP_KERNEL);
	if (!src)
		goto fail;

	type_datum_init(src);
	src->primary = 1;
	src->flavor = TYPE_TYPE;

	r = symtab_insert(policy, SYM_TYPES, strdup(d), src,
				SCOPE_DECL, 1, &value);
	src->value = value;

	if (ebitmap_set_bit(&policy->global->branch_list->declared.scope[SYM_TYPES], value - 1, 1))
		goto fail;

	policy->type_attr_map = realloc(policy->type_attr_map, sizeof(ebitmap_t)*policy->p_types.nprim);
	policy->attr_type_map = realloc(policy->attr_type_map, sizeof(ebitmap_t)*policy->p_types.nprim);
	ebitmap_init(&policy->type_attr_map[value - 1]);
	ebitmap_init(&policy->attr_type_map[value - 1]);
	ebitmap_set_bit(&policy->type_attr_map[value - 1], value - 1, 1);

	/* add the domain to all roles */
	for (i = 0; i < policy->p_roles.nprim; i++) {
		/* not sure all those three calls are needed */
		ebitmap_set_bit(&policy->role_val_to_struct[i]->types.negset, value - 1, 0);
		ebitmap_set_bit(&policy->role_val_to_struct[i]->types.types, value - 1, 1);
		type_set_expand(&policy->role_val_to_struct[i]->types,
			&policy->role_val_to_struct[i]->cache, policy, 0);
	}

	src = hashtab_search(policy->p_types.table, d);
	if (!src)
		goto fail;

	if (policydb_index_decls(policy))
		goto fail;

	if (policydb_index_classes(policy))
		goto fail;

	if (policydb_index_others(NULL, policy, 0))
		goto fail;

	if (set_attr("domain", value, policy))
		goto fail;

	VR_LOGI("Created domain '%s'\n", d);

	return src;
fail:
	VR_LOGE("Failed to create domain '%s'\n", d);
	kfree(src);
	return NULL;
}
#endif


/* hashtab traversal macro */
/*
#define hashtab_for_each(table, ptr) \
	for (int _i = 0; _i < table->size; ++_i) \
		for (*ptr = table->htable[_i]; *ptr != NULL; *ptr = (*ptr)->next)
*/

noused notrace static int set_domain_state(const char *s, int state, struct policydb *policy)
{
	struct type_datum *type;
	struct hashtab_node *cur;

	if (s == NULL) {
		u32 i;
		for (i = 0; i < policy->p_types.table->size; ++i)
			for (cur = policy->p_types.table->htable[i]; cur != NULL; cur = cur->next) {
		/*hashtab_for_each(policy->p_types.table, &cur) {*/
			type = cur->datum;
			if (ebitmap_set_bit(&policy->permissive_map, type->value, state)) {
				VR_LOGE("Could not set bit in permissive map\n");
				return -EINVAL;
			}
		}
	} else {
		type = hashtab_search(policy->p_types.table, s);
		if (type == NULL) {
				VR_LOGE("type %s does not exist\n", s);
				return -ENOENT;
		}
		if (ebitmap_set_bit(&policy->permissive_map, type->value, state)) {
			VR_LOGE("Could not set bit in permissive map\n");
			return -EINVAL;
		}
	}

	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define add_irule RS_HIDE(_b_22)
#endif
noused notrace static int add_irule(const int s, const int t, const int c, const int p,
			const int effect, const int not, struct policydb *policy)
{
	int ret = 0;
	unsigned int avd = 1U << (p - 1);
	struct avtab_datum *av, local_av;
	struct avtab_key key;
	struct avtab_node *tmp_node;

	key.source_type = s;
	key.target_type = t;
	key.target_class = c;
	key.specified = effect;

	av = avtab_search(&policy->te_avtab, &key);
	if (!av) {
		memset(&local_av, 0, sizeof(local_av));
		local_av.AVD_DATA |= avd;
		tmp_node = avtab_insert_nonunique(&policy->te_avtab, &key, &local_av);
		if (!tmp_node) {
			ret = -ENOMEM;
			return ret;
		} else if (IS_ERR(tmp_node)) {
			ret = PTR_ERR(tmp_node);
			return ret;
		} else {
			av = &tmp_node->datum;
			ret = 0;
		}

		/*if ((ret = avtab_insert(&policy->te_avtab, &key, &local_av))) {
			VR_LOGE("Error inserting into avtab\n");
			return ret;
		}
		*/
	}

	if (not)
		av->AVD_DATA &= ~avd;
	else
		av->AVD_DATA |= avd;

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define add_rule_auto RS_HIDE(_b_23)
#endif
noused notrace static int add_rule_auto(struct type_datum *src, struct type_datum *tgt,
			struct class_datum *cls, struct perm_datum *perm,
			const int effect, const int not, struct policydb *policy)
{
	int ret = 0;
	u32 i;
	struct hashtab *table;
	struct hashtab_node *cur;

	if (!src) {
		table = policy->p_types.table;
		for (i = 0; i < table->size; i++) {
			cur = table->htable[i];
			while (cur) {
				src = cur->datum;
				ret = add_rule_auto(src, tgt, cls, perm,
							effect, not, policy);
				if (ret)
					return ret;
				cur = cur->next;
			}
		}
	} else if (!tgt) {
		table = policy->p_types.table;
		for (i = 0; i < table->size; i++) {
			cur = table->htable[i];
			while (cur) {
				tgt = cur->datum;
				ret = add_rule_auto(src, tgt, cls, perm,
							effect, not, policy);
				if (ret)
					return ret;
				cur = cur->next;
			}
		}
	} else if (!cls) {
		table = policy->p_classes.table;
		for (i = 0; i < table->size; i++) {
			cur = table->htable[i];
			while (cur) {
				cls = cur->datum;
				ret = add_rule_auto(src, tgt, cls, perm,
							effect, not, policy);
				if (ret)
					return ret;
				cur = cur->next;
			}
		}
	} else if (!perm) {
		table = cls->permissions.table;
		for (i = 0; i < table->size; i++) {
			cur = table->htable[i];
			while (cur) {
				perm = cur->datum;
				ret = add_irule(src->value, tgt->value,
							cls->value, perm->value,
							effect, not, policy);
				if (ret)
					return ret;
				cur = cur->next;
			}
		}

		if (!cls->comdatum)
			return 0;

		table = cls->comdatum->permissions.table;
		for (i = 0; i < table->size; i++) {
			cur = table->htable[i];
			while (cur) {
				perm = cur->datum;
				ret = add_irule(src->value, tgt->value,
							cls->value, perm->value,
							effect, not, policy);
				if (ret)
					return ret;
				cur = cur->next;
			}
		}
	} else {
		return add_irule(src->value, tgt->value, cls->value,
				perm->value, effect, not, policy);
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define add_rule RS_HIDE(_b_24)
#endif
noused notrace static int add_rule(const char *s, const char *t, const char *c, const char *p,
			const int effect, const int not, struct policydb *policy)
{
	int ret = 0;
	struct type_datum *src = NULL, *tgt = NULL;
	struct class_datum *cls = NULL;
	struct perm_datum *perm = NULL;

	if (s) {
		src = hashtab_search(policy->p_types.table, s);
		if (!src) {
			VR_LOGE("Source type '%s' does not exist\n", s);
			ret = -EINVAL;
		}
	}
	if (t) {
		tgt = hashtab_search(policy->p_types.table, t);
		if (!tgt) {
			VR_LOGE("Target type '%s' does not exist\n", t);
			ret = -EINVAL;
		}
	}
	if (c) {
		cls = hashtab_search(policy->p_classes.table, c);
		if (!cls) {
			VR_LOGE("Class '%s' does not exist\n", c);
			ret = -EINVAL;
		}
	}
	if (p && !c) {
		VR_LOGE("No class is specified, cannot add perm '%s'\n", p);
		ret = -EINVAL;
	}

	if (ret)
		return ret;

	if (p) {
		perm = hashtab_search(cls->permissions.table, p);
		if (!perm && cls->comdatum)
			perm = hashtab_search(cls->comdatum->permissions.table, p);

		if (!perm) {
			VR_LOGE("Perm '%s' does not exist in class '%s'\n", p, c);
			return -EINVAL;
		}
	}

	return add_rule_auto(src, tgt, cls, perm, effect, not, policy);
}

/* refer to https://github.com/topjohnwu/magiskpolicy */
noused notrace static int __add_rule(int s, int t, int c, int p, int effect, int not,
	struct policydb *policy)
{
	struct avtab_key key;
	struct avtab_datum *av;
	struct avtab_datum local_av;
	int new_rule = 0;

	key.source_type = s;
	key.target_type = t;
	key.target_class = c;
	key.specified = effect;

	av = avtab_search(&policy->te_avtab, &key);
	if (av == NULL) {
		memset(&local_av, 0, sizeof(local_av));
		av = &local_av;
		new_rule = 1;
	}

	if (not) {
		if (p <= 0)
			av->AVD_DATA = 0U;
		else
			av->AVD_DATA &= ~(1U << (p - 1));
	} else {
		if (p <= 0)
			av->AVD_DATA = ~0U;
		else
			av->AVD_DATA |= 1U << (p - 1);
	}

	if (new_rule) {
		struct avtab_node *tmp_node;
		tmp_node = avtab_insert_nonunique(&policy->te_avtab, &key, av);
		if (!tmp_node) {
			VR_LOGE("Error inserting into avtab\n");
			return -ENOMEM;
		} else if (IS_ERR(tmp_node)) {
			VR_LOGE("Error inserting into avtab: %d\n", PTR_ERR(tmp_node));
			return PTR_ERR(tmp_node);
		}
	}

	return 0;
}

noused notrace static int add_rule_auto_magisk(struct type_datum *src, struct type_datum *tgt,
	struct class_datum *cls, struct perm_datum *perm, int effect, int not, struct policydb *policy)
{
	struct hashtab_node *cur;
	int ret = 0;

	if (src == NULL) {
		u32 i;
		for (i = 0; i < policy->p_types.table->size; ++i)
			for (cur = policy->p_types.table->htable[i]; cur != NULL; cur = cur->next) {
		/*hashtab_for_each(policy->p_types.table, &cur) {*/
			src = cur->datum;
			ret |= add_rule_auto_magisk(src, tgt, cls, perm, effect, not, policy);
		}
	} else if (tgt == NULL) {
		u32 i;
		for (i = 0; i < policy->p_types.table->size; ++i)
			for (cur = policy->p_types.table->htable[i]; cur != NULL; cur = cur->next) {
		/*hashtab_for_each(policy->p_types.table, &cur) {*/
			tgt = cur->datum;
			ret |= add_rule_auto_magisk(src, tgt, cls, perm, effect, not, policy);
		}
	} else if (cls == NULL) {
		u32 i;
		for (i = 0; i < policy->p_classes.table->size; ++i)
			for (cur = policy->p_classes.table->htable[i]; cur != NULL; cur = cur->next) {
		/*hashtab_for_each(policy->p_classes.table, &cur) {*/
			cls = cur->datum;
			ret |= __add_rule(src->value, tgt->value, cls->value, -1, effect, not, policy);
		}
	} else {
		return __add_rule(src->value, tgt->value, cls->value, perm ? perm->value : -1, effect, not, policy);
	}
	return ret;
}

/* refer to https://github.com/topjohnwu/magiskpolicy */

#define ioctl_driver(x) (x>>8 & 0xFF)
#define ioctl_func(x) (x & 0xFF)

#define xperm_test(x, p) (1 & (p[x >> 5] >> (x & 0x1f))) /*  refer to security_xperm_test */
#define xperm_set(x, p) (p[x >> 5] |= (1 << (x & 0x1f))) /* refer to security_xperm_set */
#define xperm_clear(x, p) (p[x >> 5] &= ~(1 << (x & 0x1f)))
#define EXTENDED_PERMS_LEN 8


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __add_xperm_rule RS_HIDE(_b_24_0)
#endif
noused notrace static int __add_xperm_rule(int s, int t, int c, u16 low, u16 high,
	int effect, int not, struct policydb *policy)
{
	struct avtab_key key;
	struct avtab_datum *av;
	struct avtab_datum local_av;
	struct avtab_extended_perms local_xperms = { 0 };
	u32 i;
	int new_rule = 0;

	key.source_type = s;
	key.target_type = t;
	key.target_class = c;
	key.specified = effect;

	av = avtab_search(&policy->te_avtab, &key);
	if (av == NULL) {
		memset(&local_av, 0, sizeof(local_av));
		local_av.AVD_XPERMS = &local_xperms;
		av = &local_av;
		new_rule = 1;
		if (ioctl_driver(low) != ioctl_driver(high)) {
			av->AVD_XPERMS->specified = AVTAB_XPERMS_IOCTLDRIVER;
			av->AVD_XPERMS->driver = 0;
		} else {
			av->AVD_XPERMS->specified = AVTAB_XPERMS_IOCTLFUNCTION;
			av->AVD_XPERMS->driver = ioctl_driver(low);
		}
	}

	if (av->AVD_XPERMS->specified == AVTAB_XPERMS_IOCTLDRIVER) {
		for (i = ioctl_driver(low); i <= ioctl_driver(high); ++i) {
			if (not)
				xperm_clear(i, av->AVD_XPERMS->perms.p);
			else
				xperm_set(i, av->AVD_XPERMS->perms.p);
		}
	} else {
		for (i = ioctl_func(low); i <= ioctl_func(high); ++i) {
			if (not)
				xperm_clear(i, av->AVD_XPERMS->perms.p);
			else
				xperm_set(i, av->AVD_XPERMS->perms.p);
		}
	}

	if (new_rule) {
		struct avtab_node *tmp_node;
		tmp_node = avtab_insert_nonunique(&policy->te_avtab, &key, av);
		if (!tmp_node) {
			VR_LOGE("Error inserting into avtab\n");
			return -ENOMEM;
		} else if (IS_ERR(tmp_node)) {
			VR_LOGE("Error inserting into avtab: %d\n", PTR_ERR(tmp_node));
			return PTR_ERR(tmp_node);
		}
	}

	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define add_xperm_rule_auto RS_HIDE(_b_24_1)
#endif
noused notrace static int add_xperm_rule_auto(struct type_datum *src, struct type_datum *tgt, struct class_datum *cls,
			uint16_t low, uint16_t high, int effect, int not, struct policydb *policy)
{
	u32 i;
	struct hashtab_node *cur;
	int ret = 0;

	if (src == NULL) {
		for (i = 0; i < policy->p_types.table->size; ++i)
			for (cur = policy->p_types.table->htable[i]; cur != NULL; cur = cur->next) {
		/*hashtab_for_each(policy->p_types.table, &cur) {*/
			src = cur->datum;
			ret |= add_xperm_rule_auto(src, tgt, cls, low, high, effect, not, policy);
		}
	} else if (tgt == NULL) {
		for (i = 0; i < policy->p_types.table->size; ++i)
			for (cur = policy->p_types.table->htable[i]; cur != NULL; cur = cur->next) {
		/*hashtab_for_each(policy->p_types.table, &cur) {*/
			tgt = cur->datum;
			ret |= add_xperm_rule_auto(src, tgt, cls, low, high, effect, not, policy);
		}
	} else if (cls == NULL) {
		for (i = 0; i < policy->p_classes.table->size; ++i)
			for (cur = policy->p_classes.table->htable[i]; cur != NULL; cur = cur->next) {
		/*hashtab_for_each(policy->p_classes.table, &cur) {*/
			cls = cur->datum;
			ret |= __add_xperm_rule(src->value, tgt->value, cls->value, low, high, effect, not, policy);
		}
	} else {
		return __add_xperm_rule(src->value, tgt->value, cls->value, low, high, effect, not, policy);
	}
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define add_xperm_rule RS_HIDE(_b_24_2)
#endif
noused notrace static int add_xperm_rule(const char *s, const char *t, const char *c, const char *range, int effect, int not,
	struct policydb *policy)
{
	struct type_datum *src = NULL, *tgt = NULL;
	struct class_datum *cls = NULL;
	u16 low, high;

	if (s) {
		src = hashtab_search(policy->p_types.table, s);
		if (src == NULL) {
			VR_LOGE("source type %s does not exist\n", s);
			return -EINVAL;
		}
	}

	if (t) {
		tgt = hashtab_search(policy->p_types.table, t);
		if (tgt == NULL) {
			VR_LOGE("target type %s does not exist\n", t);
			return -EINVAL;
		}
	}

	if (c) {
		cls = hashtab_search(policy->p_classes.table, c);
		if (cls == NULL) {
			VR_LOGE("class %s does not exist\n", c);
			return -EINVAL;
		}
	}

	if (range) {
		if (strnchr(range, '-', strlen(range))) {
			/*extern unsigned long strtoul(const char *nptr, char **endptr, int base);*/
			sscanf(range, "%hx-%hx", &low, &high);
		} else {
			sscanf(range, "%hx", &low);
			high = low;
		}
	} else {
		low = 0;
		high = 0xFFFF;
	}

	return add_xperm_rule_auto(src, tgt, cls, low, high, effect, not, policy);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define add_typerule RS_HIDE(_b_25)
#endif
noused notrace static int add_typerule(const char *s, const char *targetAttribute, char **minuses, const char *c,
			const char *p, const int effect, const int not, struct policydb *policy)
{
	int ret = 0;
	u32 i;
	struct type_datum *src, *tgt, *obj;
	struct class_datum *cls;
	struct perm_datum *perm;
	struct ebitmap *attrs;

	/* 64(0kB) should be enough for everyone, right? */
	int m[64] = { -1 };

	src = hashtab_search(policy->p_types.table, s);
	if (!src) {
		VR_LOGE("Source type '%s' does not exist\n", s);
		ret = -EINVAL;
	}
	tgt = hashtab_search(policy->p_types.table, targetAttribute);
	if (!tgt) {
		VR_LOGE("Target type '%s' does not exist\n", targetAttribute);
		ret = -EINVAL;
	}
	cls = hashtab_search(policy->p_classes.table, c);
	if (!cls) {
		VR_LOGE("Class '%s' does not exist\n", c);
		ret = -EINVAL;
	}

	if (ret)
		return ret;

	/*if (!tgt->attribute) //if (tgt->flavor != TYPE_ATTRIB)
		return -EINVAL;
	*/

	for (i = 0; minuses && minuses[i]; ++i) {
		obj = hashtab_search(policy->p_types.table,
						minuses[i]);
		if (!obj) {
			VR_LOGE("Minus type '%s' does not exist\n", minuses[i]);
			return -EINVAL;
		}
		m[i] = obj->value-1;
		m[i+1] = -1;
	}

	if (!cls->comdatum) {
		VR_LOGE("Class '%s' has no permissions table\n", c);
		return -EINVAL;
	}

	perm = hashtab_search(cls->permissions.table, p);
	if (!perm) {
		perm = hashtab_search(
				cls->comdatum->permissions.table, p);
		if (!perm) {
			VR_LOGE("Perm '%s' does not exist in class '%s'\n", p, c);
			return -EINVAL;
		}
	}

	for (i = 0; i < policy->p_types.nprim; i++) {
		attrs = flex_array_get(policy->type_attr_map_array, i);
		if ((attrs) && (ebitmap_get_bit(attrs, tgt->value-1))) {
			int found = 0;
			unsigned int j;

			for (j = 0; m[j] != -1; j++) {
				if (i != (unsigned int)m[j])
					continue;
				found = 1;
				break;
			}
			if (found)
				continue;

			ret |= add_irule(src->value, i + 1, cls->value,
					perm->value, effect, not, policy);
		}
	}

#if 0
	types = flex_array_get(policy->attr_type_map_array, tgt->value-1);

	ebitmap_for_each_positive_bit(types, node, i)
		if (ebitmap_node_get_bit(node, i)) {
			int found = 0;
			unsigned int j;

			for (j = 0; m[j] != -1; j++) {
				if (i != (unsigned int)m[j])
					continue;
				found = 1;
				break;
			}
			if (found)
				continue;

			ret |= add_irule(src->value, i + 1, cls->value,
					 perm->value, effect, not, policy);
		}
#endif

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define add_transition RS_HIDE(_b_26)
#endif
noused notrace static int add_transition(const char *srcS, const char *origS, const char *tgtS,
			const char *c, struct policydb *policy)
{
	int ret = 0;
	struct type_datum *src, *tgt, *orig;
	struct class_datum *cls;
	struct avtab_datum *av, local_av;
	struct avtab_key key;
	struct avtab_node *tmp_node;

	src = hashtab_search(policy->p_types.table, srcS);
	if (!src) {
		VR_LOGE("Source type '%s' does not exist\n", srcS);
		ret = -EINVAL;
	}
	orig = hashtab_search(policy->p_types.table, origS);
	if (!orig) {
		VR_LOGE("Origin type '%s' does not exist\n", origS);
		ret = -EINVAL;
	}
	tgt = hashtab_search(policy->p_types.table, tgtS);
	if (!tgt) {
		VR_LOGE("Target type '%s' does not exist\n", tgtS);
		ret = -EINVAL;
	}
	cls = hashtab_search(policy->p_classes.table, c);
	if (!cls) {
		VR_LOGE("Class '%s' does not exist\n", c);
		ret = -EINVAL;
	}

	if (ret)
		return ret;

	key.source_type  = src->value;
	key.target_type  = orig->value;
	key.target_class = cls->value;
	key.specified    = AVTAB_TRANSITION;

	av = avtab_search(&policy->te_avtab, &key);
	if (!av) {
		memset(&local_av, 0, sizeof(local_av));
		local_av.AVD_DATA = tgt->value;

		tmp_node = avtab_insert_nonunique(&policy->te_avtab, &key, &local_av);
		if (!tmp_node) {
			VR_LOGE("Error inserting into avtab\n");
			ret = -ENOMEM;
		} else if (IS_ERR(tmp_node)) {
			ret = PTR_ERR(tmp_node);
			VR_LOGE("Error inserting into avtab: %d\n", ret);
		} else {
			ret = 0;
		}

		/*if ((ret = avtab_insert(&policy->te_avtab, &key, &local_av))) {
			VR_LOGE("Error inserting into avtab");
			return ret;
		}
		*/
	} else {
		VR_LOGE("Warning, rule already defined! Won't override.\n");
		VR_LOGE("Previous value = %d, wanted value = %d\n",
			av->AVD_DATA, tgt->value);
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define add_file_transition RS_HIDE(_b_27)
#endif
noused notrace static int add_file_transition(const char *srcS, const char *origS, const char *tgtS,
			const char *c, const char *filename,
			struct policydb *policy)
{
	int ret = 0;
	struct type_datum *src, *tgt, *orig;
	struct class_datum *cls;
	struct filename_trans *new_transition;
	struct filename_trans_datum *otype;

	src = hashtab_search(policy->p_types.table, srcS);
	if (!src) {
		VR_LOGE("Source type '%s' does not exist\n", srcS);
		ret = -EINVAL;
	}
	orig = hashtab_search(policy->p_types.table, origS);
	if (!orig) {
		VR_LOGE("Origin type '%s' does not exist\n", origS);
		ret = -EINVAL;
	}
	tgt = hashtab_search(policy->p_types.table, tgtS);
	if (!tgt) {
		VR_LOGE("Target type '%s' does not exist\n", tgtS);
		ret = -EINVAL;
	}
	cls = hashtab_search(policy->p_classes.table, c);
	if (!cls) {
		VR_LOGE("Class '%s' does not exist\n", c);
		ret = -EINVAL;
	}

	if (ret)
		goto out;

	new_transition = kzalloc(sizeof(*new_transition), GFP_KERNEL);
	if (!new_transition) {
		ret = -ENOMEM;
		goto out;
	}

	otype = kmalloc(sizeof(*otype), GFP_KERNEL);
	if (!otype) {
		kfree(new_transition);

		ret = -ENOMEM;
		goto out;
	}
	new_transition->stype  = src->value;
	new_transition->ttype  = orig->value;
	/* new_transition->otype  = tgt->value; */
	new_transition->tclass = cls->value;
	new_transition->name   = kstrdup(filename, GFP_KERNEL);
	/* new_transition->next   = policy->filename_trans; */

	/* policy->filename_trans = new_transition; */
	otype->otype = tgt->value;

	ret = ebitmap_set_bit(&policy->filename_trans_ttypes, new_transition->ttype, 1);
	if (ret) {
		kfree(new_transition->name);
		kfree(otype);
		kfree(new_transition);
		goto out;
	}

	ret = hashtab_insert(policy->filename_trans, new_transition, otype);
	if (ret) {
		/*
		 * Do not return -EEXIST to the caller, or the system
		 * will not boot.
		 */
		if (ret != -EEXIST)
			goto out;

		/* But free memory to avoid memory leak. */
		kfree(new_transition->name);
		kfree(otype);
		kfree(new_transition);

		ret = 0;
	}

out:
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define add_type RS_HIDE(_b_28)
#endif
/* equal to add_typeattribute() */
noused notrace static int add_type(const char *domainS, const char *typeS, struct policydb *policy)
{
	u32 typeId;
	u32 i;
	struct type_datum *domain;
	struct class_datum *cl;
	struct constraint_node *n;
	struct constraint_expr *e;

	domain = hashtab_search(policy->p_types.table, domainS);
	if (!domain) {
		VR_LOGE("Domain '%s' does not exist\n", domainS);
		return -EINVAL;
	}

	set_attr(typeS, domain->value, policy);

	typeId = get_attr_id(typeS, policy);
	if (!typeId)
		return -EINVAL;

	/* Now let's update all constraints!
	 * (kernel doesn't support (yet?) type_names rules)
	 */
	for (i = 0; i < policy->p_classes.nprim; i++) {
		cl = policy->class_val_to_struct[i];
		for (n = cl->constraints; n; n = n->next) {
			for (e = n->expr; e; e = e->next) {
				if (e->expr_type != CEXPR_NAMES)
					continue;
				if (ebitmap_get_bit(&e->type_names->types, typeId - 1)) {
					int err = ebitmap_set_bit(&e->names, domain->value - 1, 1);
					if (err) {
						return err;
					}
				}
			}
		}
	}

	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define del_type RS_HIDE(_b_29)
#endif
/* equal to add_typeattribute() */
noused notrace static int del_type(const char *domainS, const char *typeS, struct policydb *policy)
{
	u32 typeId;
	u32 i;
	struct type_datum *domain;
	struct class_datum *cl;
	struct constraint_node *n;
	struct constraint_expr *e;

	domain = hashtab_search(policy->p_types.table, domainS);
	if (!domain) {
		VR_LOGE("Domain '%s' does not exist\n", domainS);
		return -EINVAL;
	}

	clear_attr(typeS, domain->value, policy);

	typeId = get_attr_id(typeS, policy);
	if (!typeId)
		return -EINVAL;

	/* Now let's update all constraints!
	 * (kernel doesn't support (yet?) type_names rules)
	 */
	for (i = 0; i < policy->p_classes.nprim; i++) {
		cl = policy->class_val_to_struct[i];
		for (n = cl->constraints; n; n = n->next) {
			for (e = n->expr; e; e = e->next) {
				if (e->expr_type != CEXPR_NAMES)
					continue;
				if (ebitmap_get_bit(&e->type_names->types, typeId - 1)) {
					int err = ebitmap_set_bit(&e->names, domain->value - 1, 0);
					if (err) {
						return err;
					}
				}
			}
		}
	}

	return 0;
}

enum {
	tasNone = 0,
	tasAttrChanged = 1,
	tasConstraintsChanged = 2,
} TypeAttributeStatus;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_policydb_update_typeattribute RS_HIDE(_b_30)
#endif
noused notrace static int vr_policydb_update_typeattribute(struct policydb *policy,
	const char *type_name, const char *attr_name, int is_clear, int *old_attr_status,
	int *updated_ptr)
{
	u32 typeId;
	u32 i;
	struct type_datum *type;
	struct class_datum *cl;
	struct constraint_node *n;
	struct constraint_expr *e;
	int old_attr_changed, old_constrains_changed;
	int curr_attr_is_set;
	int old_status, new_status = 0;
	int updated = 0;
	int err = 0;

	VR_LOGV("upd attr,%s,%s,%d\n", type_name, attr_name, is_clear);

	type = hashtab_search(policy->p_types.table, type_name);
	if (!type) {
		VR_LOGE("type '%s' does not exist\n", type_name);
		err = -ENOENT;
		goto out;
	}

	curr_attr_is_set = attr_is_set(attr_name, type->value, policy);
	if (curr_attr_is_set < 0) {
		VR_LOGE("attr '%s' does not exist\n", type_name);
		err = -ENOENT;
		goto out;
	}

	if (old_attr_status)
		old_status = *old_attr_status;
	else
		old_status = 0;

	old_attr_changed = (old_status & tasAttrChanged) ? 1 : 0;
	old_constrains_changed = (old_status & tasConstraintsChanged) ? 1 : 0;

	if (is_clear) {
		if (old_attr_changed) {
			if (curr_attr_is_set) {
				clear_attr(attr_name, type->value, policy);
			}
			updated = 1;
			new_status &= ~tasAttrChanged;

			VR_LOGV("clr attr,%d\n", new_status);
		} else {
			VR_LOGV("skip patch attr\n");
		}

	} else {
		if (!curr_attr_is_set) {
			set_attr(attr_name, type->value, policy);
			new_status |= tasAttrChanged;
			updated = 1;

			VR_LOGV("set attr,%d\n", new_status);
		} else {
			VR_LOGV("skip patch attr\n");
		}
	}

	typeId = get_attr_id(attr_name, policy);
	if (!typeId) {
		VR_LOGE("attr '%s' typeid does not exist\n", attr_name);
		err = -EINVAL;
		goto out;
	}

	/* Now let's update all constraints!
	 * (kernel doesn't support (yet?) type_names rules)
	 */
	if (is_clear) {
		if (old_constrains_changed) {
			int changed = 0;

			for (i = 0; i < policy->p_classes.nprim; i++) {
				cl = policy->class_val_to_struct[i];
				for (n = cl->constraints; n; n = n->next) {
					for (e = n->expr; e; e = e->next) {
						if (e->expr_type != CEXPR_NAMES)
							continue;
						if (ebitmap_get_bit(&e->type_names->types, typeId - 1)) {
							if (ebitmap_get_bit(&e->names, type->value - 1)) {
								err = ebitmap_set_bit(&e->names, type->value - 1, 0);
								if (err) {
									goto out;
								}
							}
							changed = 1;
						}
					}
				}
			}

			if (changed) {
				updated |= 2;
				new_status &= ~tasConstraintsChanged;

				VR_LOGV("clr constraints,%d\n", new_status);
			}
		}
	} else {
		int changed = 0;

		for (i = 0; i < policy->p_classes.nprim; i++) {
			cl = policy->class_val_to_struct[i];
			for (n = cl->constraints; n; n = n->next) {
				for (e = n->expr; e; e = e->next) {
					if (e->expr_type != CEXPR_NAMES)
						continue;
					if (ebitmap_get_bit(&e->type_names->types, typeId - 1)) {
						if (!ebitmap_get_bit(&e->names, type->value - 1)) {
							err = ebitmap_set_bit(&e->names, type->value - 1, 1);
							if (err) {
								goto out;
							}
							changed = 1;
						}
					}
				}
			}
		}

		if (changed) {
			updated |= 2;
			new_status |= tasConstraintsChanged;

			VR_LOGV("set constraints,%d\n", new_status);
		}
	}

out:
	if (old_attr_status)
		*old_attr_status = new_status;

	if (updated_ptr)
		*updated_ptr = updated;

	if (err > 0)
		err = -err;

	return err;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define vr_policydb_update_typerule RS_HIDE(_c_1)
#endif
noused notrace static int vr_policydb_update_typerule(struct policydb *policydb_ptr,
	const char *sourceAttribute,
	const int *source_minuses,
	const char *target, class_type _class,
	permissions_type permissions,
#if defined(VR_USE_MAPPING)
	struct selinux_map *map,
#endif
	u32 rule_type, int is_clear, int *updated_ptr)
{
	int ret = 0;
	int updated = 0;
	struct type_datum *src, *tgt;
	struct ebitmap *attrs;

	/* 16(0kB) should be enough for everyone, right? */
	u32 minus_types[16];
	u32 minus_count;
#if defined(VR_RULE_USE_STR_CLASS)
#if defined(VR_FIND_RULE_USE_SYS_ROUTINE)
	u32 tclass;
#else
	struct class_datum *cls;
#endif
#else
	u32 unmapped_class;
	struct class_datum *cls;
#endif
	struct avtab_datum *av;
	struct avtab_key key;
#if defined(VR_RULE_USE_STR_PERM)
	char token_buffer[32];
	u32 perms_value = 0;
	const char *perm_token = NULL;
	const char *perm_saveptr = NULL;

	#define PERMS_VALUE perms_value

	perm_token = strtok_r_rdonly(permissions, ",", &perm_saveptr);
	if (!perm_token) {
		ret = -EINVAL;
		VR_LOG("permissions is empty\n");
		goto out;
	}
#else
	#define PERMS_VALUE permissions

	if (!permissions) {
		ret = -EINVAL;
		VR_LOG("permissions is empty\n");
		goto out;
	}
#endif

#if !defined(VR_RULE_USE_STR_CLASS)
#if defined(VR_USE_MAPPING)
	unmapped_class = vr_unmap_class(map, _class);
	if (!unmapped_class
		|| unmapped_class > policydb_ptr->p_classes.nprim) {
		ret = -EINVAL;
		VR_LOG("class %u does not exist\n", _class);
		goto out;
	}

	cls = policydb_ptr->class_val_to_struct[unmapped_class - 1];
#else
	cls = vr_unmap_class_no_map_ex(policydb_ptr, _class,
		&unmapped_class);
	if (cls == NULL) {
		ret = -EINVAL;
		VR_LOG("class %u does not exist\n", _class);
		goto out;
	}
#endif

	#define CLASS_VALUE cls->value
#endif

	src = hashtab_search(policydb_ptr->p_types.table, sourceAttribute);
	if (src == NULL) {
		ret = -EINVAL;
		VR_LOG("source attribute %s does not exist\n", sourceAttribute);
		goto out;
	}

#if 0
	/*if (src->flavor != TYPE_ATTRIB) {*/
	if (!src->attribute) {
		ret = -EINVAL;
		VR_LOG("source attribute %s is not attribute\n", sourceAttribute);
		goto out;
	}
#endif

	tgt = hashtab_search(policydb_ptr->p_types.table, target);
	if (tgt == NULL) {
		ret = -EINVAL;
		VR_LOG("target type %s does not exist\n", target);
		goto out;
	}

#if defined(VR_RULE_USE_STR_CLASS)
#if defined(VR_FIND_RULE_USE_SYS_ROUTINE)
	tclass = string_to_security_class(policydb_ptr, _class);
	if (!tclass || tclass > policydb_ptr->p_classes.nprim) {
		ret = -EINVAL;
		VR_LOG("class %u does not exist\n", _class);
		goto out;
	}

	#define CLASS_VALUE tclass
#else
	cls = hashtab_search(policydb_ptr->p_classes.table, _class);
	if (cls == NULL) {
		ret = -EINVAL;
		VR_LOG("class %s does not exist\n", _class);
		goto out;
	}

	#define CLASS_VALUE cls->value
#endif
#endif

#if defined(VR_RULE_USE_STR_PERM)
	while (perm_token) {
		int len;

		if (perm_saveptr)
			len = perm_saveptr - 1 - perm_token;
		else
			len = strlen(perm_token);

		if (len < (int)sizeof(token_buffer)) {
			u32 v;
		#if !defined(VR_FIND_RULE_USE_SYS_ROUTINE)
			struct perm_datum *perm;
		#endif

			memcpy(token_buffer, perm_token, len);
			token_buffer[len] = '\0';

		#if defined(VR_FIND_RULE_USE_SYS_ROUTINE)
			v = string_to_av_perm(policydb_ptr, tclass, token_buffer);
		#else
			perm = hashtab_search(cls->permissions.table, token_buffer);
			if (perm == NULL) {
				if (cls->comdatum == NULL) {
				#if defined(VR_RULE_USE_STR_CLASS)
					VR_LOG("perm %s does not exist in class %s\n",
						token_buffer, _class);
				#else
					VR_LOG("perm %s does not exist in class %x\n",
						token_buffer, _class);
				#endif

					v = 0;
				} else {
					perm = hashtab_search(cls->comdatum->permissions.table,
						token_buffer);
					if (perm == NULL) {
					#if defined(VR_RULE_USE_STR_CLASS)
						VR_LOG("perm %s does not exist in class %s\n",
							token_buffer, _class);
					#else
						VR_LOG("perm %s does not exist in class %x\n",
							token_buffer, _class);
					#endif

						v = 0;
					} else {
						v = (1U << (perm->value - 1));
					}
				}
			} else {
				v = (1U << (perm->value - 1));
			}

		#endif

			perms_value |= v;
		}

		/*
		if ((perm_saveptr) && (!perm_saveptr[-1]))
			perm_saveptr[-1] = ',';
		*/

		perm_token = strtok_r_rdonly(NULL, ",", &perm_saveptr);
	}

	if (!perms_value) {
		ret = -EINVAL;
		goto out;
	}
#else
#if defined(VR_USE_MAPPING)
	PERMS_VALUE = vr_unmap_perms(map, _class, PERMS_VALUE);
#else
	PERMS_VALUE = vr_unmap_perms_no_map_ex(policydb_ptr, _class, PERMS_VALUE, NULL);
#endif
#endif

	if (source_minuses) {
		size_t i;
	#if 1
		for (i = 0; (i < ARRAY_SIZE(minus_types)) && (source_minuses[i] < vrseInvalid); i++) {
			struct type_datum *obj;

			obj = hashtab_search(policydb_ptr->p_types.table,
				g_vr_strings[source_minuses[i]]);
			if (!obj) {
				ret = -EINVAL;
				VR_LOGE("Minus type '%s' does not exist\n", g_vr_strings[source_minuses[i]]);
				goto out;
			}

			minus_types[i] = obj->value - 1;
		}
	#else
		for (i = 0; (i < ARRAY_SIZE(minus_types)) && (source_minuses[i]) && (source_minuses[i][0]); i++) {
			struct type_datum *obj;

			obj = hashtab_search(policydb_ptr->p_types.table, source_minuses[i]);
			if (!obj) {
				ret = -EINVAL;
				VR_LOGE("Minus type '%s' does not exist\n", source_minuses[i]);
				goto out;
			}

			minus_types[i] = obj->value - 1;
		}
	#endif
		minus_count = i;
	} else {
		minus_count = 0;
	}

	{
		u32 type_idx;
		u32 attr_val = src->value - 1;

		for (type_idx = 0; type_idx < policydb_ptr->p_types.nprim; type_idx++) {
			attrs = flex_array_get(policydb_ptr->type_attr_map_array, type_idx);
			if ((attrs) && (ebitmap_get_bit(attrs, attr_val))) {
				if (minus_count) {
					int found = 0;
					u32 j;

					for (j = 0; j < minus_count; j++) {
						if (type_idx == minus_types[j]) {
							found = 1;
							break;
						}
					}

					if (found)
						continue;
				}

				/* See if there is already a rule */
				key.source_type = type_idx + 1;
				key.target_type = tgt->value;
				key.target_class = CLASS_VALUE;
				key.specified = rule_type; /*AVTAB_ALLOWED/AVTAB_AUDITDENY/AVTAB_AUDITALLOW/AVTAB_TRANSITION/AVTAB_CHANGE/AVTAB_MEMBER*/

				av = avtab_search(&policydb_ptr->te_avtab, &key);

				if (av == NULL) {
					struct avtab_datum local_av;
					struct avtab_node *tmp_node;

					if (is_clear)
						local_av.AVD_DATA = 0xffffffff & ~PERMS_VALUE;
					else
						local_av.AVD_DATA = PERMS_VALUE;

					/*
					ret = avtab_insert(&policydb_ptr->te_avtab, &key, &local_av);
					if (ret)
					*/
					tmp_node = avtab_insert_nonunique(&policydb_ptr->te_avtab,
						&key, &local_av);
					if (!tmp_node) {
						ret = -ENOMEM;
						VR_LOG("Error inserting into avtab\n");
						goto out;
					} else if (IS_ERR(tmp_node)) {
						ret = PTR_ERR(tmp_node);
						VR_LOG("Error inserting into avtab: %d\n", ret);
						goto out;
					}

					updated = 1;
				} else {
					if (is_clear) {
						if (av->AVD_DATA & PERMS_VALUE) {
							av->AVD_DATA &= ~PERMS_VALUE;

							updated = 1;
						}
					} else {
						if ((av->AVD_DATA & PERMS_VALUE) != PERMS_VALUE) {
							av->AVD_DATA |= PERMS_VALUE;
							updated = 1;
						}
					}
				}

				/*
				ret |= add_irule(type_idx + 1, tgt->value, cls->value,
						perm->value, effect, not, policy);
				*/
			}
		}
	}

out:
	if (updated_ptr)
		*updated_ptr |= updated;

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sepol_allow RS_HIDE(_c_6)
#endif
noused notrace static int sepol_allow(const char *s, const char *t, const char *c, const char *p, struct policydb *policy)
{
	/*VR_LOGV("allow %s %s %s %s\n", s, t, c, p);*/
	return add_rule(s, t, c, p, AVTAB_ALLOWED, 0, policy);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sepol_deny RS_HIDE(_c_7)
#endif
noused notrace static int sepol_deny(const char *s, const char *t, const char *c, const char *p, struct policydb *policy)
{
	/*VR_LOGV("deny %s %s %s %s\n", s, t, c, p);*/
	return add_rule(s, t, c, p, AVTAB_ALLOWED, 1, policy);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sepol_auditallow RS_HIDE(_c_8)
#endif
int sepol_auditallow(const char *s, const char *t, const char *c, const char *p, struct policydb *policy)
{
	/*VR_LOGV("auditallow %s %s %s %s\n", s, t, c, p);*/
	return add_rule(s, t, c, p, AVTAB_AUDITALLOW, 0, policy);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sepol_auditdeny RS_HIDE(_c_9)
#endif
noused notrace static int sepol_auditdeny(const char *s, const char *t, const char *c, const char *p,
	struct policydb *policy)
{
	/*VR_LOGV("auditdeny %s %s %s %s\n", s, t, c, p);*/
	return add_rule(s, t, c, p, AVTAB_AUDITDENY, 0, policy);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sepol_typetrans RS_HIDE(_c_a)
#endif
noused notrace static int sepol_typetrans(const char *s, const char *t, const char *c, const char *d, const char *o,
	struct policydb *policy)
{
	if (o == NULL) {
		/*VR_LOGV("add_trans %s %s %s %s\n", s, t, c ,d);*/
		return add_transition(s, t, c, d, policy);
	} else {
		/*VR_LOGV("add_file_trans %s %s %s %s %s\n", s, t, c ,d, o);*/
		return add_file_transition(s, t, c, d, o, policy);
	}
}

#if 1
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sepol_allowxperm RS_HIDE(_c_b)
#endif
noused notrace static int sepol_allowxperm(const char *s, const char *t, const char *c, const char *range,
	struct policydb *policy)
{
	/*VR_LOGV("allowxperm %s %s %s %s\n", s, t, c, range);*/
	return add_xperm_rule(s, t, c, range, AVTAB_XPERMS_ALLOWED, 0, policy);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sepol_auditallowxperm RS_HIDE(_c_c)
#endif
noused notrace static int sepol_auditallowxperm(const char *s, const char *t, const char *c, const char *range,
	struct policydb *policy)
{
	/*VR_LOGV("auditallowxperm %s %s %s %s\n", s, t, c, range);*/
	return add_xperm_rule(s, t, c, range, AVTAB_XPERMS_AUDITALLOW, 0, policy);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sepol_dontauditxperm RS_HIDE(_c_d)
#endif
noused notrace static int sepol_dontauditxperm(const char *s, const char *t, const char *c, const char *range,
	struct policydb *policy)
{
	/*VR_LOGV("dontauditxperm %s %s %s %s\n", s, t, c, range);*/
	return add_xperm_rule(s, t, c, range, AVTAB_XPERMS_DONTAUDIT, 0, policy);
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sepol_permissive RS_HIDE(_c_e)
#endif
noused notrace static int sepol_permissive(const char *s, struct policydb *policy)
{
	/*VR_LOGV("permissive %s\n", s);*/
	return set_domain_state(s, 1, policy);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sepol_enforce RS_HIDE(_c_f)
#endif
noused notrace static int sepol_enforce(const char *s, struct policydb *policy)
{
	/*VR_LOGV("enforce %s\n", s);*/
	return set_domain_state(s, 0, policy);
}

#if 0
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sepol_create RS_HIDE(_c_g)
#endif
noused notrace static int sepol_create(const char *s, struct policydb *policy)
{
	/*VR_LOGV("create %s\n", s);*/
	return create_domain(s, policy);
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sepol_attradd RS_HIDE(_c_h)
#endif
noused notrace static int sepol_attradd(const char *s, const char *a, struct policydb *policy)
{
	/*VR_LOGV("attradd %s %s\n", s, a);*/
	/*return add_typeattribute(s, a, policy);*/
	return add_type(s, a, policy);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sepol_attrdel RS_HIDE(_c_i)
#endif
noused notrace static int sepol_attrdel(const char *s, const char *a, struct policydb *policy)
{
	/*VR_LOGV("attradd %s %s\n", s, a);*/
	/*return add_typeattribute(s, a, policy);*/
	return del_type(s, a, policy);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sepol_exists RS_HIDE(_c_j)
#endif
noused notrace static int sepol_exists(const char *source, struct policydb *policy)
{
	return !!hashtab_search(policy->p_types.table, source);
}

noused notrace static int update_rules_for_8_0_above(struct policydb *policy)
{
	int ret;

	ret = sepol_allow("hwservicemanager", "su", "dir", "search", policy);
	ret = sepol_allow("hwservicemanager", "su", "file", "read", policy);
	ret = sepol_allow("hwservicemanager", "su", "file", "open", policy);
	ret = sepol_allow("hwservicemanager", "su", "process", "getattr", policy);
	ret = sepol_allow("hwservicemanager", "su", "binder", "call", policy);
	ret = sepol_allow("hwservicemanager", "su", "binder", "transfer", policy);

  /* system/sepolicy/private/su.te:
  typeattribute su coredomain;
  app_domain(su)
  permissive su;
 */
	ret = sepol_attradd("su", "coredomain", policy);
	VR_LOGV("ur4o,1,%d\n", ret);
	ret = sepol_attradd("su", "appdomain", policy);
	VR_LOGV("ur4o,2,%d\n", ret);

/* system/sepolicy/public/su.te:
  typeattribute su mlstrustedsubject;
  net_domain(su)
  vndbinder_use(su)
*/
	ret = sepol_attradd("su", "mlstrustedsubject", policy);
	VR_LOGV("ur4o,3,%d\n", ret);
	ret = sepol_attradd("su", "netdomain", policy);
	VR_LOGV("ur4o,4,%d\n", ret);

#if 0
	ret = sepol_allow("vndservicemanager", "su", "dir", "search", policy);
	ret = sepol_allow("vndservicemanager", "su", "file", "read", policy);
	ret = sepol_allow("vndservicemanager", "su", "file", "open", policy);
	ret = sepol_allow("vndservicemanager", "su", "process", "getattr", policy);
#endif

#if 0
/*allow installd su:binder call;*/
	sepol_allow("installd", "su", "binder", "call");
/*unix_socket_send(hal_wifi_supplicant, wpa, su)*/
	/*allow hal_wifi_supplicant su:unix_dgram_socket sendto;*/
	sepol_allow("hal_wifi_supplicant", "su", "unix_dgram_socket", "sendto", policy);

/*
  allow domain su:unix_stream_socket connectto;
  allow domain su:fd use;
  allow domain su:unix_stream_socket { getattr getopt read write shutdown };
*/
	sepol_allow(NULL, "su", "unix_stream_socket", "connectto", policy);
	sepol_allow(NULL, "su", "fd", "use", policy);
	sepol_allow(NULL, "su", "unix_stream_socket", "getattr", policy);
	sepol_allow(NULL, "su", "unix_stream_socket", "getopt", policy);
	sepol_allow(NULL, "su", "unix_stream_socket", "read", policy);
	sepol_allow(NULL, "su", "unix_stream_socket", "write", policy);
	sepol_allow(NULL, "su", "unix_stream_socket", "shutdown", policy);

/*
  allow { domain -init } su:binder { call transfer };
  allow { domain -init } su:fd use;
  allow domain su:fifo_file { write getattr };
*/
	sepol_allow(NULL, "su", "binder", "call", policy);
	sepol_allow(NULL, "su", "binder", "transfer", policy);
/*  allow domain su:fifo_file { write getattr };*/
	sepol_allow(NULL, "su", "fifo_file", "write", policy);
	sepol_allow(NULL, "su", "fifo_file", "getattr", policy);

/*  allow domain su:process sigchld; */
	sepol_allow(NULL, "su", "process", "sigchld", policy);
#endif

	return ret;
}

#endif
