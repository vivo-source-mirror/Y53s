
/*
* could not BINDER_LIMIT_NORMAL and BINDER_LIMIT_AGGRESS are true.
* they are mutual exclusion.
* see rsc_settting_init ->
* BUILD_BUG_ON(ENABLE_BINDER_LIMIT_NORMAL && ENABLE_BINDER_LIMIT_AGGRESS);
*/
#ifndef ENABLE_BINDER_LIMIT_NORMAL
#define ENABLE_BINDER_LIMIT_NORMAL false
#endif
#ifndef ENABLE_BINDER_LIMIT_AGGRESS
/*CONFIG_RSC_APP_LAUNCH_BOOST*/
/*#define ENABLE_BINDER_LIMIT_AGGRESS true*/
#define ENABLE_BINDER_LIMIT_AGGRESS false
#endif

/*
* limit only all task is bg,
* A(bg) -> B(bg) ->C(bg) -> system_server binder
* could not BINDER_LIMIT_NORMAL and BINDER_LIMIT_AGGRESS are true.
* see rsc_settting_init ->
* BUILD_BUG_ON(ENABLE_BINDER_LIMIT_NORMAL && ENABLE_BINDER_LIMIT_AGGRESS);
*/
RSC_FEAT(BINDER_LIMIT_NORMAL, ENABLE_BINDER_LIMIT_NORMAL)

/* promote static vip task, struct_struct -> rsc_svp > 0 */
RSC_FEAT(BINDER_PROMOTE_NORMAL, false)

/*
* limit  when the first task is bg,
* A(bg) -> B(fg) ->C(fg) -> system_server binder
* could not BINDER_LIMIT_NORMAL and BINDER_LIMIT_AGGRESS are true.
* see rsc_settting_init ->
* BUILD_BUG_ON(ENABLE_BINDER_LIMIT_NORMAL && ENABLE_BINDER_LIMIT_AGGRESS);
*/
RSC_FEAT(BINDER_LIMIT_AGGRESS, ENABLE_BINDER_LIMIT_AGGRESS)

/* promote all fg task and svp task */
RSC_FEAT(BINDER_PROMOTE_AGGRESS, false)

/* fast task exit 0x10 */
RSC_FEAT(BOOST_TASKKILL, true)

#ifdef CONFIG_RSC_APP_LAUNCH_BOOST
/* limit except fg task, 0x40*/
RSC_FEAT(BINDER_LIMIT_EXCEPT_FG_TASK, false)

/*0x80*/
RSC_FEAT(APP_LAUNCH_BOOST, true)

/*0x100, true means mainthread setting by echo 3000 uid pid > /sys/rsc/launch*/
RSC_FEAT(APP_LAUNCH_BY_FRAMEWORK, true)

/*0x200*/
RSC_FEAT(APP_LAUNCH_BINDER_BOOST, true)
#endif

#ifdef CONFIG_RSC_SHOW_IO_SCHEDULE_IN_SYSTRACE
/*0x800*/
RSC_FEAT(SCHED_SLEEPING_BLOCKED_REASON_ENABLE, false)
#else
/*placeholder only*/
RSC_FEAT(SCHED_SLEEPING_BLOCKED_REASON_ENABLE, false)
#endif

#ifdef CONFIG_RSC_LOCK_BOOST
/*0x1000*/
RSC_FEAT(RSC_LOCK_BOOST_ENABLE, true)
RSC_FEAT(RSC_BINDER_BOOST_ENABLE, true)
RSC_FEAT(RSC_LOCK_BOOST_FUTEX_ENABLE, true)
RSC_FEAT(RSC_LOCK_BOOST_TO_BIGCORE, false)
RSC_FEAT(RSC_LOCK_NOT_BOOST_RENDER, false)
#else
/*placeholder only*/
RSC_FEAT(RSC_LOCK_BOOST_ENABLE, false)
RSC_FEAT(RSC_BINDER_BOOST_ENABLE, false)
RSC_FEAT(RSC_LOCK_BOOST_FUTEX_ENABLE, false)
RSC_FEAT(RSC_LOCK_BOOST_TO_BIGCORE, false)
RSC_FEAT(RSC_LOCK_NOT_BOOST_RENDER, false)
#endif