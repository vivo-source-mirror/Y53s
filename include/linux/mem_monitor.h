/*
 * include/linux/mem_monitor.h
 *
 * VIVO Kernel Memory Monitor
 *
 * <rongqianfeng@vivo.com>
 *
*/

#ifndef _MEM_MONITOR_H
#define _MEM_MONITOR_H

#ifdef CONFIG_SLUB_DEBUG

#ifdef CONFIG_DEBUG_KMEMLEAK
extern int kmemleak_skip_disable;
#endif /*CONFIG_DEBUG_KMEMLEAK*/

#endif /*CONFIG_SLUB_DEBUG*/

#endif