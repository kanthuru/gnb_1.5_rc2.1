/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_COREDUMP_H
#define _LINUX_COREDUMP_H

#include <linux/types.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <asm/siginfo.h>
#include <linux/uio_driver.h>

struct core_vma_metadata {
	unsigned long start, end;
	unsigned long flags;
	unsigned long dump_size;
};

/*
 * These are the only things you should do on a core-file: use only these
 * functions to write out all the necessary info.
 */
struct coredump_params;
extern int dump_skip(struct coredump_params *cprm, size_t nr);
extern int dump_emit(struct coredump_params *cprm, const void *addr, int nr);
extern int dump_align(struct coredump_params *cprm, int align);
extern void dump_truncate(struct coredump_params *cprm);
int dump_user_range(struct coredump_params *cprm, unsigned long start,
		    unsigned long len);
int dump_vma_snapshot(struct coredump_params *cprm, int *vma_count,
		      struct core_vma_metadata **vma_meta,
		      size_t *vma_data_size_ptr);
int match_uio_list(unsigned long addr, struct uio_info **uinfo, int *uioidx);
int match_ignore_bins(unsigned long addr);
#ifdef CONFIG_COREDUMP
extern void do_coredump(const kernel_siginfo_t *siginfo);
#else
static inline void do_coredump(const kernel_siginfo_t *siginfo) {}
#endif

extern int core_uses_pid;
extern char core_pattern[];
extern unsigned int core_pipe_limit;
extern unsigned int core_eqbmap;

#endif /* _LINUX_COREDUMP_H */
