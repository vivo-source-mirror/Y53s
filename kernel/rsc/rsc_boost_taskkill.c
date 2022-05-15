#include <linux/mm.h>
#include <linux/hugetlb.h>
#include <linux/vivo_rsc/rsc_internal.h>

#include <asm/tlb.h>

void unmap_page_range(struct mmu_gather *tlb,
			     struct vm_area_struct *vma,
			     unsigned long addr, unsigned long end,
			     struct zap_details *details);

/* see __oom_reap_task_mm in oom_killer.c */
static void __boost_free_user_mem(struct mm_struct *mm)
{
	struct vm_area_struct *vma;
	struct mmu_gather tlb;

	for (vma = mm->mmap; vma; vma = vma->vm_next) {

		/* same as !can_madv_dontneed_vma */
		if (vma->vm_flags & (VM_LOCKED|VM_HUGETLB|VM_PFNMAP))
			continue;

		if (vma_is_anonymous(vma) || !(vma->vm_flags & VM_SHARED)) {
			tlb_gather_mmu(&tlb, mm, vma->vm_start, vma->vm_end);
			unmap_page_range(&tlb, vma, vma->vm_start,
					vma->vm_end, NULL);
			tlb_finish_mmu(&tlb, vma->vm_start, vma->vm_end);
		}
	}
}

void boost_free_user_mem(void)
{
	struct mm_struct *mm = current->mm;

	if (!mm)
		return;

	if (unlikely(test_bit(MMF_OOM_SKIP, &mm->flags))) {
		/*
		rsc_info("boost taskkill %16s %5d %16s %5d MMF_OOM_SKIP\n",
			current->comm, current->pid,
			current->group_leader?current->group_leader->comm:"NULL",
			current->group_leader?current->group_leader->pid:99999);
		*/
		return;
	}

	if (unlikely(test_and_set_bit(MMF_RSC_BOOST_FREE, &mm->flags)))
		return;

	down_read(&mm->mmap_sem);
	__boost_free_user_mem(mm);
	up_read(&mm->mmap_sem);
}
