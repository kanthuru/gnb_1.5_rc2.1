// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2023 EdgeQ, Inc.
 * Raptor2 EXMGR Ops
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/irqflags.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include "raptor2_exmgr_ops.h"
#include "raptor2_riscv.h"
#include "cci.h"

static dev_t r2devt;
static struct class *r2exmgrclass;
static struct cdev r2cdev;
static int wait_for_exception = 1;
static int dump_su_mems = 0;
static int dump_rtos_mems = 0;
static DECLARE_WAIT_QUEUE_HEAD(exmgr_wqh);
extern struct eqsu_membuf eqsu_mbuf_table[];
static struct exmgr_mapping_access *ema_m0 = NULL;
static struct exmgr_mapping_access *ema_rtos = NULL;
static int rtosaddravbl = 0;
static cci_crash_dump_info_t crashinfo;

static r2_chip_version_t r2chipvers;

static int exmgr_map_dmabuf(int minor, struct exmgr_mapping_access *ema)
{
	struct eqsu_membuf *mbuftbl;

	if ((minor <= 0) || (minor >= MRTOS)) {
		pr_err("%s: minor %d out of range\n", __func__, minor);
		return -ERANGE;
	}

	if (ema == NULL) {
		pr_err("%s: ema is NULL for minor %d\n", __func__, minor);
		return -ENOMEM;
	}

	mbuftbl = ema_m0->mptr;
	if (mbuftbl == NULL) {
		pr_err("%s: mbuftbl is NULL minor:%d\n", __func__, minor);
		return -ENOMEM;
	}

	if (mbuftbl[minor].mbuf == NULL) {
		pr_err("%s: mbuf is NULL for minor:%d %s SU%d\n",
			__func__, minor, engstr[mbuftbl[minor].etype], mbuftbl[minor].suid);
		return -ENOMEM;
	}

	ema->mptr = mbuftbl[minor].mbuf;
	ema->size = mbuftbl[minor].mbufsize;
	ema->virtaddr = (u64)mbuftbl[minor].mbuf;

	return 0;
}

static struct exmgr_mapping_access* alloc_exmgr_map(int minor)
{
	struct exmgr_mapping_access *ema = NULL;

	if ((minor < 0) || (minor >= MRTOS)) {
		pr_err("%s: minor %d out of range\n", __func__, minor);
		return NULL;
	}

	if ((minor == SU_START) && ema_m0)
		return ema_m0;

	if ((minor == MRTOS) && ema_rtos)
		return ema_rtos;

	ema = (struct exmgr_mapping_access *)kzalloc(sizeof(*ema), GFP_KERNEL);
	if (ema == NULL) {
		pr_err("%s: kzalloc failed for ema\n", __func__);
		return NULL;
	}

	spin_lock_init(&ema->lock);
	ema->nprocs = 0;

	return ema;
}

static int r2exmgr_device_op_open(struct inode *inode, struct file *filp)
{
	int minor;
	struct exmgr_mapping_access *ema = NULL;

	minor = MINOR(inode->i_rdev);

	ema = filp->private_data;
	if (ema == NULL) {
		ema = alloc_exmgr_map(minor);
		if (ema == NULL) {
			pr_err("%s: alloc_exmgr_map failed for minor %d\n",
					__func__, minor);
			return -ENOMEM;
		}
		filp->private_data = ema;
	}

	if (ema->nprocs >= EXMGR_MAX_PROCS) {
		pr_err("%s: num procs exceeded\n", __func__);
		return -EBUSY;
	}

	spin_lock(&ema->lock);
	++ema->nprocs;
	spin_unlock(&ema->lock);

	return nonseekable_open(inode, filp);
}

static void r2exmgr_device_vma_open(struct vm_area_struct *vma)
{
	//pr_info("%s: virt: %lx, phys: %lx\n",
	//	__func__, vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

static void r2exmgr_device_vma_close(struct vm_area_struct *vma)
{
	//pr_info("%s: virt: %lx, phys: %lx\n",  __func__, vma->vm_start,
	//	vma->vm_pgoff << PAGE_SHIFT);
}

static vm_fault_t r2exmgr_device_vma_fault(struct vm_fault *vmf)
{
	struct page *page;
	unsigned long offset;
	uint8_t *pageptr = NULL;
	struct exmgr_mapping_access *ema;
	struct vm_area_struct *vma;

	vma = vmf->vma;
	if (vma == NULL) {
		pr_err("%s: vma is NULL\n", __func__);
		return VM_FAULT_SIGBUS;
	}

	ema = (struct exmgr_mapping_access *)(vma->vm_private_data);

	if (ema == NULL) {
		pr_err("%s: ema is NULL\n", __func__);
		return VM_FAULT_SIGBUS;
	}

	offset = (vmf->address - vma->vm_start) + (vma->vm_pgoff << PAGE_SHIFT);
	if (offset >= ema->size) {
		pr_err("%s: vmf->address 0x%lx is out of range\n",
							__func__, vmf->address);
		return VM_FAULT_SIGBUS;
	}

	pageptr = (uint8_t *)(ema->virtaddr + offset);
	if (!pageptr) {
		pr_err("%s: Page for vmf->address 0x%lx not found\n",
							__func__, vmf->address);
		return VM_FAULT_SIGBUS;
	}

	page = virt_to_page(pageptr);
	if (!page) {
		pr_err("%s: Page for vmf->address 0x%lx pageptr 0x%llx not found\n",
				__func__, vmf->address, (u64)pageptr);
		return VM_FAULT_SIGBUS;
	}

	get_page(page);
	vmf->page = page;

	return 0;
}

static struct vm_operations_struct r2exmgr_device_vm_ops = {
	.open = r2exmgr_device_vma_open,
	.close = r2exmgr_device_vma_close,
	.fault = r2exmgr_device_vma_fault,
};

static int r2exmgr_device_op_mmap(struct file *filp, struct vm_area_struct *vma)
{
	size_t size;
	int ret, minor;
	unsigned long pa = 0;
	struct exmgr_mapping_access *ema;

	minor = MINOR(filp->f_inode->i_rdev);
	if ((minor <= 0) || (minor >= MRTOS)) {
		pr_err("%s: minor %d out of range\n", __func__, minor);
		return -1;
	}

	ema = (struct exmgr_mapping_access *)(filp->private_data);
	if (ema == NULL) {
		pr_err("%s: ema is NULL for minor %d\n", __func__, minor);
		return -ENOMEM;
	}

	ret = exmgr_map_dmabuf(minor, ema);
	if (ret < 0) {
		pr_err("%s: exmgr_map_dmabuf failed for minor %d\n", __func__, minor);
		return ret;
	}

	size = vma->vm_end - vma->vm_start;

	if (minor == SU_START) {
		pr_err("%s: not supported for minor %d\n", __func__, minor);
		return -ENOTSUPP;
	}

	if ((minor == MRTOS) && !rtosaddravbl) {
		pr_err("%s: RTOS memory address is 0 for minor %d\n", __func__, minor);
		return -EADDRNOTAVAIL;
	}

	r2exmgr_device_vma_open(vma);
	pa = virt_to_phys(ema->mptr);
	vma->vm_pgoff = pa >> PAGE_SHIFT;

	if (minor ==  MRTOS)
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	else
		vma->vm_page_prot = phys_mem_access_prot(filp, vma->vm_pgoff,
				size, vma->vm_page_prot);

	vma->vm_ops = &r2exmgr_device_vm_ops;
	vma->vm_flags |= VM_READ | VM_IO | VM_DONTEXPAND;
	vma->vm_private_data = ema;

	if (minor == MRTOS)
		pr_info("%s: remapping for rtos mem virt addr: %llx\n",
				__func__, (u64)(ema->mptr));

	return remap_pfn_range(vma, vma->vm_start,
			vma->vm_pgoff, size,
			vma->vm_page_prot);
}

static int r2exmgr_device_op_release(struct inode *inode, struct file *filp)
{
	int minor;
	struct exmgr_mapping_access *ema;

	ema = (struct exmgr_mapping_access *)(filp->private_data);
	if (ema == NULL) {
		pr_err("ema is NULL\n");
		return -ENODEV;
	}

	minor = MINOR(filp->f_inode->i_rdev);

	spin_lock(&ema->lock);
	if (ema->nprocs)
		--ema->nprocs;

	if ((minor == SU_START) || (minor == MRTOS))
		goto out;

	ema->mptr = NULL;

	if (ema->nprocs == 0)
		kfree(ema);

out:
	spin_unlock(&ema->lock);

	return 0;
}

static int enable_memories(struct r2_engine_su *engsu)
{
	u32 mapsz;
	u64 mapaddr;
	void __iomem *cmu;

	mapaddr = engsu->sudevaddr + engsu->sudev.cmu_offs;
	mapsz = engsu->sudev.cmusz;

	cmu = ioremap(mapaddr, mapsz);

	if (cmu == NULL) {
		pr_err("%s: Unable to map CMU sudev_base=0x%llx, cmu_offs=0x%llx, cmusz=0x%x\n",
		__func__, engsu->sudevaddr, engsu->sudev.cmu_offs, engsu->sudev.cmusz);
		return -ENOMEM;
	}

	if (engsu->etype == ENGINE_TXU)
		writel(0xC0008815, cmu+CMU_CORE_CONFIG_REG);
	else
		writel(0x00008BDA, cmu+CMU_CORE_CONFIG_REG);

	iounmap(cmu);
	return 0;
}

static int copy_sumem_tobuf(struct r2_engine_su *engsu, su_dma_buf_t *dmabuf)
{
	int ret;
	u32 mapsz;
	u64 mapaddr;

	if (engsu == NULL) {
		pr_crit("%s: engsu is NULL\n", __func__);
		return -ENXIO;
	}

	if (dmabuf == NULL) {
		pr_crit("%s: dmabuf is NULL\n", __func__);
		return -ENXIO;
	}

	ret = enable_memories(engsu);
	if (ret < 0) {
		pr_crit("%s: enable_memories failed for %s SU%d\n",
				__func__, engstr[engsu->etype], engsu->su_index);
		return ret;
	}

	/* Map ILM */
	if (engsu->membm & MEMTYPE_ILM_BM) {
		mapaddr = engsu->sudevaddr + engsu->sudev.ilm_offs;
		mapsz = engsu->sudev.ilmsz;

		engsu->map.ilm = ioremap(mapaddr, mapsz);
		if (engsu->map.ilm == NULL) {
			pr_err("%s: ILM map fail for %s SU%d mapaddr=0x%llx, mapsz=%x\n",
					__func__, engstr[engsu->etype], engsu->su_index,
					mapaddr, mapsz);
			return -ENOMEM;
		}

		dmabuf->ilmsz = mapsz;
		memcpy_fromio(dmabuf->ilm, engsu->map.ilm, mapsz);
		iounmap(engsu->map.ilm);
	}

	/* Map DLM */
	if (engsu->membm & MEMTYPE_DLM_BM) {
		mapaddr = engsu->sudevaddr + engsu->sudev.dlm_offs;
		mapsz = engsu->sudev.dlmsz;

		engsu->map.dlm = ioremap(mapaddr, mapsz);
		if (engsu->map.dlm == NULL) {
			pr_err("%s: DLM map fail for %s SU%d mapaddr=0x%llx, mapsz=%x\n",
					__func__, engstr[engsu->etype], engsu->su_index,
					mapaddr, mapsz);
			return -ENOMEM;
		}

		dmabuf->dlmsz = mapsz;
		memcpy_fromio(dmabuf->dlm, engsu->map.dlm, mapsz);
		iounmap(engsu->map.dlm);
	}

	/* Map LMEM */
	if (engsu->membm & MEMTYPE_LMEM_CNOC_BM) {
		mapaddr = engsu->lmemaddr_cpu;
		mapsz = engsu->lmemsz_cpu;

		engsu->map.lmem = ioremap(mapaddr, mapsz);
		if (engsu->map.lmem == NULL) {
			pr_err("%s: LMEM map fail for %s SU%d mapaddr=0x%llx, mapsz=%x\n",
					__func__, engstr[engsu->etype], engsu->su_index,
					mapaddr, mapsz);
			return -ENOMEM;
		}

		dmabuf->lmemsz = mapsz;
		memcpy_fromio(dmabuf->lmem, engsu->map.lmem, mapsz);
		iounmap(engsu->map.lmem);
	}

	/* Map LMEM_B2 */
	if (engsu->membm & MEMTYPE_LMEM_CNOC_B2_BM) {
		mapaddr = engsu->lmemb2_addr_cpu;
		mapsz = engsu->lmemb2_sz_cpu;

		engsu->map.lmem_b2 = ioremap(mapaddr, mapsz);
		if (engsu->map.lmem_b2 == NULL) {
			pr_err("%s: LMEM_B2 map fail for %s SU%d mapaddr=0x%llx, mapsz=%x\n",
					__func__, engstr[engsu->etype], engsu->su_index,
					mapaddr, mapsz);
			return -ENOMEM;
		}

		dmabuf->lmemb2sz = mapsz;
		memcpy_fromio(dmabuf->lmem_b2, engsu->map.lmem_b2, mapsz);
		iounmap(engsu->map.lmem_b2);
	}

	/* Map Cluster ILM */
	if ((engsu->membm & MEMTYPE_CLUSTER_ILM_LOCAL_BM) ||
		       (engsu->membm & MEMTYPE_CLUSTER_ILM_CNOC_BM)) {
		mapaddr = engsu->clilmaddr_cnoc;
		mapsz = engsu->clilmsz_cnoc;

		engsu->map.clilm = ioremap(mapaddr, mapsz);
		if (engsu->map.clilm == NULL) {
			pr_err("%s: CLILM map fail for %s SU%d mapaddr=0x%llx, mapsz=%x\n",
					__func__, engstr[engsu->etype], engsu->su_index,
					mapaddr, mapsz);
			return -ENOMEM;
		}

		dmabuf->clilmsz = mapsz;
		memcpy_fromio(dmabuf->clilm, engsu->map.clilm, mapsz);
		iounmap(engsu->map.clilm);
	}

	return 0;
}

static int copy_engine_sumems(struct eqsu_membuf eqsu_mbuf_table[])
{
	int sz, ret;
	r2_suidx_t idx;
	r2_engine_t etype;
	unsigned long addr;
	su_dma_buf_t *dmabuf;

	for (idx = PPU0; idx < SU_END; idx++) {

		struct r2_engine_su *engsu = eqsu_mbuf_table[idx].engsu;

		if ((r2chipvers == R2_VERSION_A0) && (idx >= IRING0))
			break;

		if (!engsu->enabled)
			continue;

		sz = eqsu_mbuf_table[idx].mbufsize;
		etype = eqsu_mbuf_table[idx].etype;

		dmabuf = (su_dma_buf_t *)__get_free_pages(GFP_KERNEL | GFP_DMA | __GFP_ZERO,
										get_order(sz));
		if (dmabuf == NULL) {
			pr_err("%s: get_free_pages failed for %s, size %d, order %d\n",
					__func__,  engstr[etype], sz, get_order(sz));
			return -ENOMEM;
		}

		eqsu_mbuf_table[idx].mbuf = dmabuf;
		addr = (unsigned long)dmabuf;

		while (sz > 0) {
			SetPageReserved(virt_to_page(addr));
			addr += PAGE_SIZE;
			sz -= PAGE_SIZE;
		}

		ret = copy_sumem_tobuf(eqsu_mbuf_table[idx].engsu, eqsu_mbuf_table[idx].mbuf);
		if (ret < 0) {
			pr_err("%s: copy_sumem_tobuf failed for index %d %s SU%d\n",
					__func__, idx, engstr[etype], eqsu_mbuf_table[idx].suid);
			return ret;
		}
	}

	return 0;
}

static inline int try_wfe(void)
{
	return (wait_for_exception == 0);
}

int trigger_zephyr_coredump(cci_crash_dump_info_t *crash_dump)
{
	if (crash_dump == NULL) {
		pr_err("%s: crash_dump is NULL\n", __func__);
		return -EFAULT;
	}

	pr_info("%s triggered\n", __func__);

	memset(&crashinfo, 0, sizeof(cci_crash_dump_info_t));
	memcpy(&crashinfo, crash_dump, sizeof(cci_crash_dump_info_t));

	dump_su_mems = 0;
	dump_rtos_mems = 1;
	wait_for_exception = 0;
	wake_up(&exmgr_wqh);

	return 0;
}
EXPORT_SYMBOL(trigger_zephyr_coredump);

#if defined(WLAN)
static int generate_zephyr_coredump(struct r2_rtos_info *rtosinfo)
{
	if (rtosinfo == NULL) {
		pr_err("rtosinfo is NULL\n");
		return -EFAULT;
	}

	memcpy(&rtosinfo->z.crashinfo, &crashinfo, sizeof(cci_crash_dump_info_t));

	return 0;
}
#endif //WLAN

static void free_su_dmabufs(struct eqsu_membuf eqsu_mbuf_table[])
{
	int sz, size;
	r2_suidx_t idx;
	unsigned long addr;
	su_dma_buf_t *dmabuf;

	if (eqsu_mbuf_table == NULL) {
		pr_err("eqsu_mbuf_table is NULL\n");
		return;
	}

	for (idx = PPU0; idx < SU_END; idx++) {

		struct r2_engine_su *engsu = eqsu_mbuf_table[idx].engsu;

		if ((r2chipvers == R2_VERSION_A0) && (idx >= IRING0))
			break;

		if (!engsu->enabled)
			continue;

		dmabuf = (su_dma_buf_t *)eqsu_mbuf_table[idx].mbuf;
		sz = size = eqsu_mbuf_table[idx].mbufsize;
		addr = (unsigned long)dmabuf;

		while (size > 0) {
			ClearPageReserved(virt_to_page(addr));
			addr += PAGE_SIZE;
			size -= PAGE_SIZE;
		}

		if (sz > 0)
			free_pages((unsigned long)dmabuf, get_order(sz));
	}
}

static long r2exmgr_device_op_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret, minor;
	unsigned long flags, dmap = 0;
	struct exmgr_mapping_access *ema;
#if defined(WLAN)
	struct r2_rtos_info *rtosinfo;
#endif
	cci_crash_dump_info_t cinfo;

	memset(&cinfo, 0, sizeof(cci_crash_dump_info_t));

	ema = (struct exmgr_mapping_access *)(filp->private_data);
	if (ema == NULL) {
		pr_err("ema is NULL\n");
		return -ENODEV;
	}

	minor = MINOR(filp->f_inode->i_rdev);

	ret = 0;
	switch(cmd) {
	default:
		pr_err("%s: Invalid ioctl code 0x%x\n", __func__, cmd);
		return -EINVAL;
	case EXMGR_NOWAIT_FOR_EXCEPTION:
		local_irq_save(flags);
		wait_for_exception = 0;
		dump_su_mems = 0;
		dump_rtos_mems = 0;
		local_irq_restore(flags);
		wake_up(&exmgr_wqh);
		return 0;
	case EXMGR_WAIT_FOR_EXCEPTION:
		wait_event(exmgr_wqh, try_wfe());
		if (dump_su_mems) {
			ret = copy_engine_sumems(ema->mptr);
			if (ret)
				pr_err("%s: copy_engine_sumems failed, returns %d\n",
								__func__, ret);
			else
				dmap |= DUMPTYPE_SUMEM;
		}

#if defined(WLAN)
		if (dump_rtos_mems) {
			pr_info("%s: calling generate_zephyr_coredump\n", __func__);
			rtosinfo = (struct r2_rtos_info *)(ema_rtos->mptr);
			ret = generate_zephyr_coredump(rtosinfo);
			if (ret)
				pr_err("%s: generate_zephyr_coredump failed, returns %d\n",
									__func__, ret);
			else
				dmap |= DUMPTYPE_RTOSMEM;
		}
#endif
		wait_for_exception = 1;
		dump_su_mems = 0;
		dump_rtos_mems = 0;
		break;
	case EXMGR_DUMP_MEMS_AND_EXIT:
		local_irq_save(flags);
		wait_for_exception = 0;
		dump_su_mems = 1;
		dump_rtos_mems = 1;
		local_irq_restore(flags);
		wake_up(&exmgr_wqh);
		return 0;
	case EXMGR_DUMP_RTOSMEM_AND_EXIT:
		local_irq_save(flags);
		wait_for_exception = 0;
		dump_su_mems = 0;
		dump_rtos_mems = 1;
		local_irq_restore(flags);
		wake_up(&exmgr_wqh);
		return 0;
	case EXMGR_FREESU_DMABUFS:
		local_irq_save(flags);
		free_su_dmabufs(ema->mptr);
		local_irq_restore(flags);
		return 0;
	}

	ret = copy_to_user((unsigned long *)arg, &dmap, sizeof(dmap));
	if (ret) {
		pr_err("copy_to_user failed, returns %d\n", ret);
		ret = -1;
	}

	return ret;
}

const struct file_operations r2exmgr_device_ops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.open		= r2exmgr_device_op_open,
	.mmap		= r2exmgr_device_op_mmap,
	.unlocked_ioctl	= r2exmgr_device_op_ioctl,
	.release	= r2exmgr_device_op_release,
};

int raptor2_exmgr_ops_init(struct eqsu_membuf eqsu_mbuf_table[],
				struct r2_rtos_info *rtosinfo, r2_chip_version_t r2vers)
{
	int ret;
	u32 major, minor;
	int num_minor_devs;

	r2chipvers = r2vers;

#if defined(WLAN)
	if (rtosinfo == NULL) {
		pr_err("%s: rtosinfo is NULL\n", __func__);
		return -1;
	}
#endif
	if (r2chipvers == R2_VERSION_A0)
		num_minor_devs = IRING0;
	else
		num_minor_devs = SU_END;

	ret = alloc_chrdev_region(&r2devt, 0, num_minor_devs, "raptor2_exmgr");
	if (ret < 0) {
		pr_err("%s: alloc_chrdev_region failed with err code %d\n", __func__, ret);
		return ret;
	}
	pr_info("%s: alloc_chrdev_region succeeded with major number %d, minor number %d\n",
				__func__, MAJOR(r2devt), MINOR(r2devt));

	r2exmgrclass = class_create(THIS_MODULE, "r2exmgr_class");
	cdev_init(&r2cdev, &r2exmgr_device_ops);
	major = MAJOR(r2devt);
#if defined(WLAN)
	++num_minor_devs;
#endif
	ret = cdev_add(&r2cdev, r2devt, num_minor_devs);
	if (ret) {
		pr_err("%s: Failed to add cdev to subsys, cdev_add returns %d\n",
									__func__, ret);
		return ret;
	}

	for (minor = SU_START; minor < num_minor_devs; minor++) {
		pr_info("%s: Creating device %s%u\n", __func__, EXMGR_DEV_NAME, minor);
		device_create(r2exmgrclass, NULL, MKDEV(major, minor),
							NULL, EXMGRDEV(%u), minor);
	}

#if defined(WLAN)
	device_create(r2exmgrclass, NULL, MKDEV(major, MRTOS),
							NULL, EXMGRDEV(%u), MRTOS);
	rtosaddravbl = 1;
	ema_rtos = alloc_exmgr_map(MRTOS);
	ema_rtos->mptr = (uint8_t *)rtosinfo;
	if (ema_rtos->mptr == NULL) {
		pr_err("%s: ema_rtos->mptr is NULL\n", __func__);
		return -ENOMEM;
	}

	ema_rtos->size = rtosinfo->info.size;
	ema_rtos->virtaddr = (u64)(ema_rtos->mptr);
	pr_info("%s: Mapping RTOS mem %llx size %llx virt:%llx to dev r2exmgr%d\n", __func__,
		rtosinfo->info.startaddr, rtosinfo->info.size, ema_rtos->virtaddr, MRTOS);
#endif

	ema_m0 = alloc_exmgr_map(SU_START);
	ema_m0->mptr = (struct eqsu_membuf *)eqsu_mbuf_table;

#if defined(WLAN)
	/* clear out crashinfo */
	memset(&crashinfo, 0, sizeof(cci_crash_dump_info_t));
	pr_info("%s: cleared crashinfo\n", __func__);
#endif

	return 0;
}

void raptor2_exmgr_ops_cleanup(void)
{
	int num_minor_devs;

	if (r2chipvers == R2_VERSION_A0)
		num_minor_devs = IRING0;
	else
		num_minor_devs = SU_END;

	cdev_del(&r2cdev);
	device_destroy(r2exmgrclass, r2devt);
	class_destroy(r2exmgrclass);
	unregister_chrdev_region(r2devt, num_minor_devs);
	kfree(ema_m0);
	memunmap(ema_rtos->mptr);
	kfree(ema_rtos);
}
