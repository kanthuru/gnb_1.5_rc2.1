/*
 * EdgeQ Inc.
 *
 * Raptor2 Kernel Driver for Logger
 *
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
#include <asm/io.h>
#include "raptor2_logger.h"

static dev_t r2devt;
static struct class *r2logclass;
static struct cdev r2cdev;
//static struct log_mapping_access *lma;

static int r2log_device_op_open(struct inode *inode, struct file *filp)
{
	struct log_mapping_access *lma;

	lma = (struct log_mapping_access *)(filp->private_data);
	if(lma == NULL) {
		lma = (struct log_mapping_access *)kzalloc(sizeof(*lma), GFP_KERNEL);
		if(lma == NULL) {
			pr_err("%s: kzalloc failed for lma\n", __FUNCTION__);
			return -ENOMEM;
		}
		memset(lma, 0, sizeof(*lma));
		spin_lock_init(&lma->lock);
		lma->nprocs = 0;
		filp->private_data = lma;
	}

	pr_info("%s: r2log_device_op_open success\n", __FUNCTION__);
	return nonseekable_open(inode, filp);
}

#if 0
static int r2log_device_op_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;
	struct log_mapping_access *lma;

	lma = (struct log_mapping_access *)(filp->private_data);

	if(lma == NULL) {
		pr_err("%s: lma is NULL\n", __FUNCTION__);
		return -ENODEV;
	}

	if(lma->nprocs >= LOGGER_MAX_PROCS) {
		pr_err("%s: max processes exceeded\n", __FUNCTION__);
		return -EBUSY;
	}

	spin_lock(&lma->lock);
	++lma->nprocs;
	spin_unlock(&lma->lock);

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= VM_READ;
	vma->vm_flags |= VM_WRITE;
	vma->vm_flags |= VM_SHARED;

	ret = vm_iomap_memory(vma, LOGGER_MEM_BASE_PHYS, sizeof(struct engine_ppbuf_mapping_all));
	if(ret) {
		pr_err("%s: vm_iomap_memory 0x%llx failed with error %d\n",
				__FUNCTION__, LOGGER_MEM_BASE_PHYS, ret);
		return ret;
	}
	pr_info("%s: r2log_device_op_mmap phys: 0x%llx virt: 0x%lx success\n",
			__FUNCTION__, LOGGER_MEM_BASE_PHYS, vma->vm_start);

	return 0;
}

#else

void r2log_device_vma_open(struct vm_area_struct *vma)
{
	pr_info("%s: virt: %lx, phys: %lx\n",
			__FUNCTION__, vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

void r2log_device_vma_close(struct vm_area_struct *vma)
{
	pr_info("%s: virt: %lx, phys: %lx\n",  __FUNCTION__, vma->vm_start,
			vma->vm_pgoff << PAGE_SHIFT);
}

vm_fault_t r2log_device_vma_fault(struct vm_fault *vmf)
{
	struct page *page;
	unsigned long offset;
	void *pageptr = NULL;
	struct log_mapping_access *lma;
	struct vm_area_struct *vma;

	vma = vmf->vma;
	if(vma == NULL) {
		pr_err("%s: vma is NULL\n", __FUNCTION__);
		return VM_FAULT_SIGBUS;
	}

	lma = (struct log_mapping_access *)(vma->vm_private_data);

	if(lma == NULL) {
		pr_err("%s: lma is NULL\n", __FUNCTION__);
		return VM_FAULT_SIGBUS;
	}

	offset = (vmf->address - vma->vm_start) + (vma->vm_pgoff << PAGE_SHIFT);
	if(offset >= lma->size) {
		pr_err("%s: vmf->address 0x%lx is out of range\n", __FUNCTION__, vmf->address);
		return VM_FAULT_SIGBUS;
	}

	pageptr = lma->virtaddr + offset;
	if(!pageptr) {
		pr_err("%s: Page for vmf->address 0x%lx not found\n", __FUNCTION__, vmf->address);
		return VM_FAULT_SIGBUS;
	}

	page = virt_to_page(pageptr);
	if(!page) {
		pr_err("%s: Page for vmf->address 0x%lx pageptr 0x%llx not found\n",
				__FUNCTION__, vmf->address, (u64)pageptr);
		return VM_FAULT_SIGBUS;
	}

	get_page(page);
	vmf->page = page;

	return 0;
}

static struct vm_operations_struct r2log_device_vm_ops = {
	.open = r2log_device_vma_open,
	.close = r2log_device_vma_close,
	.fault = r2log_device_vma_fault,
};

static int r2log_device_op_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct log_mapping_access *lma;

	lma = (struct log_mapping_access *)(filp->private_data);

	if(lma == NULL) {
		pr_err("%s: lma is NULL\n", __FUNCTION__);
		return -ENODEV;
	}

	if(lma->nprocs >= LOGGER_MAX_PROCS) {
		pr_err("%s: max processes exceeded\n", __FUNCTION__);
		return -EBUSY;
	}

	spin_lock(&lma->lock);
	++lma->nprocs;
	spin_unlock(&lma->lock);

	lma->epmap = memremap(LOGGER_MEM_BASE_PHYS, sizeof(struct engine_ppbuf_mapping_all), MEMREMAP_WT);
	if(lma->epmap == NULL) {
		pr_err("%s: memremap failed for %lx\n", __FUNCTION__, LOGGER_MEM_BASE_PHYS);
		return -ENOMEM;
	}

	memset_io(lma->epmap, 0, sizeof(struct engine_ppbuf_mapping_all));

	lma->virtaddr = (u64)(lma->epmap);
	lma->size = sizeof(struct engine_ppbuf_mapping_all);

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_ops = &r2log_device_vm_ops;
	vma->vm_flags |= VM_READ;
	vma->vm_flags |= VM_WRITE;
	vma->vm_flags |= VM_SHARED;
	vma->vm_private_data = lma;

	r2log_device_vma_open(vma);

	pr_info("%s: r2log_device_op_mmap phys: 0x%lx virt: 0x%lx success\n",
			__FUNCTION__, LOGGER_MEM_BASE_PHYS, vma->vm_start);

	return  remap_pfn_range(vma, vma->vm_start,
			LOGGER_MEM_BASE_PHYS >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot);
}
#endif

static int r2log_device_op_release(struct inode *inode, struct file *filp)
{
	struct log_mapping_access *lma;

	lma = (struct log_mapping_access *)(filp->private_data);
	if(lma == NULL) {
		pr_err("lma is NULL\n");
		return -ENODEV;
	}

	spin_lock(&lma->lock);
	if(lma->nprocs) {
		--lma->nprocs;
		if(lma->nprocs == 0) {
			memunmap((void *)lma->virtaddr);
			kfree(lma);
		}
	}
	spin_unlock(&lma->lock);

	pr_info("%s: r2log_device_op_release success\n", __FUNCTION__);
	return 0;
}

const struct file_operations r2log_device_ops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.open		= r2log_device_op_open,
	.mmap		= r2log_device_op_mmap,
	.release	= r2log_device_op_release,
};

static int __init raptor2_logger_init(void)
{
#if 0
	r2_cdev_major = register_chrdev(0, LOGGER_DEV_NAME, &r2log_device_ops);  
	if(r2_cdev_major < 0) {
		pr_err("register_chrdev failed with %d\n", r2_cdev_major);
		return r2_cdev_major;
	}

	pr_info("register_chrdev succeeded with major number %d\n", r2_cdev_major);
	return 0;
#else
	int ret;

	ret = alloc_chrdev_region(&r2devt, 0, 1, "raptor2_logger");
	if(ret < 0) {
		pr_err("%s: alloc_chrdev_region failed with err code %d\n", __FUNCTION__, ret);
		return ret;
	}
	pr_info("%s: alloc_chrdev_region succeeded with major number %d, minor number %d\n", __FUNCTION__, MAJOR(r2devt), MINOR(r2devt));

	r2logclass = class_create(THIS_MODULE, "r2log_class");
	cdev_init(&r2cdev, &r2log_device_ops);

	ret = cdev_add(&r2cdev, r2devt, 1);
	if(ret) {
		pr_err("%s: Failed to add cdev to subsys, cdev_add returns %d\n", __FUNCTION__, ret);
		return ret;
	}

	device_create(r2logclass, NULL, r2devt, NULL, LOGGER_DEV_NAME);

	return 0;
#endif
}

static void __init raptor2_logger_cleanup(void)
{
#if 0
	unregister_chrdev(r2_cdev_major, LOGGER_DEV_NAME);
#else
	cdev_del(&r2cdev);
	device_destroy(r2logclass, r2devt);
	class_destroy(r2logclass);
	unregister_chrdev_region(r2devt, 1);
#endif
}

module_init(raptor2_logger_init);
module_exit(raptor2_logger_cleanup);
