/*
 * EdgeQ Inc.
 *
 * Raptor2 Kernel Driver for Shared Memory
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
#include "raptor2_shm.h"

static dev_t r2devnum;
static struct class *r2shmclass;
static struct cdev r2cdev;


static int r2shm_device_op_open(struct inode *inode, struct file *filp)
{
	struct shm_mapping_access *sma;

	sma = (struct shm_mapping_access *)(filp->private_data);
	if(sma == NULL) {
		sma = (struct shm_mapping_access *)kzalloc(sizeof(*sma), GFP_KERNEL);
		if(sma == NULL) {
			pr_err("%s: kzalloc failed for sma\n", __FUNCTION__);
			return -ENOMEM;
		}
		memset(sma, 0, sizeof(*sma));
		spin_lock_init(&sma->lock);
		sma->nprocs = 0;
		filp->private_data = sma;
	}

	pr_info("%s: r2shm_device_op_open success\n", __FUNCTION__);
	return nonseekable_open(inode, filp);
}

void r2shm_device_vma_open(struct vm_area_struct *vma)
{
	pr_info("%s: virt: %lx, phys: %lx\n",
			__FUNCTION__, vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

void r2shm_device_vma_close(struct vm_area_struct *vma)
{
	pr_info("%s: virt: %lx, phys: %lx\n",  __FUNCTION__, vma->vm_start,
			vma->vm_pgoff << PAGE_SHIFT);
}

vm_fault_t r2shm_device_vma_fault(struct vm_fault *vmf)
{
	struct page *page;
	unsigned long offset;
	unsigned char *pageptr = NULL;
	struct shm_mapping_access *sma;
	struct vm_area_struct *vma;

	vma = vmf->vma;
	if(vma == NULL) {
		pr_err("%s: vma is NULL\n", __FUNCTION__);
		return VM_FAULT_SIGBUS;
	}

	sma = (struct shm_mapping_access *)(vma->vm_private_data);

	if(sma == NULL) {
		pr_err("%s: sma is NULL\n", __FUNCTION__);
		return VM_FAULT_SIGBUS;
	}
	pr_info("%s: sma is 0x%lx\n", __FUNCTION__, (unsigned long)sma);

	offset = (vmf->address - vma->vm_start) + (vma->vm_pgoff << PAGE_SHIFT);
	if(offset >= sma->size) {
		pr_err("%s: vmf->address 0x%lx is out of range\n", __FUNCTION__, vmf->address);
		return VM_FAULT_SIGBUS;
	}
	pr_info("%s: vmf->address 0x%lx offset is 0x%lx \n", __FUNCTION__, (unsigned long)vmf->address, (unsigned long)offset);

	pageptr = (unsigned char *)(sma->virtaddr + offset);
	if(!pageptr) {
		pr_err("%s: Page for vmf->address 0x%lx not found\n", __FUNCTION__, vmf->address);
		return VM_FAULT_SIGBUS;
	}
	pr_info("%s: pageptr is 0x%lx\n", __FUNCTION__, (unsigned long)pageptr);

	page = virt_to_page(pageptr);
	if(!page) {
		pr_err("%s: Page for vmf->address 0x%lx pageptr 0x%llx not found\n",
				__FUNCTION__, vmf->address, (u64)pageptr);
		return VM_FAULT_SIGBUS;
	}
	pr_info("%s: page is 0x%lx\n", __FUNCTION__, (unsigned long)page);

	get_page(page);
	vmf->page = page;

	return 0;
}

static struct vm_operations_struct r2shm_device_vm_ops = {
	.open = r2shm_device_vma_open,
	.close = r2shm_device_vma_close,
	.fault = r2shm_device_vma_fault,
};

static int r2shm_device_op_mmap(struct file *filp, struct vm_area_struct *vma)
{
	//void __iomem *mptr;
	void *mptr;
	struct shm_mapping_access *sma;

	sma = (struct shm_mapping_access *)(filp->private_data);

	if(sma == NULL) {
		pr_err("%s: sma is NULL\n", __FUNCTION__);
		return -ENODEV;
	}

	if(sma->nprocs >= RAPTOR2_SHM_MAX_PROCS) {
		pr_err("%s: max processes exceeded\n", __FUNCTION__);
		return -EBUSY;
	}

	if(RAPTOR2_WLAN_SHM_PHYS_BASE & ~PAGE_MASK)
                return -ENODEV;

	spin_lock(&sma->lock);
	++sma->nprocs;
	spin_unlock(&sma->lock);

	mptr = memremap(RAPTOR2_WLAN_SHM_PHYS_BASE, RAPTOR2_WLAN_SHM_SIZE, MEMREMAP_WT);
	if(mptr == NULL) {
		pr_err("%s: memremap failed for 0x%lx\n", __FUNCTION__, RAPTOR2_WLAN_SHM_PHYS_BASE);
		return -ENOMEM;
	}

	sma->virtaddr = (u64)(mptr); 
	sma->size = RAPTOR2_WLAN_SHM_SIZE;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_ops = &r2shm_device_vm_ops;
	vma->vm_flags |= VM_READ;
	vma->vm_flags |= VM_WRITE;
	vma->vm_flags |= VM_SHARED;
	vma->vm_private_data = sma;

	r2shm_device_vma_open(vma);

	pr_info("%s: Phys: 0x%lx virt: 0x%lx success\n",
			__FUNCTION__, RAPTOR2_WLAN_SHM_PHYS_BASE, vma->vm_start);

	//return 0;
	return  remap_pfn_range(vma, vma->vm_start,
			RAPTOR2_WLAN_SHM_PHYS_BASE >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot);
					
}

static int r2shm_device_op_release(struct inode *inode, struct file *filp)
{
	struct shm_mapping_access *sma;

	sma = (struct shm_mapping_access *)(filp->private_data);
	if(sma == NULL) {
		pr_err("sma is NULL\n");
		return -ENODEV;
	}

	spin_lock(&sma->lock);
	if(sma->nprocs) {
		--sma->nprocs;
		if(sma->nprocs == 0) {
			//iounmap((void __iomem *)sma->virtaddr);
			memunmap((void *)sma->virtaddr);
			kfree(sma);
		}
	}
	spin_unlock(&sma->lock);

	pr_info("%s: Success\n", __FUNCTION__);
	return 0;
}

const struct file_operations r2shm_device_ops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.open		= r2shm_device_op_open,
	.mmap		= r2shm_device_op_mmap,
	.release	= r2shm_device_op_release,
};

static int __init raptor2_shm_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&r2devnum, 0, 1, "raptor2_shm");
	if(ret < 0) {
		pr_err("%s: alloc_chrdev_region failed with err code %d\n", __FUNCTION__, ret);
		return ret;
	}
	pr_info("%s: alloc_chrdev_region succeeded with major number %d, minor number %d\n",
			__FUNCTION__, MAJOR(r2devnum), MINOR(r2devnum));

	r2shmclass = class_create(THIS_MODULE, "r2shm_class");
	cdev_init(&r2cdev, &r2shm_device_ops);
	ret = cdev_add(&r2cdev, r2devnum, 1);
	if(ret) {
		pr_err("%s: Failed to add cdev to subsys, cdev_add returns %d\n", __FUNCTION__, ret);
		return ret;
	}

	device_create(r2shmclass, NULL, r2devnum, NULL, RAPTOR2_WLAN_SHM_DEV_NAME);

	return 0;
}

static void __init raptor2_shm_cleanup(void)
{
	cdev_del(&r2cdev);
	device_destroy(r2shmclass, r2devnum);
	class_destroy(r2shmclass);
	unregister_chrdev_region(r2devnum, 1);
}

module_init(raptor2_shm_init);
module_exit(raptor2_shm_cleanup);
