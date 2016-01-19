#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <asm/uaccess.h>

#include <linux/epiphany.h>

MODULE_AUTHOR("XCube, Ben Chaco");
MODULE_DESCRIPTION("Adapteva Epiphany Driver");
MODULE_LICENSE("GPL");

#define UseReservedMem 1

#define DRIVER_NAME            "epiphany"

/* The physical address of the start and end of the Epiphany device memory */
#define EPIPHANY_MEM_START      0x80800000UL
#define EPIPHANY_MEM_END        0xC0000000UL

/* FPGA elink register base
 * TODO: Define in device tree */
#define ELINK_REGS_BASE		0x81000000UL
#define ELINK_REGS_SIZE		0x00100000UL

#define ELINK_RXCFG		0xF0300UL
#define ELINK_MAILBOXLO		0xF0730UL
#define ELINK_MAILBOXHI		0xF0734UL
#define ELINK_MAILBOXSTAT	0xF0738UL

/* FPGA elink IRQ NR
 * TODO: Define in device tree */
#define EPIPHANY_MAILBOX_IRQ_NR	87 /* 55 + 32 */

/* The physical address of the DRAM shared between the host and Epiphany */
#define HOST_MEM_START          0x3E000000UL
#define HOST_MEM_END            0x40000000UL

/* Physical address range that can be mapped to FPGA logic */
/* TODO: Define in devicetree */
#define PL_MEM_START            0x40000000UL
#define PL_MEM_END              0x80000000UL

/* One mutex will do it for now */
static DEFINE_MUTEX(epiphany_mutex);

static DECLARE_WAIT_QUEUE_HEAD(epiphany_mailbox_wq);

/* Function prototypes */
static int epiphany_init(void);
static void epiphany_exit(void);
static int epiphany_open(struct inode *, struct file *);
static int epiphany_release(struct inode *, struct file *);
static int epiphany_mmap(struct file *, struct vm_area_struct *);
static long epiphany_ioctl(struct file *, unsigned int, unsigned long);
static irqreturn_t epiphany_mailbox_irq_handler(int, void *);

static struct file_operations epiphany_fops = {
	.owner = THIS_MODULE,
	.open = epiphany_open,
	.release = epiphany_release,
	.mmap = epiphany_mmap,
	.unlocked_ioctl = epiphany_ioctl
};

static int major = 0;
static dev_t dev_no = 0;
static struct cdev *epiphany_cdev = 0;
static struct class *class_epiphany = 0;
static struct device *dev_epiphany = 0;
static epiphany_alloc_t global_shm;

static void __iomem *elink_regs;


union elink_rxcfg {
	u32 reg;
	struct {
		unsigned test_mode:1;
		unsigned mmu_enable:1;
		unsigned remap_mode:2;
		unsigned remap_sel:12;
		unsigned remap_pattern:12;
		unsigned mailbox_irq_enable:1;
	};
} __packed;

union elink_mailboxstat {
	u32 reg;
	struct {
		unsigned not_empty:1;
		unsigned full:1;
		unsigned half_full:1;
		unsigned:13;
		u16 count;
	};
} __packed;

static int epiphany_init(void)
{
	int result = 0;
	void *ptr_err = 0;

	result = alloc_chrdev_region(&dev_no, 0, 1, DRIVER_NAME);
	if (result < 0) {
		printk(KERN_ERR "Failed to register the epiphany driver\n");
		return result;
	}

	major = MAJOR(dev_no);
	dev_no = MKDEV(major, 0);

	epiphany_cdev = cdev_alloc();
	epiphany_cdev->ops = &epiphany_fops;
	epiphany_cdev->owner = THIS_MODULE;

	result = cdev_add(epiphany_cdev, dev_no, 1);
	if (result < 0) {
		printk(KERN_INFO
		       "epiphany_init() - Unable to add character device\n");
	}

	class_epiphany = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(ptr_err = class_epiphany)) {
		goto err2;
	}

	dev_epiphany = device_create(class_epiphany, NULL, dev_no, NULL,
				     DRIVER_NAME);
	if (IS_ERR(ptr_err = dev_epiphany)) {
		goto err;
	}

	elink_regs = ioremap_nocache(ELINK_REGS_BASE, ELINK_REGS_SIZE);
	if (!elink_regs) {
		printk(KERN_ERR "epiphany_init(): cannot map elink regs\n");

		ptr_err = ERR_PTR(-EIO);
		goto err;
	}

	if (request_irq(EPIPHANY_MAILBOX_IRQ_NR, epiphany_mailbox_irq_handler,
			IRQF_TRIGGER_RISING, "epiphany-mailbox",
			dev_epiphany)) {
		printk(KERN_ERR "epiphany_init(): cannot register IRQ %d\n",
		       EPIPHANY_MAILBOX_IRQ_NR);

		ptr_err = ERR_PTR(-EIO);
		goto err;
	}


#if UseReservedMem
	/* 
	 ** Use the system reserved memory until we have a way
	 ** to tell epiphany what the dynamically allocated address is
	 */
	global_shm.size = GLOBAL_SHM_SIZE;
	global_shm.flags = 0;
	global_shm.bus_addr = 0x8e000000     + 0x01000000;	/* From platform.hdf + shared_dram offset */
	global_shm.phy_addr = HOST_MEM_START + 0x01000000;	/* From platform.hdf + shared_dram offset */
	global_shm.kvirt_addr = (unsigned long)ioremap_nocache(global_shm.phy_addr, 0x01000000);	/* FIXME: not portable */
	global_shm.uvirt_addr = 0;	/* Set by user when mmapped */
	global_shm.mmap_handle = global_shm.phy_addr;
#else
	// Allocate shared memory
	// Zero the shared memory
	memset(&global_shm, 0, sizeof(global_shm));
	global_shm.size = GLOBAL_SHM_SIZE;
	global_shm.flags = GFP_KERNEL;
	global_shm.kvirt_addr = __get_free_pages(GFP_KERNEL,
						 get_order(GLOBAL_SHM_SIZE));
	if (!global_shm.kvirt_addr) {
		printk(KERN_ERR
		       "epiphany_init() - Unable to allocate contiguous "
		       "memory for global shared region\n");
		goto err;
	}

	global_shm.phy_addr = __pa(global_shm.kvirt_addr);
	global_shm.bus_addr = global_shm.phys_addr;
#endif

	// Zero the Global Shared Memory region
	memset((void *)global_shm.kvirt_addr, 0, GLOBAL_SHM_SIZE);

	printk(KERN_INFO
	       "epiphany_init() - shared memory: bus 0x%08lx, phy 0x%08lx, kvirt 0x%08lx\n",
	       global_shm.bus_addr, global_shm.phy_addr, global_shm.kvirt_addr);

	return 0;

err:
	class_destroy(class_epiphany);
err2:
	cdev_del(epiphany_cdev);
	unregister_chrdev_region(dev_no, 1);

	return PTR_ERR(ptr_err);
}

static void epiphany_exit(void)
{
#if (UseReservedMem == 0)
	free_pages(global_shm.kvirt_addr, get_order(global_shm.size));
#endif
	free_irq(EPIPHANY_MAILBOX_IRQ_NR, dev_epiphany);
	device_destroy(class_epiphany, MKDEV(major, 0));
	class_destroy(class_epiphany);
	cdev_del(epiphany_cdev);
	unregister_chrdev_region(dev_no, 1);
}

static int epiphany_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int epiphany_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct vm_operations_struct mmap_mem_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys
#endif
};

/**
 * Map memory that can be shared between the Epiphany
 * device and user-space
 */
static int epiphany_map_host_memory(struct vm_area_struct *vma)
{
	int err;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);


	err = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			vma->vm_end - vma->vm_start, vma->vm_page_prot);

	if (err) {
		printk(KERN_ERR "Failed mapping host memory to vma 0x%08lx, "
				"size 0x%08lx, page offset 0x%08lx\n",
				vma->vm_start, vma->vm_end - vma->vm_start,
				vma->vm_pgoff);
	}

	return err;
}

static int epiphany_map_device_memory(struct vm_area_struct *vma)
{
	int err, retval = 0;
	unsigned long pfn = vma->vm_pgoff;
	unsigned long size = vma->vm_end - vma->vm_start;

	vma->vm_flags |= (VM_IO | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

#ifdef EHalUsesOffsetsRatherThanAbsoluteAddress
	pfn = (EPIPHANY_MEM_START + off) >> PAGE_SHIFT;
#endif

	err = io_remap_pfn_range(vma, vma->vm_start, pfn, size,
			vma->vm_page_prot);

	if (err) {
		printk(KERN_ERR "Failed mapping device memory to vma 0x%08lx, "
				"size 0x%08lx, page offset 0x%08lx\n",
				vma->vm_start, vma->vm_end - vma->vm_start,
				vma->vm_pgoff);
		retval = -EAGAIN;
	}

	return retval;
}

static int epiphany_mmap(struct file *file, struct vm_area_struct *vma)
{
	int retval = 0;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long size = vma->vm_end - vma->vm_start;

	vma->vm_ops = &mmap_mem_ops;

	if ((EPIPHANY_MEM_START <= off ) && ((off + size) <= EPIPHANY_MEM_END)) {
		retval = epiphany_map_device_memory(vma);
	} else if ((PL_MEM_START <= off ) && ((off + size) <= PL_MEM_END)) {
		retval = epiphany_map_device_memory(vma);
	} else if ((HOST_MEM_START <= off) && ((off + size) <= HOST_MEM_END)) {
		retval = epiphany_map_host_memory(vma);
	} else {
		printk(KERN_DEBUG "epiphany_mmap - invalid request to map "
				"0x%08lx, length 0x%08lx bytes\n", off, size);
		retval = -EINVAL;
	}

	return retval;
}

static inline void reg_write(unsigned long reg, u32 val)
{
	iowrite32(val, (u8 __iomem *)elink_regs + reg);
}

static inline u32 reg_read(unsigned long reg)
{
	return ioread32((u8 __iomem *)elink_regs + reg);
}

static void epiphany_mailbox_irq_enable(void)
{
	union elink_rxcfg cfg;

	cfg.reg = reg_read(ELINK_RXCFG);
	cfg.mailbox_irq_enable = 1;
	reg_write(ELINK_RXCFG, cfg.reg);
}

static void epiphany_mailbox_irq_disable(void)
{
	union elink_rxcfg cfg;

	cfg.reg = reg_read(ELINK_RXCFG);
	cfg.mailbox_irq_enable = 0;
	reg_write(ELINK_RXCFG, cfg.reg);
}

static bool epiphany_mailbox_empty_p(void)
{
	union elink_mailboxstat status;

	status.reg = reg_read(ELINK_MAILBOXSTAT);

	return !status.not_empty;
}

static long epiphany_ioc_mailbox_read(struct file *file, void __user *dst)
{
	struct epiphany_mailbox_msg msg;

	if (mutex_lock_interruptible(&epiphany_mutex))
		return -ERESTARTSYS;

	while (epiphany_mailbox_empty_p()) {
		mutex_unlock(&epiphany_mutex);

		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (mutex_lock_interruptible(&epiphany_mutex))
			return -ERESTARTSYS;

		epiphany_mailbox_irq_enable();

		mutex_unlock(&epiphany_mutex);

		if (wait_event_interruptible(epiphany_mailbox_wq,
					     !epiphany_mailbox_empty_p()))
			return -ERESTARTSYS;

		if (mutex_lock_interruptible(&epiphany_mutex))
			return -ERESTARTSYS;
	}

	msg.from = reg_read(ELINK_MAILBOXLO);
	msg.data = reg_read(ELINK_MAILBOXHI);

	mutex_unlock(&epiphany_mutex);

	if (copy_to_user(dst, &msg, sizeof(msg)))
		return -EACCES;

	return 0;
}

static unsigned epiphany_mailbox_count(void)
{
	union elink_mailboxstat status;

	status.reg = reg_read(ELINK_MAILBOXSTAT);

	return status.count;
}

static long epiphany_ioc_mailbox_count(struct file *file)
{
	long count;

	if (mutex_lock_interruptible(&epiphany_mutex))
		return -ERESTARTSYS;

	count = epiphany_mailbox_count();

	mutex_unlock(&epiphany_mutex);

	return count;
}

static irqreturn_t epiphany_mailbox_irq_handler(int irq, void *dev_id)
{
	bool empty;

	(void) dev_id;

	mutex_lock(&epiphany_mutex);
	empty = epiphany_mailbox_empty_p();
	if (!empty)
		epiphany_mailbox_irq_disable();
	mutex_unlock(&epiphany_mutex);

	if (!empty)
		wake_up(&epiphany_mailbox_wq);

	return empty ? IRQ_NONE : IRQ_HANDLED;
}

static long epiphany_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	int retval = 0;
	int err = 0;
	epiphany_alloc_t *ealloc = NULL;

	if (_IOC_TYPE(cmd) != EPIPHANY_IOC_MAGIC) {
		return -ENOTTY;
	}

	if (_IOC_NR(cmd) > EPIPHANY_IOC_MAXNR) {
		return -ENOTTY;
	}

	if (_IOC_DIR(cmd) & _IOC_READ) {
		err =
		    !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err =
		    !access_ok(VERIFY_WRITE, (void __user *)arg,
			       _IOC_SIZE(cmd));
	}

	if (err) {
		return -EFAULT;
	}

	switch (cmd) {
	case EPIPHANY_IOC_GETSHM:
		ealloc = (epiphany_alloc_t *) (arg);
		if (copy_to_user(ealloc, &global_shm, sizeof(*ealloc))) {
			printk(KERN_ERR "EPIPHANY_IOC_GETSHM - failed\n");
			retval = -EACCES;
		}

		break;

	case EPIPHANY_IOC_MAILBOX_READ:
		retval = epiphany_ioc_mailbox_read(file, (void __user *) arg);
		break;

	case EPIPHANY_IOC_MAILBOX_COUNT:
		retval = epiphany_ioc_mailbox_count(file);
		break;

	default:		/* Redundant, cmd was checked against MAXNR */
		return -ENOTTY;
	}

	return retval;
}

module_init(epiphany_init);
module_exit(epiphany_exit);
