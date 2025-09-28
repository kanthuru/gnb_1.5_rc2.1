/* SPDX-License-Identifier: GPL-2.0 */
/**
 * Test driver to edgeq endpoint functionality header
 *
 * Copyright (C) 2021 EdgeQ Inc.
 * Author: Ankit Jindal <ankit.jindal@edgeq.io>
 * Author: Pranavkumar Sawargaonkar <pranav.sawargaonkar@edgeq.io>
 */

#ifndef __UIO_EPF_EDGEQ__
#define __UIO_EPF_EDGEQ__

#define KB(x)                                   ((x)*1024ULL)
#define MB(x)                                   (KB(x)*1024ULL)

#define PCI_VENDOR_ID_EDGEQ                     0x1ece
#define PCI_DEVICE_ID_LINK3						0xfff1
#define PCI_DEVICE_ID_LINK4						0xffff

/* Max Number of Devices supported */
#define MAX_EP_FUNC_DEVS                        1
#define HOST_DEV_DATA_OFFSET                    8192

/* Queue Parameters */
#define MAX_QUEUE_SIZE                          MB(2)
#define MAX_MSG_SIZE                            KB(32)
#define MAX_MSGS                                64
#define MAX_QUEUES                              16
#define MAX_ADD_QUEUES                          16
#define MAX_NUM_QUEUES                          32
#define MAX_QUEUE_MEM                           MB(64)

/* Host side addtional status Bits in Global DATA */
#define STATUS_HOST_INIT_DEV                    (1UL << 32)

/* Max EP devices */
#define MAX_PER_EP_UIO_DEVS                     10
#define MAX_DEVICE                              4

#define DDR_SIZE_GBL                            KB(256)         /* Global Data size */
#define DDR_SIZE_QM                             MB(64)          /* QM Size */
#define DDR_SIZE_DMA(ccix_active) ((ccix_active)?(MB(64)):(MB(128)))	/* DATA size */

/* GLOBAL Data Parameters */
#define GLOB_START_OFFSET(ccix_active) ((ccix_active)?(0xF80000):(0xF80000))
#define GLOB_STRUCT_SIZE                        KB(16)
#define GLOB_DEVICE_SIZE                        KB(24)
#define GLOB_TOTAL_SIZE                         KB(256)

#define DMA_RLL_ADDR				0x9A3000000
#define DMA_RLL_SIZE				0x4000000

/* Status Bits in GLOBAL_DATA */
#define STATUS_EP_NOT_READY                     0
#define STATUS_EP_READY                         (1UL << 0)
#define STATUS_EP_GBL_READY                     (1UL << 1)
#define STATUS_EP_DEV_INIT_REQ                  (1UL << 2)
#define STATUS_EP_DEV_INIT_IN_PROGRESS          (1UL << 3)
#define STATUS_EP_DEV_INIT_DONE                 (1UL << 4)
#define STATUS_EP_MAP_IN_PROGRESS               (1UL << 5)
#define STATUS_EP_MAP_DONE                      (1UL << 6)
#define STATUS_EP_MAP_FAIL                      (1UL << 7)

/* Netlink Socket Id */
#define NETLINK_EDGEQ_PCI(ccix_active)          ((ccix_active)?(30):(29))
#define MAX_NETLINK_PAYLOAD                     1024

/* Work request type */
#define NO_OP                                   0
#define POLL_FLAG                               1
#define POLL_FOR_REMOTE                         2

/* BAR Address Index */
#define EDGEQ_DMA_BAR                           0
#define EDGEQ_DDR_BAR                           2

#define IS_DEV_READY(x)                         ((x) & (STATUS_EP_INIT_DONE))
#define SET_DEV_READY(x)                        ((x) | (STATUS_EP_INIT_DONE))

/* UIO NODES */
#define EDGEQ_BAR_UIO                           0
#define EDGEQ_QM_DATA_UIO                       1
#define EDGEQ_GLOBAL_UIO                        2
#define EDGEQ_DMA_UIO                           3

/* DMA Parameters */
#define DMA_DESC_SIZE                           24
#define DMA_SIZE(ccix_active)                   ((ccix_active) ? (256) : (512))
#define DMA_MAX_ELE                             256
#define DMA_MAX_CH_SIZE                         MB(3)
#define DMA_NUM_CH				16
#define MAX_CH_MEM(ccix_active)                 ((ccix_active) ? (MB(64)) : (MB(128)))

#define R2_DEBUG				0	/* Set to 1 to enable logs */

#define R2PR_LOG_ERR(...)			pr_err(__VA_ARGS__)
#if R2_DEBUG
#define R2PR_LOG(...)				pr_info(__VA_ARGS__)
#else
#define R2PR_LOG(...)
#endif
/*************************** GLOBAL Structures ************************/
typedef struct global_data {
	uint64_t num_devices;                   // Number of device requested
	uint64_t status;

	uint64_t num_queues;                    // [USER]Number of queues
	uint64_t available_queues;              // Unclaimed queues
	uint64_t queue_map;                     // Bit map with one bit reps. one queue
	uint64_t queue_region_addr;             // [USER]Host queue address
	uint64_t queue_region_size;             // [USER]Sum of contiguous memory

	uint64_t num_dma_read_ch;               // Number of DMA read channels
	uint64_t available_read_ch;
	uint64_t dma_read_chan_map;             // DMA read channel bit map

	uint64_t num_dma_write_ch;              // Number of DMA write channels
	uint64_t available_write_ch;
	uint64_t dma_write_chan_map;            // DMA write channel bit map

	uint64_t dma_region_addr;               // [USER]DMA physical address
	uint64_t dma_region_size;               // [USER]Sum of contiguous memory

	/* Below 2 fields are ignored in case of host and are used by EP only*/
	uint64_t host_global_paddr;             // Host side physical address
	uint64_t host_global_size;              // Host side global region size
	uint64_t remote_glob_lpaddr;            // Remote side's glob local paddr

	uint64_t ep_bar_ddr_paddr;              // EP side physical address

	uint64_t dma_fault;			// Flag for DMA Fault handling
	uint64_t seqt_enable;			// Enable Single EQT flag for DMA
} global_t;

typedef struct QUEUES {
	uint64_t queue_id;
	uint64_t msg_size;
	uint64_t num_msgs;
	uint64_t queue_paddr;
	uint64_t f_index;
	uint64_t r_index;
	uint64_t sign;
	uint64_t allocated;
	uint64_t tx_claimed;
	uint64_t rx_claimed;
} glob_queue;

typedef struct mem_region {
	uint64_t paddr;
	uint64_t vaddr;
	uint64_t size;
} mem_region_t;

typedef struct QM {
	uint64_t num_queues;
	uint64_t queue_map;
	uint64_t max_msg_size;
	uint64_t queue_init_complete;
	glob_queue queue[MAX_QUEUES];
} glob_qm;

typedef struct DMA_PARAMS {
	int64_t curr_weight;
	uint64_t ll_start_paddr;
	uint64_t dma_req_index;
	uint64_t dma_xfer_index;
	int32_t weight[DMA_MAX_ELE];
} glob_dma_params;

typedef struct DMA {
	uint64_t dma_cpu_id;
	uint64_t dma_read_chan_map;
	uint64_t num_read_channel;

	uint64_t dma_write_chan_map;
	uint64_t num_write_channel;

	glob_dma_params dma_chan_params[DMA_NUM_CH];
} glob_dma_desc;

typedef struct DEVICE_DATA {
	uint64_t started;
	uint64_t ready;
	uint64_t device_id;
	mem_region_t data_mem_region;
	glob_qm dev_qm;
	glob_dma_desc dev_dma;
} glob_device_t;


/******************************* Netlink Specific Data ****************************/
#define REQ_SET_FTL_DESC                        1
#define REQ_INIT_DEV                            2
#define REQ_WAIT_DESC                           3
#define REQ_EXPOSE_REG_UIO                      4

#define MAX_UIO_NAME_SIZE                       32
#define UIO_MAX_MAPS                            5

struct edgeq_user_data {
	int     req_type;
	int     req_status;                     // 0: Success -1: Fail
	char    data[512];
};

/* Work Queue structure */
struct edgeq_ucmd_work {
	int req_type;
	struct delayed_work cmd_handler;
	void *data;
};

/* FTL Descriptors */
struct ftl_descriptor {
	uint64_t num_devices;                   // [USER]Total number of devices supported

	// QUEUE POOL
	uint64_t queue_region_addr;             // [USER]Physical queue address
	uint64_t queue_region_size;             // [USER]Sum of contiguous memory

	// DMA Channels
	uint64_t num_dma_read_ch;
	uint64_t num_dma_write_ch;

	// DMA POOL: LL Region; for Host only
	uint64_t dma_region_addr;               // [USER]DMA physical address
	uint64_t dma_region_size;               // [USER]Sum of contiguous memory

	// DMA POOL: LL Region; for EP only
	uint64_t ep_dma_region_addr;            // [USER]DMA physical address
	uint64_t ep_dma_region_size;            // [USER]Sum of contiguous memory

	uint64_t seqt_enable;			// [USER]Single EQT enable
};

struct device_descriptor {
	uint64_t device_id;
	uint64_t num_queues;
	uint64_t num_dma_read_chan;
	uint64_t num_dma_write_chan;
	uint64_t cpu_id;
};

/* UIO Device Descriptor */
struct uio_descriptor {
	int uio_dev_id;                         /* QM, DATA etc */
	int uio_map_count;                      /* Number of maps */
	int map_id[UIO_MAX_MAPS];               /* map0, map1 etc */
	unsigned long map_paddr[UIO_MAX_MAPS];  /* Hugepage physical address */
	unsigned long map_size[UIO_MAX_MAPS];   /* Hugepage size */
	char uio_name[MAX_UIO_NAME_SIZE];
};

struct wait_flags {
	uint64_t        dev_id;
	bool            linkup_ready;
	bool            is_local_global_mem_mapped;
	bool            is_remote_global_mem_mapped;
	bool            is_local_device_ready;
};

/************************** Raptor2 EdgeQ Structure *****************************/

/*
 * Represents a Global structure
 *	MAP 0: GLOBAL REG SPACE
 */

struct raptor2_global {
	struct uio_info                 *gdata_info;
	global_t                        *glob;                  /* Pointer */
	phys_addr_t                     glob_paddr;

	mem_region_t                    dma_reg_region;         /* BAR 0: DMA REGISTER */
	mem_region_t                    ddr_region;             /* BAR 2: DDR REGION   */
	mem_region_t                    queue_region;           /* Queue Region */
	mem_region_t                    ll_region;

	/* Status Flags */
	bool                   is_global_mem_mapped;
	bool                   is_qm_mem_mapped;
	bool                   is_dma_reg_mem_mapped;
};

/*
 *Represents a device structure:
 *	MAP 0 - LOCAL DEV REG SPACE
 *	MAP 1 - REMOTE DEV REG SPACE
 *	MAP 2 - REMOTE QUEUE
 *	MAP 3 - DMA REG SPACE
 */
struct raptor2_device {
	struct uio_info         *dev_info;

	mem_region_t            device_reg_space;

	bool           is_claimed;
	bool           is_local_dreg_ready;
	bool           is_remote_dreg_ready;
	bool           is_remote_queue_ready;
	bool           is_dma_reg_ready;
	bool           is_device_init_done;
};

/* Represents a Raptor2 Device */
struct raptor2_prv_data {
	struct sock                     *netlink_sk;
	struct workqueue_struct         *r2_wrkq;
	struct edgeq_ucmd_work          *edgeq_work;
	struct edgeq_ucmd_work          *edgeq_work2;
	struct pci_dev                  *pdev;
	struct device                   *dev;
	struct uio_info                 *bar_ddr;
	bool                   linkup_ready;

	/* Pointers to local structures */
	struct raptor2_global           *local_gbl;
	bool                   exposed_local_glob_region;

	/* Pointers to global structures */
	struct raptor2_global           *remote_gbl;
	bool                   exposed_remote_glob_region;

	/* Pointer to local device structure */
	struct raptor2_device           *raptor2_ldev[MAX_DEVICE];
	bool                   exposed_local_device[MAX_DEVICE];
};

static void edgeq_netlink_recv(struct sk_buff *skb);

static void test_read(phys_addr_t vaddr, size_t bytes_sz)
{
	int i = 0;
	uint64_t *addr = (uint64_t *) vaddr;

	R2PR_LOG("######## TEST READ ########\n");
	for (i = 0; i <= (bytes_sz/8); i++) {
		R2PR_LOG("Address=%llx \t Value =%llx\n", (uint64_t)addr, *addr);
		addr++;
	}
	R2PR_LOG("##############################\n");
}

void uio_print_recv_flags(struct wait_flags *flg)
{
	R2PR_LOG("###########################################\n");
	pr_info("# DEVICE ID                           =%lld #\n",
		flg->dev_id);
	pr_info("# linkup_ready                        =%d #\n",
		flg->linkup_ready);
	pr_info("# is_local_global_mem_mapped          =%d #\n",
		flg->is_local_global_mem_mapped);
	pr_info("# is_remote_global_mem_mapped         =%d #\n",
		flg->is_remote_global_mem_mapped);
	pr_info("# is_local_device_ready               =%d #\n",
		flg->is_local_device_ready);
	pr_info("###########################################\n");
}

void uio_decode_print_status(uint64_t status)
{
	R2PR_LOG("#################################\n");
	R2PR_LOG("STATUS: SIDE NOT READY        =%lld\n",
		(status & 0x1));
	R2PR_LOG("STATUS: SIDE READY            =%lld\n",
		((status & STATUS_EP_READY)>>0));
	R2PR_LOG("STATUS: GLOBAL READY          =%lld\n",
		((status & STATUS_EP_GBL_READY)>>1));

	R2PR_LOG("STATUS: DEVICE INIT REQ       =%lld\n",
		((status & STATUS_EP_DEV_INIT_REQ)>>2));
	R2PR_LOG("STATUS: DEVICE INIT STARTED   =%lld\n",
		((status & STATUS_EP_DEV_INIT_IN_PROGRESS)>>3));
	R2PR_LOG("STATUS: DEVICE INIT DONE      =%lld\n",
		((status & STATUS_EP_DEV_INIT_DONE)>>4));

	R2PR_LOG("STATUS: MAP INIT STARTED      =%lld\n",
		((status & STATUS_EP_MAP_IN_PROGRESS)>>5));
	R2PR_LOG("STATUS: MAP INIT DONE         =%lld\n",
		((status & STATUS_EP_MAP_DONE)>>6));
	R2PR_LOG("STATUS: MAP INIT FAILED       =%lld\n",
		((status & STATUS_EP_MAP_FAIL)>>7));
	R2PR_LOG("#################################\n");
}

void uio_dump_glob(global_t *glb)
{
	R2PR_LOG("Dumping global structure\n");
	test_read((uint64_t)glb, sizeof(global_t));
}

void uio_dump_device(glob_device_t *glbdev)
{
	R2PR_LOG("Dumping device structure\n");
	R2PR_LOG("#################################\n");
	R2PR_LOG("Device ID =%lld\n",
		glbdev->device_id);

	R2PR_LOG("Num Queues =%lld\n",
		glbdev->dev_qm.num_queues);
	R2PR_LOG("Queue BMAP =%lld\n",
		glbdev->dev_qm.queue_map);

	R2PR_LOG("DMA CPU ID =%lld\n",
		glbdev->dev_dma.dma_cpu_id);
	R2PR_LOG("DMA Read Channel Map =%lld\n",
		glbdev->dev_dma.dma_read_chan_map);
	R2PR_LOG("DMA Num Read Channel =%lld\n",
		glbdev->dev_dma.num_read_channel);
	R2PR_LOG("DMA Write Channel Map =%lld\n",
		glbdev->dev_dma.dma_write_chan_map);
	R2PR_LOG("DMA Num Read Channel =%lld\n",
		glbdev->dev_dma.num_write_channel);
	R2PR_LOG("#################################\n");
}

#endif  /* __UIO_EPF_EDGEQ__ */
