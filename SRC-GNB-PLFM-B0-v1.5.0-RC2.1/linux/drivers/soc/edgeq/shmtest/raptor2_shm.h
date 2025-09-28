/*
 * EdgeQ Inc.
 *
 * Raptor2 Header file for SHM Testing
 */


#define RAPTOR2_WLAN_SHM_DEV_NAME	"r2wlanshm"
#define RAPTOR2_WLAN_SHM_PHYS_BASE	0x402000000
#define RAPTOR2_WLAN_SHM_SIZE		0x01000000
#define RAPTOR2_SHM_MAX_PROCS		1


#ifdef __KERNEL__
struct shm_mapping_access {
	spinlock_t lock;
	u64 virtaddr;
	int size;
	uint32_t nprocs;
};
#endif
