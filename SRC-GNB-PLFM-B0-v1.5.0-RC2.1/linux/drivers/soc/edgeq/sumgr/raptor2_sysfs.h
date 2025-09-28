/*
 * EdgeQ Inc.
 *
 * Raptor2 SYSFS header defines for Debug & Diag
 */

#define MAX_IMAGE_SIZE			(4*1024*1024)
#define MAX_ILMBUF_SIZE			(64*1024)
#define MAX_CLILMBUF_SIZE		(256*1024)
#define MAX_DLMBUF_SIZE			(128*1024)
#define MAX_LMEMBUF_SIZE		(1024*1024)

#define SU_INFO_SIZE			12
#define SU_CONF_SIZE			8
#define SU_STATS_SIZE			40

#define SU_DLM_INFO_OFFS		0x0
#define SU_DLM_CONF_OFFS		SU_INFO_SIZE
#define SU_DLM_STATS_OFFS		(SU_INFO_SIZE + SU_CONF_SIZE)
#define SU_DLM_HEARTBEAT_OFFSET		0x00000008
#define SU_ASSERT_BIT			0x00000002

typedef enum {
	rt_b,
	rt_w,
	rt_l,
	rt_q,
} rtype_t;

typedef enum {
	SU_ENTRY_EXEC_IMAGE,
	SU_ENTRY_IMAGE_SIZE,
	SU_ENTRY_RESET,
	SU_ENTRY_INFO,
	SU_ENTRY_CONF,
	SU_ENTRY_REGS,
	SU_ENTRY_AU,
	SU_ENTRY_ILM,
	SU_ENTRY_ILM_0,
	SU_ENTRY_DLM,
	SU_ENTRY_DLM_0,
	SU_ENTRY_LMEM,
	SU_ENTRY_LMEM_0,
	SU_ENTRY_CLILM,
	SU_ENTRY_CLILM_0,
	SU_ENTRY_sPLIC_INT,
	SU_ENTRY_STATS,
	SU_ENTRY_DBG,
	SU_ENTRY_MEM_ENABLE,
	SU_ENTRY_HART_ID,
	SU_ENTRY_HEART_BEAT,
	SU_ENTRY_TRIGGER,
	/* Always the last one, do not remove */
	SU_ENTRY_NONE= 255,
} r2_su_entry_t;


struct r2_sysfs_entry
{
	r2_su_entry_t stype;
	const char *name;
};

struct r2_sysfs_entry r2_sysfs_entry_table [] = {
	{SU_ENTRY_EXEC_IMAGE,		"exec_image"},
	{SU_ENTRY_IMAGE_SIZE,		"image_size"},
	{SU_ENTRY_RESET,		"reset"},
	{SU_ENTRY_INFO,			"info"},
	{SU_ENTRY_CONF,			"conf"},
	{SU_ENTRY_REGS,			"regs"},
	{SU_ENTRY_AU,			"au"},
	{SU_ENTRY_ILM,			"ilm"},
	{SU_ENTRY_ILM_0,		"ilm_0"},
	{SU_ENTRY_DLM,			"dlm"},
	{SU_ENTRY_DLM_0,		"dlm_0"},
	{SU_ENTRY_LMEM,			"lmem"},
	{SU_ENTRY_LMEM_0,		"lmem_0"},
	{SU_ENTRY_CLILM,		"clilm"},
	{SU_ENTRY_CLILM_0,		"clilm_0"},
	{SU_ENTRY_sPLIC_INT,		"sPLIC_int"},
	{SU_ENTRY_STATS,		"stats"},
	{SU_ENTRY_DBG,			"dbg"},
	{SU_ENTRY_MEM_ENABLE,		"mem_enable"},
	{SU_ENTRY_HART_ID,		"hart_id"},
	{SU_ENTRY_HEART_BEAT,		"heart_beat"},
	{SU_ENTRY_TRIGGER,		"trigger"},
	/* Always the last one, all new entries before */
	{SU_ENTRY_NONE,			NULL},
};

struct r2_kobj_entry {
	//struct kobj_attribute image_attr;
	struct bin_attribute image_attr;
	struct kobj_attribute image_size_attr;
	struct kobj_attribute reset_attr;
	struct kobj_attribute info_attr;
	struct kobj_attribute conf_attr;
	struct kobj_attribute regs_attr;
	struct kobj_attribute au_attr;
	struct bin_attribute ilm_attr;
	struct kobj_attribute ilm0_attr;
	struct bin_attribute dlm_attr;
	struct kobj_attribute dlm0_attr;
	struct bin_attribute lmem_attr;
	struct kobj_attribute lmem0_attr;
	struct bin_attribute clilm_attr;
	struct kobj_attribute clilm0_attr;
	struct kobj_attribute splic_int_attr;
	struct kobj_attribute stats_attr;
	struct kobj_attribute dbg_attr;
	struct kobj_attribute su_mem_enable_attr;
	struct kobj_attribute hartid_attr;
	struct kobj_attribute heartbeat_attr;
	struct kobj_attribute trigger_attr;
};
