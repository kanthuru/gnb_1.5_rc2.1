/*
 * Utility functions to perform mem test
 */

#include <common/debug.h>
#include <lib/mmio.h>
#include <platform_def.h>
#include <lib/spinlock.h>
#include <plat/common/platform.h>
#include <lib/psci/psci.h>


#define  L1_CACHE_SIZE     0x008000
#define  L2_CACHE_SIZE     0x040000
#define  L3_CACHE_SIZE     0x100000


#define  MEM_TEST_BASE_ADDR   UL(0x402000000)
#define  MEM_TEST_SIZE        UL(0x000400000)  // 4 MB


#define  LOOP_SIZE( _write_size, _write_var )    ( (_write_size)/sizeof(_write_var) )

#define  SLEEP_CYCLES(_count)                       \
    do {					    \
        volatile int _sleep_count=_count;	    \
	while(_sleep_count--);			    \
    }while(0)

// Memory test sync & pattern
static spinlock_t mem_test_lock;
static uint32_t   last_write_cpu_id;

// CPU ID sync
static spinlock_t iteration_lock;
static volatile uint32_t   iteration_count;

// Start count sync
static spinlock_t start_lock;
static volatile uint32_t   start_count;

// Test count.
static volatile uint32_t   test_count;

static void  mem_test_set_cpu_id(uint32_t cpu_id)
{
    last_write_cpu_id = cpu_id;
    return;
}

static uint32_t  mem_test_write_get_cpu_id()
{
    return last_write_cpu_id;
}

int  mem_test_write_pattern(uintptr_t base, unsigned int size, unsigned int cpu_id)
{
    uint32_t loop=0;
    uint64_t pattern=0;
    uint32_t loop_size = LOOP_SIZE( size, pattern);

    for(loop=0; loop < loop_size; loop++ ) {
	mmio_write_64( (base + (loop * 8)), (base + cpu_id + (loop * 8)));
    }

    return 0;
}


int mem_test_read_pattern(uintptr_t base, unsigned int size, unsigned int cpu_id)
{
    uint32_t loop=0;
    uint64_t pattern=0;
    uint64_t rpattern=0;
    uint32_t loop_size = LOOP_SIZE( size, pattern);

    for(loop=0; loop < loop_size; loop++ ) {
	rpattern = mmio_read_64(base + (loop * 8) );
	pattern = base + cpu_id + (loop * 8);
	if( rpattern != pattern )
	{
	    INFO("Write Pattern: %llx Read Pattern: %llx loop_index: %d \n", pattern, rpattern, loop);
	    volatile int test_loop=1;
	    while(test_loop);
	    return 1;
	}
    }

    return 0;
}

/*
 *  Common function entry for all cores (primary+secondary) to perform memory test.
 *  Below is the high level procedure.
 *     1. Acquire the spin lock.
 *     2. Get the last written CPU ID.
 *     3. Read memory and verify the pattern.
 *     4. Write new pattern with CPU_ID as the magic.
 *     5. Write the CPU_ID that performed oepration (4).
 */
int  memory_operation_test_common()
{
    uint32_t my_cpu_id=0;
    uint32_t op_cpu_id=0;

    while(1)
    {
    	spin_lock(&start_lock);
    	printf("Core %d entering\n", plat_my_core_pos());
    	start_count++;
    	spin_unlock(&start_lock);
    	while(start_count < 16);

	spin_lock(&mem_test_lock);
	//INFO("Acquired mem_test_lock\n");   Tough to sync this log line. Commenting for now.

	my_cpu_id = plat_my_core_pos();
	op_cpu_id = mem_test_write_get_cpu_id();
	mem_test_read_pattern(MEM_TEST_BASE_ADDR, MEM_TEST_SIZE, op_cpu_id);
	INFO("Memtest passed for Curr_CPUID: %d OP_CPU_ID: %d\n", my_cpu_id, op_cpu_id);
	mem_test_write_pattern(MEM_TEST_BASE_ADDR, MEM_TEST_SIZE, my_cpu_id);
	mem_test_set_cpu_id(my_cpu_id);
	//INFO("Memtest write completed for Curr_CPU_ID: %d OP_CPU_ID: %d\n", my_cpu_id, my_cpu_id);

	spin_unlock(&mem_test_lock);
	//INFO("mem_test_lock released\n");  Tough to sync this log line. Commenting for now.

	spin_lock(&iteration_lock);
	if( iteration_count != 15 )
	{
             iteration_count++;
	}
	else 
	{
	    start_count = 0;
	    test_count++;
	    iteration_count = 0;
	}
	spin_unlock(&iteration_lock);

	while( iteration_count != 0 );

	while( test_count >= 4 );
    }

    return 0;
}


/*
 * First write from Primary CPU.
 */
void primary_cpu_start_function()
{
    uint32_t cpu_id = plat_my_core_pos();

    INFO("Primary CPU initial write operation\n");
    /* Start with initial write of memory pattern on primary CPU. */
    spin_lock( &mem_test_lock );
    mem_test_set_cpu_id(cpu_id);
    mem_test_write_pattern(MEM_TEST_BASE_ADDR, MEM_TEST_SIZE, cpu_id);
    spin_unlock( &mem_test_lock );

    /* Wake up secondary cores and primary core to perfom sleep cycles. */
    // TODO: PSCI Call to poweron secondary CPU's.
    psci_cpu_on(0x81000001, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81000100, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81000101, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81000200, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81000201, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81000300, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81000301, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81000400, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81000401, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81000500, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81000501, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81010000, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81010001, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81010100, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81010101, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81010200, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81010201, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81010300, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);
    psci_cpu_on(0x81010301, (uintptr_t)memory_operation_test_common, 0);
    SLEEP_CYCLES(0xFFFF);

    INFO("Primary CPU finished operation.\n");

    memory_operation_test_common();

    return;
}


#if 0
extern void plat_invalidate_icache(void);

//static spinlock_t test_lock;

static int  spin_lock_test(uintptr_t base, unsigned int size, unsigned int test_iterations)
{
    uint32_t loop=0;
    uint32_t test_cnt=0;
    uint64_t pattern=0;
    uint64_t rpattern = 0;
    uint32_t loop_size = (size/sizeof(pattern));

    printf("Start writing DDR loop. Base: %lx Size: %x \n", base, size);
    for(test_cnt=0; test_cnt < test_iterations; test_cnt++)
    {
	printf("Base : %lx \n", base);
	plat_invalidate_icache();

	//spin_lock( &test_lock );
	for(loop=0; loop < loop_size; loop++ ) {
	    mmio_write_64( (base + (loop * 8)), (base + (loop * 8)));
	}

	printf("Start Reading DDR loop. Size: %x Itr: %d\n", size, test_cnt);
	for(loop=0; loop < loop_size; loop++ ) {
	    rpattern = mmio_read_64(base + (loop * 8) );
	    pattern = base + (loop * 8);
	    if( rpattern != pattern )
	    {
		printf("Write Patter: %llx Read Pattern: %llx loop_index: %d \n", pattern, rpattern, loop);
		volatile int test_loop=1;
		while(test_loop);
		return 1;
	    }

	}
	printf("spin lock test size: %x Itr: %d success\n", size, test_cnt);

	//spin_unlock( &test_lock );
	base = base + size;
    }

    return 0;
}

static int ddr_test_data_bus(void)
{
#define  L1_CACHE_SIZE     0x008000
#define  L2_CACHE_SIZE     0x040000
#define  L3_CACHE_SIZE     0x100000

    return 0;

    spin_lock_test( 0x401000000, (L1_CACHE_SIZE * 2), 1024 );
    spin_lock_test( 0x401000000, ((L2_CACHE_SIZE) * 2), 256 );
    spin_lock_test( 0x401000000, ((L3_CACHE_SIZE) * 2), 64 );

    return 0;
}

#endif
