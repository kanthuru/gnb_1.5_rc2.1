/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2020, EdgeQ
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <linux/sizes.h>

/* Place holder. Raptor will not need SPL for CPUSS. */
#ifdef CONFIG_SPL
#define CONFIG_SPL_MAX_SIZE		0x00100000
#define CONFIG_SPL_BSS_START_ADDR	0x81400000
#define CONFIG_SPL_BSS_MAX_SIZE		0x00100000
#define CONFIG_SYS_SPL_MALLOC_START	0x82000000
#define CONFIG_SYS_SPL_MALLOC_SIZE	0x00100000
#define CONFIG_SPL_LOAD_FIT_ADDRESS	0x83000000
#endif



#if defined(CONFIG_TARGET_EQ_RAPTOR2_FPGA)

#define CONFIG_SYS_SDRAM_BASE		0x400000000
#define CONFIG_SYS_SDRAM_SIZE           0x400000000
#define R2_SDRAM_BASE_NS		0x401C00000

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_A0_EVB)

#define CONFIG_SYS_SDRAM_BASE		0x400000000
#define CONFIG_SYS_SDRAM_SIZE           0x200000000
#define R2_SDRAM_BASE_NS                0x401C00000

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_SINGLE_PPU_CLUSTER)

#define CONFIG_SYS_SDRAM_BASE		0x400000000
#define CONFIG_SYS_SDRAM_SIZE           0x400000000
#define R2_SDRAM_BASE_NS                0x401C00000

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_A0_PEGASUS)

#define CONFIG_SYS_SDRAM_BASE		0x400000000
#define CONFIG_SYS_SDRAM_SIZE           0x200000000
#define R2_SDRAM_BASE_NS                0x401C00000

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_B0_FPGA)

#define CONFIG_SYS_SDRAM_BASE		0x800000000
#define CONFIG_SYS_SDRAM_SIZE           0x200000000
#define R2_SDRAM_BASE_NS                0x801C00000

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_B0_TITAN)

#define CONFIG_SYS_SDRAM_BASE		0x800000000
#define CONFIG_SYS_SDRAM_SIZE           0x200000000
#define R2_SDRAM_BASE_NS                0x801C00000

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_B0_HAWK)

#define CONFIG_SYS_SDRAM_BASE		0x800000000
#define CONFIG_SYS_SDRAM_SIZE           0x200000000
#define R2_SDRAM_BASE_NS                0x801C00000

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_B0_PEGASUS)

#define CONFIG_SYS_SDRAM_BASE		0x800000000
#define CONFIG_SYS_SDRAM_SIZE           0x200000000
#define R2_SDRAM_BASE_NS                0x801C00000

#else

#define CONFIG_SYS_SDRAM_BASE		0x400000000
#define CONFIG_SYS_SDRAM_SIZE           0x40000000
#define R2_SDRAM_BASE_NS                0x401C00000
#endif

#define SYS_SDRAM_SIZE_8GB	0x200000000
#define SYS_SDRAM_SIZE_16GB	0x400000000
#define SYS_SDRAM_SIZE_32GB	0x800000000

#define CONFIG_SYS_INIT_SP_ADDR		(R2_SDRAM_BASE_NS + SZ_2M)

#define CONFIG_SYS_LOAD_ADDR		(R2_SDRAM_BASE_NS + SZ_2M)

#define CONFIG_SYS_MALLOC_LEN		SZ_8M

#define CONFIG_SYS_BOOTM_LEN		SZ_64M

#if (IS_ENABLED(CONFIG_TARGET_EQ_RAPTOR2_A0))
#define CONFIG_STANDALONE_LOAD_ADDR	0x401C00000
#elif (IS_ENABLED(CONFIG_TARGET_EQ_RAPTOR2_B0))
#define CONFIG_STANDALONE_LOAD_ADDR	0x801C00000
#endif

//#define CONFIG_SYS_MEMTEST_START	0x400200000
//#define CONFIG_SYS_MEMTEST_END	0x47F000000

#define CONFIG_ENV_OVERWRITE		1

#define CONFIG_SYS_MAXARGS		48

#ifdef CONFIG_TARGET_EQ_RAPTOR2_FPGA

#define COUNTER_FREQUENCY		3680000

/* Serial & console */
#define UART0_BASE			0x70380000
#define CONFIG_SYS_NS16550_SERIAL
#ifndef CONFIG_DM_SERIAL
# define CONFIG_SYS_NS16550_REG_SIZE    -4
# define CONFIG_SYS_NS16550_COM1        UART0_BASE
#endif

#define CONFIG_SYS_NS16550_MEM32

#elif (defined(CONFIG_TARGET_EQ_RAPTOR2_A0) || \
	defined(CONFIG_TARGET_EQ_RAPTOR2_B0))

/* Serial & console */
#if (IS_ENABLED(CONFIG_TARGET_EQ_RAPTOR2_B0))
#define UART0_BASE                      0x6E450000
#else
#define UART0_BASE			0x70434000
#endif

#define CONFIG_SYS_NS16550_SERIAL
#ifndef CONFIG_DM_SERIAL
# define CONFIG_SYS_NS16550_REG_SIZE    -4
# define CONFIG_SYS_NS16550_COM1        UART0_BASE
#endif

/* DCTCXO on I2C bus */
#ifdef CONFIG_I2C_DCTCXO
#define R2_A0_DCTCXO_I2C_BUS 0
#define R2_A0_DCTCXO_I2C_SLAVE_ADDR 0x60
#endif

#define CONFIG_SYS_NS16550_MEM32

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_SINGLE_PPU_CLUSTER)

#define COUNTER_FREQUENCY		3680000

/* Serial & console */
#define UART0_BASE			0x70434000
#define CONFIG_SYS_NS16550_SERIAL
#ifndef CONFIG_DM_SERIAL
# define CONFIG_SYS_NS16550_REG_SIZE    -4
# define CONFIG_SYS_NS16550_COM1        UART0_BASE
#endif

#define CONFIG_SYS_NS16550_MEM32

#else

#define UART0_BASE			0x70400000
#define COUNTER_FREQUENCY		10000000

#endif

#if defined(CONFIG_TARGET_EQ_RAPTOR2_A0_EVB)

#define COUNTER_FREQUENCY		40000000

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_A0_PEGASUS)

#define COUNTER_FREQUENCY               40960000        //40.96MHz for Pegasus board

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_B0_FPGA)

#define COUNTER_FREQUENCY               61440000

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_B0_TITAN)

#define COUNTER_FREQUENCY               61440000

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_B0_HAWK)

#define COUNTER_FREQUENCY               61440000

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_B0_PEGASUS)

#define COUNTER_FREQUENCY               61440000

#endif

/* Environment options */

#ifndef CONFIG_SPL_BUILD
#define BOOT_TARGET_DEVICES(func) \
	func(DHCP, dhcp, na)

#define	CONFIG_SYS_CBSIZE		1024	/* Console I/O Buffer Size */
#define	CONFIG_SYS_PBSIZE		1024	/* Buffer size for Console output */

#include <config_distro_bootcmd.h>

#ifdef CONFIG_TARGET_EQ_RAPTOR2_FPGA

#define CONFIG_EXTRA_ENV_SETTINGS \
	"fdt_high=0xffffffffffffffff\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"kernel_addr_r=0x410000000\0" \
	"ramdisk_addr_r=0x420000000\0" \
	"fdt_addr_r=0x430000000\0" \
	"bootargs=root=/dev/ram0 rw console=ttyS0,115200n8 \
	earlycon=edgeq8250,mmio32,0x70380040,115200n8 maxcpus=1 \
	uio_pdrv_genirq.of_id=generic-uio \0" \
	"bootcmd=booti ${kernel_addr_r} ${ramdisk_addr_r} ${fdt_addr_r}\0" \
	"bootdelay=10\0" \
	BOOTENV

#elif CONFIG_TARGET_EQ_RAPTOR2_B0_FPGA

#define CONFIG_EXTRA_ENV_SETTINGS \
	"kernel_addr_r=0x808000000\0" \
	"ramdisk_addr_r=0x80C000000\0" \
	"fdt_addr_r=0x806000000\0" \
	"zephyr_addr_r=0x8C0000000\0" \
	"bootargs=root=/dev/ram0 rw console=ttyS0,115200n8 \
	earlycon=edgeq8250,mmio32,0x6E450040,115200n8 maxcpus=1 mem=4G \
	uio_pdrv_genirq.of_id=generic-uio \0" \
	"bootcmd=booti ${kernel_addr_r} ${ramdisk_addr_r} ${fdt_addr_r}\0" \
	"bootdelay=1\0" \
	"boot_cpu=0x81010000\0" \
	"boot_zephyr_r=bootzf ${zephyr_addr_r} ${boot_cpu}\0" \
	BOOTENV

#elif CONFIG_TARGET_EQ_RAPTOR2_B0

#define CONFIG_EXTRA_ENV_SETTINGS \
	"arch=arm\0" \
	"baudrate=115200\0" \
	"board_name=titan\0" \
	"bootcmd_factory=run bootargs_factory; run load_images_f; run boot_linux_r\0" \
	"_bootargs=console=ttyS0,115200n8 earlycon=edgeq8250,mmio32,0x6E450040,115200n8 " \
	"sctp.no_checksums=Y idle=poll selinux=0 enforcing=0 nmi_watchdog=0 audit=0 irqaffinity=0 " \
	"skew_tick=1 isolcpus=2-12,14,15 nosoftlockup nohz=on nohz_full=2-12,14,15 " \
	"rcu_nocbs=2-12,14,15 sysctl.kernel.sched_min_granularity_ns=100000 " \
	"sysctl.kernel.sched_wakeup_granularity_ns=20000 sysctl.kernel.sched_latency_ns=500000 " \
	"sysctl.kernel.sched_rt_runtime_us=-1 sysctl.vm.min_free_kbytes=204800 fsck.mode=force fsck.repair=yes rootfstype=ext3\0" \
	"bootargs_factory=setenv bootargs root=/dev/ram0 rw console=ttyS0,115200n8 earlycon=edgeq8250,mmio32,0x6E450040,115200n8\0" \
	"rfs_partition=rootfs_a\0" \
	"bootargs_f=setenv bootargs root=PARTLABEL=${rfs_partition} rootwait rw ${_bootargs}\0" \
	"bootargs_r=setenv bootargs root=/dev/ram0 rw ${_bootargs}\0" \
	"boot_cmd=bootcmd_factory\0" \
	"bootcmd=run ${boot_cmd}\0" \
	"bootcmd_f=run bootargs_f; run emmc_boot\0" \
	"bootcmd_fit=run bootargs_f; run emmc_boot_fit\0" \
	"bootcmd_r=run bootargs_r; run tftp_boot\0" \
	"bootdelay=5\0" \
	"boot_linux_f=booti ${kernel_addr_r} - ${fdt_addr_r}\0" \
	"b0_board=b0_titan\0" \
	"boot_linux_fit=bootm ${fit_addr_r}#${b0_board}\0" \
	"boot_linux_r=booti ${kernel_addr_r} ${ramdisk_addr_r} ${fdt_addr_r}\0" \
	"cpu=armv8\0" \
	"emmc_boot=run emmc_load_fdt; run emmc_load_kernel; run boot_linux_f\0" \
	"emmc_boot_fit=run emmc_load_fit; run boot_linux_fit\0" \
	"linux_partition=1\0" \
	"emmc_load_fit=ext4load mmc 0:${linux_partition} ${fit_addr_r} /raptor2-B0.itb\0" \
	"ethact=eth0\0" \
	"ethlink=2\0" \
	"ethspeed=10g\0" \
	"fdt_addr_f=0xEC0000\0" \
	"fdt_addr_r=0x811000000\0" \
	"kernel_addr_f=0xA00000\0" \
	"firmware_addr_f=0x1000000\0" \
	"firmware_addr_r=0x802000000\0" \
	"fit_addr_r=0x831800000\0" \
	"flash_firmware=sf probe; sf update ${firmware_addr_r} ${firmware_addr_f} ${filesize}\0" \
	"kernel_addr_r=0x812000000\0" \
	"load_images_f=sf probe; sf read ${fdt_addr_r} ${fdt_addr_f} 0x40000; sf read ${kernel_addr_r} ${kernel_addr_f} 0x4C0000; sf read ${ramdisk_addr_r} ${ramdisk_addr_f} 0x580000\0" \
	"loopback=none\0" \
	"ramdisk_addr_f=0x480000\0" \
	"ramdisk_addr_r=0x816000000\0" \
	"soc=raptor2\0" \
	"tftp_boot=run tftp_linux; run boot_linux_r\0" \
	"tftp_fdt=tftp ${fdt_addr_r} ${img_path}/raptor2-B0-${board_name}.dtb\0" \
	"tftp_firmware=tftp ${firmware_addr_r} ${img_path}/firmware.bin\0" \
	"tftp_kernel=tftp ${kernel_addr_r} ${img_path}/Image\0" \
	"tftp_linux=run tftp_fdt; run tftp_kernel; run tftp_ramdisk\0" \
	"tftp_ramdisk=tftp ${ramdisk_addr_r} ${img_path}/uRamdisk.img\0" \
	"vendor=edgeq\0" \
	"kernel_comp_addr_r=0x82B400000\0" \
	"kernel_comp_size=0x1000000\0" \
	BOOTENV

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_A0_EVB)

#define CONFIG_EXTRA_ENV_SETTINGS \
	"spl_addr_f=0x0\0" \
	"fip_addr_f=0x100000\0" \
	"fdt_addr_f=0x200000\0" \
	"kernel_addr_f=0x300000\0" \
	"ramdisk_addr_f=0x1800000\0" \
	"spl_addr_r=0x402000000\0" \
	"fip_addr_r=0x404000000\0" \
	"zephyr_addr_r=0x4C0000000\0" \
	"fdt_addr_r=0x406000000\0" \
	"kernel_addr_r=0x408000000\0" \
	"ramdisk_addr_r=0x40C000000\0" \
	"boot_cpu=0x81000500\0" \
	"_bootargs=console=ttyS0,115200n8 " \
		"earlycon=edgeq8250,mmio32,0x70434040,115200n8 maxcpus=16 sctp.no_checksums=Y " \
		"idle=poll selinux=0 enforcing=0 nmi_watchdog=0 audit=0 kthread_cpus=0 irqaffinity=0 " \
		"skew_tick=1 isolcpus=1-15 nosoftlockup nohz=on nohz_full=1-15 rcu_nocbs=1-15 " \
		"sysctl.kernel.sched_min_granularity_ns=100000 sysctl.kernel.sched_wakeup_granularity_ns=20000 " \
		"sysctl.kernel.sched_latency_ns=500000 sysctl.kernel.sched_rt_runtime_us=-1\0" \
	"bootargs_r=setenv bootargs root=/dev/ram0 rw ${_bootargs}\0" \
	"bootargs_f=setenv bootargs root=/dev/mmcblk0p2 rootwait rw ${_bootargs}\0" \
	"flash_spl=sf probe; sf update ${spl_addr_r} ${spl_addr_f} 0x80000\0" \
	"flash_fip=sf probe; sf update ${fip_addr_r} ${fip_addr_f} ${filesize}\0" \
	"flash_firmware=sf probe; sf update ${spl_addr_r} ${spl_addr_f} ${filesize} \0" \
	"flash_fdt=sf probe; sf update ${fdt_addr_r} ${fdt_addr_f} 0x100000\0" \
	"flash_kernel=sf probe; sf update ${kernel_addr_r} ${kernel_addr_f} 0x1500000\0" \
	"flash_ramdisk=sf probe; sf update ${ramdisk_addr_r} ${ramdisk_addr_f} 0x800000\0" \
	"flash_spl_fip=run flash_spl; run flash_fip\0" \
	"flash_linux=run flash_fdt; run flash_kernel; run flash_ramdisk\0" \
	"flash_all=run flash_spl_fip; run flash_linux\0" \
	"flash_all_bootrom=run flash_firmware; run flash_linux\0" \
	"load_demo=sf probe; sf read ${fdt_addr_r} ${fdt_addr_f} 0x100000; \
		sf read ${kernel_addr_r} ${kernel_addr_f} 0x1500000; \
		sf read ${ramdisk_addr_r} ${ramdisk_addr_f} 0x800000\0" \
	"tftp_firmware=tftp ${spl_addr_r} ${img_path}/firmware.bin\0" \
	"tftp_kernel=tftp ${kernel_addr_r} ${img_path}/Image\0" \
	"tftp_ramdisk=tftp ${ramdisk_addr_r} ${img_path}/uRamdisk.img\0" \
	"tftp_fdt=tftp ${fdt_addr_r} ${img_path}/raptor2-A0.dtb\0" \
	"tftp_fdt_amp=tftp ${fdt_addr_r} ${img_path}/raptor2-A0-wlan.dtb\0" \
	"tftp_zephyr=tftp ${zephyr_addr_r} ${img_path}/zephyr.bin\0" \
	"tftp_linux=run tftp_kernel; run tftp_ramdisk; run tftp_fdt\0" \
	"tftp_linux_amp=run tftp_kernel; run tftp_ramdisk; run tftp_fdt_amp\0" \
	"tftp_boot=run tftp_linux; run boot_linux_r\0" \
	"tftp_bootz=run tftp_zephyr; run boot_zephyr_r\0" \
	"tftp_boot_amp=run tftp_linux_amp; run tftp_bootz; run boot_linux_r\0" \
	"load_firmware=ext4load mmc 0:1 ${spl_addr_r} /firmware.bin\0" \
	"load_fdt_img=ext4load mmc 0:1 ${fdt_addr_r} /raptor2-A0.dtb\0" \
	"load_kernel_img=ext4load mmc 0:1 ${kernel_addr_r} /Image\0" \
	"load_zephyr_img=ext4load mmc 0:1 ${zephyr_addr_r} /zephyr.bin\0" \
	"emmc_boot=run load_kernel_img; run load_fdt_img; run boot_linux_f\0" \
	"boot_linux_r=booti ${kernel_addr_r} ${ramdisk_addr_r} ${fdt_addr_r}\0" \
	"boot_linux_f=booti ${kernel_addr_r} - ${fdt_addr_r}\0" \
	"boot_zephyr_r=bootzf ${zephyr_addr_r} ${boot_cpu}\0" \
	"bootcmd=run bootcmd_f\0" \
	"bootcmd_f=run bootargs_f; run emmc_boot\0" \
	"bootcmd_r=run bootargs_r; run tftp_boot\0" \
	"bootdelay=5\0" \
	"ethact=eth0\0" \
	"ethlink=2\0" \
	"ethspeed=10g\0" \
	"loopback=none\0" \
	BOOTENV

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_SINGLE_PPU_CLUSTER)

#define CONFIG_EXTRA_ENV_SETTINGS \
	"fdt_high=0xffffffffffffffff\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"kernel_addr_r=0x403000000 \0" \
	"ramdisk_addr_r=0x407000000 \0" \
	"fdt_addr_r=0x402000000\0" \
	"bootargs=root=/dev/ram0 rw console=ttyS0,19200 \
	earlycon=edgeq8250,mmio32,0x70434040,115200n8 maxcpus=1 \
	uio_pdrv_genirq.of_id=generic-uio \0" \
	"bootcmd=booti ${kernel_addr_r} ${ramdisk_addr_r} ${fdt_addr_r}\0" \
	"bootdelay=10\0" \
	BOOTENV

#elif defined(CONFIG_TARGET_EQ_RAPTOR2_A0_PEGASUS)

#define CONFIG_EXTRA_ENV_SETTINGS \
	"spl_addr_f=0x0\0" \
	"fip_addr_f=0x100000\0" \
	"fdt_addr_f=0x200000\0" \
	"kernel_addr_f=0x300000\0" \
	"ramdisk_addr_f=0x1800000\0" \
	"spl_addr_r=0x402000000\0" \
	"fip_addr_r=0x404000000\0" \
	"zephyr_addr_r=0x4C0000000\0" \
	"fdt_addr_r=0x406000000\0" \
	"kernel_addr_r=0x408000000\0" \
	"ramdisk_addr_r=0x40C000000\0" \
	"boot_cpu=0x81000500\0" \
	"_bootargs=console=ttyS0,115200n8 " \
		"earlycon=edgeq8250,mmio32,0x70434040,115200n8 maxcpus=16 sctp.no_checksums=Y " \
		"idle=poll selinux=0 enforcing=0 nmi_watchdog=0 audit=0 kthread_cpus=0 irqaffinity=0 " \
		"skew_tick=1 isolcpus=1-15 nosoftlockup nohz=on nohz_full=1-15 rcu_nocbs=1-15 " \
		"sysctl.kernel.sched_min_granularity_ns=100000 sysctl.kernel.sched_wakeup_granularity_ns=20000 " \
		"sysctl.kernel.sched_latency_ns=500000 sysctl.kernel.sched_rt_runtime_us=-1\0" \
	"bootargs_r=setenv bootargs root=/dev/ram0 rw ${_bootargs}\0" \
	"bootargs_f=setenv bootargs root=/dev/mmcblk0p2 rootwait rw ${_bootargs}\0" \
	"flash_spl=sf probe; sf update ${spl_addr_r} ${spl_addr_f} 0x80000\0" \
	"flash_fip=sf probe; sf update ${fip_addr_r} ${fip_addr_f} ${filesize}\0" \
	"flash_firmware=sf probe; sf update ${spl_addr_r} ${spl_addr_f} ${filesize} \0" \
	"flash_fdt=sf probe; sf update ${fdt_addr_r} ${fdt_addr_f} 0x100000\0" \
	"flash_kernel=sf probe; sf update ${kernel_addr_r} ${kernel_addr_f} 0x1500000\0" \
	"flash_ramdisk=sf probe; sf update ${ramdisk_addr_r} ${ramdisk_addr_f} 0x800000\0" \
	"flash_spl_fip=run flash_spl; run flash_fip\0" \
	"flash_linux=run flash_fdt; run flash_kernel; run flash_ramdisk\0" \
	"flash_all=run flash_spl_fip; run flash_linux\0" \
	"flash_all_bootrom=run flash_firmware; run flash_linux\0" \
	"load_demo=sf probe; sf read ${fdt_addr_r} ${fdt_addr_f} 0x100000; \
		sf read ${kernel_addr_r} ${kernel_addr_f} 0x1500000; \
		sf read ${ramdisk_addr_r} ${ramdisk_addr_f} 0x800000\0" \
	"tftp_firmware=tftp ${spl_addr_r} ${img_path}/firmware.bin\0" \
	"tftp_kernel=tftp ${kernel_addr_r} ${img_path}/Image\0" \
	"tftp_ramdisk=tftp ${ramdisk_addr_r} ${img_path}/uRamdisk.img\0" \
	"tftp_fdt=tftp ${fdt_addr_r} ${img_path}/raptor2-A0-pegasus.dtb\0" \
	"tftp_fdt_amp=tftp ${fdt_addr_r} ${img_path}/raptor2-A0-wlan.dtb\0" \
	"tftp_zephyr=tftp ${zephyr_addr_r} ${img_path}/zephyr.bin\0" \
	"tftp_linux=run tftp_kernel; run tftp_ramdisk; run tftp_fdt\0" \
	"tftp_linux_amp=run tftp_kernel; run tftp_ramdisk; run tftp_fdt_amp\0" \
	"tftp_boot=run tftp_linux; run boot_linux_r\0" \
	"tftp_bootz=run tftp_zephyr; run boot_zephyr_r\0" \
	"tftp_boot_amp=run tftp_linux_amp; run tftp_bootz; run boot_linux_r\0" \
	"load_firmware=ext4load mmc 0:1 ${spl_addr_r} /firmware.bin\0" \
	"load_fdt_img=ext4load mmc 0:1 ${fdt_addr_r} /raptor2-A0-pegasus.dtb\0" \
	"load_kernel_img=ext4load mmc 0:1 ${kernel_addr_r} /Image\0" \
	"load_zephyr_img=ext4load mmc 0:1 ${zephyr_addr_r} /zephyr.bin\0" \
	"emmc_boot=run load_kernel_img; run load_fdt_img; run boot_linux_f\0" \
	"boot_linux_r=booti ${kernel_addr_r} ${ramdisk_addr_r} ${fdt_addr_r}\0" \
	"boot_linux_f=booti ${kernel_addr_r} - ${fdt_addr_r}\0" \
	"boot_zephyr_r=bootzf ${zephyr_addr_r} ${boot_cpu}\0" \
	"bootcmd=run bootcmd_f\0" \
	"bootcmd_f=run bootargs_f; run emmc_boot\0" \
	"bootcmd_r=run bootargs_r; run tftp_boot\0" \
	"bootdelay=5\0" \
	"ethact=eth0\0" \
	"ethlink=2\0" \
	"ethspeed=2g5\0" \
	"loopback=none\0" \
	BOOTENV

#else

#define CONFIG_EXTRA_ENV_SETTINGS \
	"fdt_high=0xffffffffffffffff\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"kernel_addr_r=0x420000000\0" \
	"ramdisk_addr_r=0x431000000\0" \
	"fdt_addr_r=0x430000000\0" \
	"bootargs=root=/dev/vda rw console=ttyAMA0,115200 \
	earlycon=pl011,mmio32,0x70400000 maxcpus=16 \
	uio_pdrv_genirq.of_id=generic-uio \0" \
	"bootcmd=booti ${kernel_addr_r} - ${fdt_addr_r}\0" \
	"bootdelay=3\0" \
	BOOTENV

#endif /* CONFIG_TARGET_EQ_RAPTOR2_FPGA || CONFIG_TARGET_EQ_RAPTOR2_SINGLE_PPU_CLUSTER */

#endif /* CONFIG_SPL_BUILD */


#endif /* __CONFIG_H */
