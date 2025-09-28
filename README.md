## To compile Linux

#REQUIREMENT:
$ aarch64-linux-gnu-gcc -v
Using built-in specs.
COLLECT_GCC=aarch64-linux-gnu-gcc
COLLECT_LTO_WRAPPER=/usr/lib/gcc-cross/aarch64-linux-gnu/10/lto-wrapper
Target: aarch64-linux-gnu
Configured with: ../src/configure -v --with-pkgversion='Ubuntu 10.5.0-1ubuntu1~20.04' --with-bugurl=file:///usr/share/doc/gcc-10/README.Bugs --enable-languages=c,ada,c++,go,d,fortran,objc,obj-c++,m2 --prefix=/usr --with-gcc-major-version-only --program-suffix=-10 --enable-shared --enable-linker-build-id --libexecdir=/usr/lib --without-included-gettext --enable-threads=posix --libdir=/usr/lib --enable-nls --with-sysroot=/ --enable-clocale=gnu --enable-libstdcxx-debug --enable-libstdcxx-time=yes --with-default-libstdcxx-abi=new --enable-gnu-unique-object --disable-libquadmath --disable-libquadmath-support --enable-plugin --enable-default-pie --with-system-zlib --enable-libphobos-checking=release --without-target-system-zlib --enable-multiarch --enable-fix-cortex-a53-843419 --disable-werror --enable-checking=release --build=x86_64-linux-gnu --host=x86_64-linux-gnu --target=aarch64-linux-gnu --program-prefix=aarch64-linux-gnu- --includedir=/usr/aarch64-linux-gnu/include --with-build-config=bootstrap-lto-lean --enable-link-mutex
Thread model: posix
Supported LTO compression algorithms: zlib
gcc version 10.5.0 (Ubuntu 10.5.0-1ubuntu1~20.04)

#COMPILATION:
export INSTALL_MOD_PATH="MODULE_INSTALL"
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- defconfig && \
	make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image -j`nproc` && \
	make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- dtbs && \
	make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules -j`nproc` && \
	make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules_install
mkimage -f arch/arm64/boot/raptor2-B0.its arch/arm64/boot/raptor2-B0.itb
cd MODULE_INSTALL
find . -name *.ko > modules.list && for line in `cat modules.list`; do aarch64-linux-gnu-strip --strip-debug $line; done && rm modules.list
mkdir -p <path-to-buildroot>/buildroot/board/edgecompute/raptor2/rootfs-overlay/usr/lib/modules
cp -r MODULE_INSTALL/lib/modules/* <path-to-buildroot>/buildroot/board/edgecompute/raptor2/rootfs-overlay/usr/lib/modules/

#UPDATE:
1. Power-up the board and boot into the Linux
2. tftp/scp the "raptor2-B0.itb" file in boot partition of the emmc
3. Reboot the board to apply the changes.



## To compile buildroot

#REQUIREMENT:
If there is change in linux, compile the same first.

#COMPILATION:
make edgeq_raptor2_defconfig && make -j`nproc`

#UPDATE:
1. Halt the bootup at u-boot.
2. run bootcmd_factory
3. Once linux boots up, tftp/scp rootfs.ext3 to the /tmp on the board.
4. RUN: mkfs.ext3 -F /dev/mmcblk0p14 -m 0 && dd if=/tmp/rootfs.ext3 of=/dev/mmcblk0p14
5. Once this is done tftp/scp two files from ${REL DIR}/FLASHER: fstab_create partition.xml
6. RUN: chmod 777 fstab_create && ./fstab_create
7. This will generate fstab file which can be copied to rootfs by running below steps:
	a. mount /dev/mmcblk0p14 /mnt
	b. cp fstab /mnt/etc/fstab
	c. sync && umount /mnt
	d. sync && reboot


## To compile U-Boot

#REQUIREMENT:
$ aarch64-linux-gnu-gcc -v
Using built-in specs.
COLLECT_GCC=aarch64-linux-gnu-gcc
COLLECT_LTO_WRAPPER=/usr/lib/gcc-cross/aarch64-linux-gnu/10/lto-wrapper
Target: aarch64-linux-gnu
Configured with: ../src/configure -v --with-pkgversion='Ubuntu 10.5.0-1ubuntu1~20.04' --with-bugurl=file:///usr/share/doc/gcc-10/README.Bugs --enable-languages=c,ada,c++,go,d,fortran,objc,obj-c++,m2 --prefix=/usr --with-gcc-major-version-only --program-suffix=-10 --enable-shared --enable-linker-build-id --libexecdir=/usr/lib --without-included-gettext --enable-threads=posix --libdir=/usr/lib --enable-nls --with-sysroot=/ --enable-clocale=gnu --enable-libstdcxx-debug --enable-libstdcxx-time=yes --with-default-libstdcxx-abi=new --enable-gnu-unique-object --disable-libquadmath --disable-libquadmath-support --enable-plugin --enable-default-pie --with-system-zlib --enable-libphobos-checking=release --without-target-system-zlib --enable-multiarch --enable-fix-cortex-a53-843419 --disable-werror --enable-checking=release --build=x86_64-linux-gnu --host=x86_64-linux-gnu --target=aarch64-linux-gnu --program-prefix=aarch64-linux-gnu- --includedir=/usr/aarch64-linux-gnu/include --with-build-config=bootstrap-lto-lean --enable-link-mutex
Thread model: posix
Supported LTO compression algorithms: zlib
gcc version 10.5.0 (Ubuntu 10.5.0-1ubuntu1~20.04)

>> Untar the following packages:
u-boot.tar.gz and FIRMWARE.tar.gz

#COMPILATION:
cd u-boot
export CROSS_COMPILE=aarch64-linux-gnu- && export ARCH=arm64
make edgeq_raptor2_b0_hawk_defconfig && make -j`nproc`



## To compile ATF

#REQUIREMENT:
$ aarch64-linux-gnu-gcc -v
Using built-in specs.
COLLECT_GCC=aarch64-linux-gnu-gcc
COLLECT_LTO_WRAPPER=/usr/lib/gcc-cross/aarch64-linux-gnu/10/lto-wrapper
Target: aarch64-linux-gnu
Configured with: ../src/configure -v --with-pkgversion='Ubuntu 10.5.0-1ubuntu1~20.04' --with-bugurl=file:///usr/share/doc/gcc-10/README.Bugs --enable-languages=c,ada,c++,go,d,fortran,objc,obj-c++,m2 --prefix=/usr --with-gcc-major-version-only --program-suffix=-10 --enable-shared --enable-linker-build-id --libexecdir=/usr/lib --without-included-gettext --enable-threads=posix --libdir=/usr/lib --enable-nls --with-sysroot=/ --enable-clocale=gnu --enable-libstdcxx-debug --enable-libstdcxx-time=yes --with-default-libstdcxx-abi=new --enable-gnu-unique-object --disable-libquadmath --disable-libquadmath-support --enable-plugin --enable-default-pie --with-system-zlib --enable-libphobos-checking=release --without-target-system-zlib --enable-multiarch --enable-fix-cortex-a53-843419 --disable-werror --enable-checking=release --build=x86_64-linux-gnu --host=x86_64-linux-gnu --target=aarch64-linux-gnu --program-prefix=aarch64-linux-gnu- --includedir=/usr/aarch64-linux-gnu/include --with-build-config=bootstrap-lto-lean --enable-link-mutex
Thread model: posix
Supported LTO compression algorithms: zlib
gcc version 10.5.0 (Ubuntu 10.5.0-1ubuntu1~20.04)

>> Untar the following packages:
atf.tar.gz and FIRMWARE.tar.gz mbedtls.tar.gz

#COMPILATION:
cd atf && \
make PLAT=raptor2 all fiptool certtool LOG_LEVEL=40 B0_BUILD=1 \
	TRUSTED_BOARD_BOOT=1  MBEDTLS_DIR=../mbedtls

NOTE: To generate the fip.bin, kindly follow te below section "UPDATE THE FIRMWARE"

#UPDATE THE FIRMWARE
1. cd <SRC DIRECTORY> && tar -xpf FIRMWARE.tar.gz
2. Kindly follow the <SRC DIRECTORY>/FIRMWARE/NON_SECURE/README.md for Non Secure Image generation and flashing

