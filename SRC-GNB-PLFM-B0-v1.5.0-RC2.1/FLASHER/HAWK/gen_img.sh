#!/bin/bash

images=(MBR.bin firmware_factory.bin ddr.bin uRamdisk.img Image raptor2-B0_f.dtb firmware_prod.bin firmware_prod.bin)
offsets=(0x0 0x80000 0x300000 0x480000 0xA00000 0xEC0000 0x1000000 0x1800000)
blocksize=0x40000
length=${#images[@]}

dd if=/dev/zero bs=1M count=32 | tr "\000" "\377" > flash_image.bin

for (( j=0; j<length; j++ ));
do
	echo Image ${images[$j]} has offset ${offsets[$j]}
	offset=$((${offsets[$j]} / ${blocksize}))

	dd if=${images[$j]} of=flash_image.bin bs=$((${blocksize})) seek=${offset} conv=notrunc
done

