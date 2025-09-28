#!/bin/bash

cd output/images/
mkdir -p rfs/
mkdir -p tmp/

rm -rf rfs/*
rm -rf tmp/*
tar -xf rootfs.tar -C rfs/

if genext2fs -b 131072 -N 1024 -U -d ./rfs/ ./tmp/virtblk.img; then
  echo "Rootfs (virtblk.img) is created and stored at output/images/tmp/"
fi
if mkimage -A arm64 -O linux -T ramdisk -d tmp/virtblk.img tmp/uRamdisk-br.img; then
  echo "uRamdisk (uRamdisk-br.img) is created and stored at output/images/tmp/"
fi
