#/bin/sh

ulimit -c unlimited
#dpdk-hugepages.py -p 32M --setup 128M
#echo "/coredump/core.%e" > /proc/sys/kernel/core_pattern
export SYSREPO_REPOSITORY_PATH="/config/sysrepo"
export SYSREPO_SHM_PREFIX="edgeq_"
