#!/bin/bash

#Set the core pattern
echo "/coredump/core.%e" > /proc/sys/kernel/core_pattern
#Network settings
echo 4096 > /proc/sys/net/ipv4/ipfrag_max_dist
echo 16 >  /proc/sys/net/core/rps_sock_flow_entries
echo 30000 > /sys/class/net/eth0/queues/rx-0/rps_cpus
echo 16 > /sys/class/net/eth0/queues/rx-0/rps_flow_cnt
cat /proc/sys/net/core/rps_sock_flow_entries
