#!/bin/bash

export VPP_INTERFACES=("eth0")	# List of interfaces to be used for VPP e.g. ("eth0" "eth1")
export UPF_INTERFACE="eth0"	# Interface to be used for UPF (active interface)
export ENABLE_LCP="true"	# Flag to enable / disable LCP mode
export ENABLE_L2_FWD="false"	# Flag to enable / disable L2 forwarding mode
export ENABLE_GTPU_PLUGIN="false"	# Flag to enable / disable GTPU plugin
#export UPF_VLAN_ID="500"	# VLAN ID that is to be used for UPF interface

source /etc/vpp/init.sh

/usr/bin/vpp -c /etc/vpp/startup.conf

source /etc/vpp/post_setup.sh
