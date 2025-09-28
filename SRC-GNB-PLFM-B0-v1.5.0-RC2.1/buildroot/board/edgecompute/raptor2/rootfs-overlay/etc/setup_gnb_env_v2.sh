#!/bin/sh

if [[ -z ${BOARD_NUMBER} ]]; then echo "Please Enter The BOARD NUM: " && read BOARD_NUMBER; fi

if [[ -z ${L23_PHY_PMD} ]]; then 
	echo "L23 shall use Virtual NIC!!"
	systemctl start vpp.service; 
elif [ ${L23_PHY_PMD} == 1 ]; then
	echo "L23 shall use Physical NIC!!"
else 
	echo "Kindly run 'unset L23_PHY_PMD' for virtual NIC or 'export L23_PHY_PMD=1' for Physical NIC" && exit 1; 
fi

status_check(){
        export failure_reason=$1
        echo "Failing in ${failure_reason}.!" && exit 1
}

net_interface(){
        echo "Setting up network environment"

        # For communication between CU-DU software.
        ifconfig lo:du 127.0.0.2
        ifconfig lo:1 127.0.0.3

        # SCTP checksum default to BlueArcus
        echo N > /sys/module/sctp/parameters/no_checksums

        sleep 2

        echo "Setting up network environment Done"
}

setup_raptor(){
        echo "Setting up MGMT binary environment. Please wait....."
        rm -rf /raptor
        mkdir /raptor

        ln -s /mgmt/bin /raptor/bin
        ln -s /mgmt/etc /raptor/etc
        ln -s /mgmt/lib /raptor/lib
        ln -s /mgmt/yang /raptor/yang
        echo "Finished setting up MGMT binary environment"

        echo "Setting up L1FW binary environment. Please wait....."
        ln -s /l1fw/crss_bins /raptor/crss_bins
        ln -s /l1fw /raptor/l1fw
        echo "Finished setting up L1FW binary environment"

        echo "Setting up L23SW binaries. Please wait......"
	# Copy the contents and rename
        cp -rf /cu /tmp
        cp -rf /du /tmp

        mkdir /raptor/l23sw
        ln -s /tmp/cu /raptor/l23sw/cu
        ln -s /tmp/du /raptor/l23sw/du
        echo "Finished setting up L23SW binaries environment"
}

net_interface  || status_check "Net interface settings"
setup_raptor   || status_check "setup raptor"
