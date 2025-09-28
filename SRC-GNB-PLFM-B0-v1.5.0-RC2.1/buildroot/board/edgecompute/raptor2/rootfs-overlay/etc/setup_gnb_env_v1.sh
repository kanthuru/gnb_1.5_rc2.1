#!/bin/sh
#

if [[ -z ${ETHERNET_PORT} ]]; then export ETHERNET_PORT="eth0" && echo "Using default eth0 interface"; fi
if [[ -z ${BOARD_NUMBER} ]]; then echo "Please Enter The BOARD NUM: " && read BOARD_NUMBER; fi

status_check(){
        export failure_reason=$1
        echo "Failing in ${failure_reason}.!" && exit 1
}

net_interface(){
        echo "Setting up network environment"

        # Generate the IP based on board-num
        IP_OFFSET=$BOARD_NUMBER
        IP_ETH0="192.168.3."$IP_OFFSET
        IP_OFFSET=50
        IP_OFFSET=$((IP_OFFSET + $BOARD_NUMBER))
        IP_ETH01="192.168.3."$IP_OFFSET

        # MTU set to 9000
        ifconfig $ETHERNET_PORT mtu 9000

        # Board IP. Verify the eth interface.
        ifconfig $ETHERNET_PORT $IP_ETH0 netmask 255.255.255.0 up

        # Verify the nexter interface IP.
        ifconfig "$ETHERNET_PORT":1 $IP_ETH01 netmask 255.255.255.0 up

        # For communication between CU-DU software.
        ifconfig lo:du 127.0.0.2
        ifconfig lo:1 127.0.0.3

        # SCTP checksum default to BlueArcus
        echo N > /sys/module/sctp/parameters/no_checksums

        sleep 2

        echo "Setting up network environment Done"
}

set_l23_config(){
        echo "Changing CU NgC & NgU address"

        cp /cu/config/oam_3gpp_cu_sa_1du_1cell.json /tmp/config_step1.json
        cat /tmp/config_step1.json | sed -re '/EP_NgC/{N;N;N;N;s/(localIpAd.*)192.168.3..*\"/\1'$IP_ETH0'\"\2/;}' > /tmp/config_step2.json
        cat /tmp/config_step2.json | sed -re '/EP_NgU/{N;N;N;N;s/(localIpAd.*)192.168.3..*\"/\1'$IP_ETH0'\"\2/;}' > /tmp/config_step3.json
        cat /tmp/config_step3.json | sed 's/\(\"gNBId\": \)\(\".*.\"\)/\1\"'$BOARD_NUMBER'\"/' > /tmp/config_final.json

        cp /cu/config/oam_3gpp_cu_sa_1du_1cell.json /cu/config/oam_3gpp_cu_sa_1du_1cell.json.orig
        mv /tmp/config_final.json /cu/config/oam_3gpp_cu_sa_1du_1cell.json
        echo "NgC & NgU Addresses and gNBId changed in CU config".

        CELL_ID=$(( $BOARD_NUMBER << 4 ))
        CELL_ID=$(( $CELL_ID + 1 ))
        NR_CELL_ID=`printf "%09x" $CELL_ID`

        cp /du/config/oam_3gpp_cell_cfg_mu1_1cell.json /tmp/config_du_step1.json
        cat /tmp/config_du_step1.json | sed 's/\(\"nrCellId\": \)\(\".*.\"\)/\1\"'$NR_CELL_ID'\"/' > /tmp/config_du_step2.json
        cat /tmp/config_du_step2.json | sed 's/\(\"gNBId\": \)\(\".*.\"\)/\1\"'$BOARD_NUMBER'\"/' > /tmp/config_du_final.json

        cp /du/config/oam_3gpp_cell_cfg_mu1_1cell.json /du/config/oam_3gpp_cell_cfg_mu1_1cell.json.orig
        mv /tmp/config_du_final.json /du/config/oam_3gpp_cell_cfg_mu1_1cell.json

        echo "gNBId & CellId changed in DU config"
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
set_l23_config || status_check "Update L23 config"
setup_raptor   || status_check "setup raptor"

