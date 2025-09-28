#!/bin/sh

. ./fw_functions.sh

usage () {
	echo "  Usage: $0 command [options]"
	echo "  command: firmware_B0/firmware_B0_secure"
	echo "  options: -spl_ver <SPL_VERSION> -cust_csk_id <active csk id> -cust_sha <value> -edgeq_csk_id <value> -edgeq_sha <value>"
	echo "  -spl_ver The version of SPL"
	echo "  -cust_csk_id and -edgeq_csk_id has values from 0-31"
	echo "  -cust_sha and -edgeq_sha are the hashing algorithm to be used: {0:sha256; 1:sha512}"
	echo -n  "\n"
	echo "  Example: $0 firmware_B0"
	echo -n  "\n"
}

unset build_cmd
unset clean_cmd

if [ $# -lt  1 ]
then
	echo "!!!Invalid args !!!!!!"
	usage
	exit 1
fi

while [ "$1" != "" ];
do
	case $1 in

		firmware_B0 | firmware_B0_secure )
			build_cmd=$1
			;;

		*_clean )
			clean_cmd=$1
			;;

		-spl_ver ) shift
			SPL_VERSION=$1
			;;

		-edgeq_sha ) shift
			EDGEQ_SHA_TYPE=$1
			if [ $EDGEQ_SHA_TYPE -gt 1 ]
			then
				echo "invalid Edgeq sha value"
				exit 2
			fi
			;;

		-cust_sha ) shift
			CUST_SHA_TYPE=$1
			if [ $CUST_SHA_TYPE -gt 1 ]
			then
				echo "invalid Customer sha value"
				exit 2
			fi
		;;

		-edgeq_csk_id ) shift
			EDGEQ_CSK_ID=$1
			;;

		-cust_csk_id ) shift
			CUST_CSK_ID=$1
		;;

		-h | --help )
			usage
			exit
			;;

		* )
			echo "illegal option: $1"
			usage
			exit 1
	esac
	shift
done

SPL_VERSION=${SPL_VERSION:-0}
EDGEQ_SHA_TYPE=${EDGEQ_SHA_TYPE:-0}
EDGEQ_CSK_ID=${EDGEQ_CSK_ID:-0}
CUST_SHA_TYPE=${CUST_SHA_TYPE:-0}
fw_option_flags -edgeq_sha ${EDGEQ_SHA_TYPE} -edgeq_csk_id ${EDGEQ_CSK_ID} -cust_sha ${CUST_SHA_TYPE} -cust_csk_id ${CUST_CSK_ID}

if [ -n "${build_cmd}" ]
then
	#raptor_build ${build_cmd}
	raptor_img_generation ${build_cmd}
else
	make ${clean_cmd}
	if [ -e ${FIRMWARE_FILE} ]
	then
		rm -f ${FIRMWARE_FILE}
	fi
fi

