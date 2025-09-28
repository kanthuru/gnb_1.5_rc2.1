#!/bin/sh

KAK_CERT=${SECURE_KEYS_PATH}/kak.crt
KAK_PRIVATE_KEY=${SECURE_KEYS_PATH}/kak_private_key.pem
CSK_CERT=${SECURE_KEYS_PATH}/csk.crt

SPL_META_DATA_SIZE=256
SPL_HDR_SIZE=292
SPL_SIGN_SIZE=256
DDR_HDR_SIZE=24
DDR_SIGN_SIZE=256
BSS_RT_HDR_SIZE=24
BSS_RT_SIGN_SIZE=256
SECURE_HDR_SIZE=3072

SECURE_HDR_FILE=secure_hdr.bin
CUST_SECURE_HDR_FILE=cust_secure_hdr.bin
SPL_HDR_FILE=spl_hdr.bin
DDR_HDR_FILE=ddr_hdr.bin
BSS_RT_HDR_FILE=bss_rt_hdr.bin
FIRMWARE_FILE=firmware.bin
SPL_PAD_FILE=spl_pad.bin
FIP_PAD_FILE=fip_pad.bin
FIP_PAD_HDR_FILE=fip_pad_hdr.bin
FIP_PAD_DATA_FILE=fip_pad_data.bin

#default options
SPL_LOAD_ADDR=0x1C80A000
SPL_VERSION=0
IMG_LOAD_ADDR=0x18000000	#txu lmem
BSS_RT_LOAD_ADDR=0x6FE1C000                #OCM
DDR_VERSION=0
BSS_RT_VERSION=0
DDR_FLASH_OFFSET=0x300000
BSS_RT_FLASH_OFFSET=0x280000
RESERVED=0x0
SPL_IMAGE=spl_image.bin
DDR_IMAGE=ddr_image.bin
BSS_RUNTIME_IMAGE=bss_rt_image.bin
FIP_MAX_SIZE=1470464

#image ids
DDR_IMAGE_ID=0
BSS_RT_IMAGE_ID=1

HASH_ALG="sha256"


error_check () {
	ret=$?
	if [ $ret -ne 0 ]
	then
		echo "!!!!error:$ret"
		exit
	fi
}

customer_secure_hdr_generation () {
	if [ -e ${KAK_CERT} -a -e ${KAK_PRIVATE_KEY} ]
	then
		#extract public key from certificate
		openssl x509 -in ${KAK_CERT} -pubkey  -noout > kak_pub_key.pem
		#convert public key from PEM to DER format
		openssl rsa -pubin -inform PEM -in kak_pub_key.pem -outform DER -out kak_pub_key.der

		if [ -n ${CSK_CERT} ]
		then
			#extract public key from certificate
			openssl x509 -in ${CSK_CERT} -pubkey -noout > csk_pub_key.pem
			#convert public key from PEM to DER format
			openssl rsa -pubin -inform PEM -in csk_pub_key.pem -outform DER -out csk_pub_key.der

			printf "%02x" ${CUST_CSK_ID} | xxd -p -r > csk_id
			cat csk_id csk_pub_key.der > csk.key
			rm -f csk_id csk_pub_key.pem csk_pub_key.der
			#sign csk_id,csk public key with KAK private key
			openssl dgst -${CUST_HASH_ALG_OPENSSL} -sign ${KAK_PRIVATE_KEY} -out csk.sign csk.key
			rm -f csk_id

			dd if=/dev/zero of=rsv.bin bs=2048 count=1 > /dev/null 2>&1
			#pad header to 3K
			dd if=/dev/zero of=pad.bin bs=179 count=1 > /dev/null 2>&1
			#append all files to generate secure header
			cat kak_pub_key.der csk.key rsv.bin csk.sign pad.bin > ${CUST_SECURE_HDR_FILE}
			# remove intermediate files
			rm -f csk.key rsv.bin csk.sign pad.bin

			echo "CUST_HASH_ALG_OPENSSL ${CUST_HASH_ALG_OPENSSL}"
			echo "CUST_CSK_ID ${CUST_CSK_ID}"
			echo "customer secure header is generated"
		else
			echo "!!!!!ERROR: CSK_CERT is not exported:${CSK_CERT} !!!!!"
			exit 2
		fi

	else
		echo "!!!!ERROR: KAK_CERT & KAK_PRIVATE_KEY is not exported !!!!!!"
		exit 2
	fi
}

spl_hdr_generation () {
	if [ -z ${SPL_BIN} ]
	then
		echo "!!!Error, Please export SPL_BIN"
		exit 2
	fi

	version=${SPL_VERSION}
	sha_type=${EDGEQ_SHA_TYPE}
	img_size=`stat -c%s $SPL_BIN`
	img_offset=$SPL_HDR_SIZE
	sign_offset=${img_size}
	spl_size_with_hdr=`expr ${SPL_HDR_SIZE} + ${img_size}`
	spl_total_img_size=`expr ${spl_size_with_hdr} + ${SPL_SIGN_SIZE}`
	if [ ${SPL_ELF} ]
	then
		if [ ${RISCV_CROSS_COMPILE_PREFIX} ]
		then
			img_load_addr=0x`${RISCV_CROSS_COMPILE_PREFIX}-nm ${SPL_ELF} | grep -w _start |awk '{print $1}'`
		else
			echo "!!!Error, Please export RISCV_CROSS_COMPILE_PREFIX"
			exit 2
		fi
	else
		img_load_addr=${SPL_LOAD_ADDR}
	fi
	# assuming load adress, entry address are same
	img_entry_addr=${img_load_addr}
	#fip_offset_addr=`expr ${SECURE_HDR_SIZE} + ${spl_total_img_size}`
	#fip.bin is at 1MB location
	fip_offset_addr=`expr 1024 \* 1024`
	spl_size_with_secure=`expr ${spl_total_img_size} + ${SECURE_HDR_SIZE}`
	spl_pad_size=`expr ${fip_offset_addr} - ${spl_size_with_secure}`
	dd if=/dev/zero of=${SPL_PAD_FILE} bs=${spl_pad_size} count=1 > /dev/null 2>&1
	echo "spl_pad_size  :${spl_pad_size}"
	fip_size=${FIP_MAX_SIZE}
	error_check

	echo "------------SPL HEADER-----------------"
	echo "	ver         :${version}"
	echo "	sha_type    :${sha_type}"
	echo "	img_size    :${img_size}"
	echo "	img_offset  :${img_offset}"
	echo "	sign_offset :${sign_offset}"
	echo "	load_addr   :${img_load_addr}"
	echo "	fip_offset  :${fip_offset_addr}"
	echo "	fip_size    :${fip_size}"
	echo "---------------------------------------"
	#generate hdr file
	printf "%08x" $version | xxd -p -r  >spl_hdr_tmp.bin
	printf "%08x" $sha_type | xxd -p -r >>spl_hdr_tmp.bin
	#fill meta data with zero
	meta_size=0
	while [ ${meta_size} -lt ${SPL_META_DATA_SIZE} ]
	do
		printf "%08x" 0 | xxd -p -r >>spl_hdr_tmp.bin
		meta_size=`expr $meta_size + 4`
	done
	printf "%08x" ${sign_offset} | xxd -p -r >>spl_hdr_tmp.bin
	printf "%08x" ${img_size} | xxd -p -r >>spl_hdr_tmp.bin
	printf "%08x" ${img_offset} | xxd -p -r >>spl_hdr_tmp.bin
	printf "%08x" ${img_load_addr} | xxd -p -r >>spl_hdr_tmp.bin
	printf "%08x" ${img_entry_addr} | xxd -p -r >>spl_hdr_tmp.bin
	printf "%08x" ${fip_offset_addr} | xxd -p -r >>spl_hdr_tmp.bin
	printf "%08x" ${fip_size} | xxd -p -r >>spl_hdr_tmp.bin
	#convert the file into little endian format
	echo '"%08x"' >fmt
	hexdump -f fmt -v spl_hdr_tmp.bin | xxd -p -r  >${SPL_HDR_FILE}
	rm -f spl_hdr_tmp.bin fmt
	echo "SPL header is generated"
}

ddr_hdr_generation () {
	if [ -z ${DDR_BIN} ]
	then
		echo "!!!Error, Please export DDR_BIN"
		exit 2
	fi

	version=${DDR_VERSION}
	fw_image_id=${DDR_IMAGE_ID}
	img_size=`stat -c%s $DDR_BIN`
	img_offset=$DDR_HDR_SIZE
	sign_offset=${img_size}
	img_load_addr=${IMG_LOAD_ADDR}
	error_check

	echo "------------DDR HEADER-----------------"
	echo "	ver         :${version}"
	echo "	fw_image_id :${fw_image_id}"
	echo "	img_size    :${img_size}"
	echo "	img_offset  :${img_offset}"
	echo "	sign_offset :${sign_offset}"
	echo "	img_load_addr   :${img_load_addr}"
	echo "---------------------------------------"
	#generate hdr file
	printf "%08x" $version | xxd -p -r  >ddr_hdr_tmp.bin
	printf "%08x" $fw_image | xxd -p -r >>ddr_hdr_tmp.bin
	printf "%08x" ${img_size} | xxd -p -r >>ddr_hdr_tmp.bin
	printf "%08x" ${img_offset} | xxd -p -r >>ddr_hdr_tmp.bin
	printf "%08x" ${img_load_addr} | xxd -p -r >>ddr_hdr_tmp.bin
	printf "%08x" ${sign_offset} | xxd -p -r >>ddr_hdr_tmp.bin
	#convert the file into little endian format
	echo '"%08x"' >fmt
	hexdump -f fmt -v ddr_hdr_tmp.bin | xxd -p -r  >${DDR_HDR_FILE}
	cat ${DDR_HDR_FILE} $DDR_BIN  > ddr_image.bin
	rm -f ddr_hdr_tmp.bin fmt
	echo "DDR header is generated"
}

bss_rt_hdr_generation () {
	if [ -z ${BSS_RUNTIME_BIN} ]
	then
		echo "!!!Error, Please export BSS_RUNTIME_BIN"
		exit 2
	fi
	echo "started generating bss header"
	version=${BSS_RT_VERSION}
	fw_image_id=${BSS_RT_IMAGE_ID}
	img_size=`stat -c%s $BSS_RUNTIME_BIN`
	img_offset=$BSS_RT_HDR_SIZE
	sign_offset=${img_size}
	img_load_addr=${BSS_RT_LOAD_ADDR}
	error_check

	echo "------------BSS_RT HEADER-----------------"
	echo "  ver         :${version}"
	echo "  fw_image_id :${fw_image_id}"
	echo "  img_size    :${img_size}"
	echo "  img_offset  :${img_offset}"
	echo "  sign_offset :${sign_offset}"
	echo "  img_load_addr   :${img_load_addr}"
	echo "---------------------------------------"
	#generate hdr file
	printf "%08x" $version | xxd -p -r  >bss_rt_hdr_tmp.bin
	printf "%08x" $fw_image | xxd -p -r >>bss_rt_hdr_tmp.bin
	printf "%08x" ${img_size} | xxd -p -r >>bss_rt_hdr_tmp.bin
	printf "%08x" ${img_offset} | xxd -p -r >>bss_rt_hdr_tmp.bin
	printf "%08x" ${img_load_addr} | xxd -p -r >>bss_rt_hdr_tmp.bin
	printf "%08x" ${sign_offset} | xxd -p -r >>bss_rt_hdr_tmp.bin
	#convert the file into little endian format
	echo '"%08x"' >fmt
	hexdump -f fmt -v bss_rt_hdr_tmp.bin | xxd -p -r  >${BSS_RT_HDR_FILE}
	cat ${BSS_RT_HDR_FILE} $BSS_RUNTIME_BIN  > bss_rt_image.bin
	rm -f bss_rt_hdr_tmp.bin fmt
	echo "BSS_RT header is generated"
}

nonsecure_img_generation () {
	spl_hdr_generation
	bss_rt_hdr_generation
	ddr_hdr_generation
	#fill secure header with zeros
	dd if=/dev/zero of=${SECURE_HDR_FILE} bs=${SECURE_HDR_SIZE} count=1 > /dev/null 2>&1
	dd if=/dev/zero of=${CUST_SECURE_HDR_FILE} bs=${SECURE_HDR_SIZE} count=1 > /dev/null 2>&1
	dd if=/dev/zero of=${SPL_SIGN_FILE} bs=${SPL_SIGN_SIZE} count=1 > /dev/null 2>&1
	
	fip_size=`stat -c%s $FIP_BIN`
	fip_pad_size=`expr ${FIP_MAX_SIZE} - ${fip_size}`
	dd if=/dev/zero of=${FIP_PAD_FILE} bs=${fip_pad_size} count=1 > /dev/null 2>&1
	echo "fip_size: ${fip_size} fip_pad_size ${fip_pad_size}"

        # Generate a 4-byte DEADBEEF header
	header="\xde\xad\xbe\xef"
	# Generate a 64-byte file with the header repeated
	printf "$header%.0s" {1..16} > ${FIP_PAD_HDR_FILE}
	# Generate a fip_pad.bin
	dd if=/dev/zero of=${FIP_PAD_FILE} bs=${fip_pad_size} count=1 > /dev/null 2>&1
	# Calculate the fip padding date size by substracting 64 byte header from fip pad size
	fip_pad_data_size=`expr ${fip_pad_size} - 64`
	dd if=/dev/zero of=${FIP_PAD_DATA_FILE} bs=${fip_pad_data_size} count=1 > /dev/null 2>&1
	# Generate consolidated fip_pad.bin
	cat ${FIP_PAD_HDR_FILE} ${FIP_PAD_DATA_FILE} > ${FIP_PAD_FILE}

	cat ${SECURE_HDR_FILE} ${SPL_HDR_FILE} ${SPL_BIN} ${SPL_SIGN_FILE} ${SPL_PAD_FILE} ${FIP_BIN} ${FIP_PAD_FILE} ${CUST_SECURE_HDR_FILE} ${BSS_RUNTIME_IMAGE} > ${FIRMWARE_FILE}
	error_check
	rm -f ${SECURE_HDR_FILE} ${CUST_SECURE_HDR_FILE} ${SPL_HDR_FILE} ${SPL_SIGN_FILE} ${SPL_PAD_FILE}
	echo "Generated ${FIRMWARE_FILE} successfully"
}

secure_B0_img_generation_customer () {
	customer_secure_hdr_generation

	fip_size=`stat -c%s $FIP_BIN`
	fip_pad_size=`expr ${FIP_MAX_SIZE} - ${fip_size}`
	dd if=/dev/zero of=${FIP_PAD_FILE} bs=${fip_pad_size} count=1 > /dev/null 2>&1
	echo "fip_size: ${fip_size} fip_pad_size ${fip_pad_size}"
	
	# Generate a 4-byte DEADBEEF header
	header="\xde\xad\xbe\xef"
	# Generate a 64-byte file with the header repeated
	printf "$header%.0s" {1..16} > ${FIP_PAD_HDR_FILE}
	# Generate a fip_pad.bin
	dd if=/dev/zero of=${FIP_PAD_FILE} bs=${fip_pad_size} count=1 > /dev/null 2>&1
	# Calculate the fip padding date size by substracting 64 byte header from fip pad size
	fip_pad_data_size=`expr ${fip_pad_size} - 64`
	dd if=/dev/zero of=${FIP_PAD_DATA_FILE} bs=${fip_pad_data_size} count=1 > /dev/null 2>&1
	# Generate consolidated fip_pad.bin
	cat ${FIP_PAD_HDR_FILE} ${FIP_PAD_DATA_FILE} > ${FIP_PAD_FILE}

	cat ${SECURE_HDR_FILE} ${SPL_IMAGE} ${FIP_BIN} ${FIP_PAD_FILE} ${CUST_SECURE_HDR_FILE} ${BSS_RUNTIME_IMAGE} > ${FIRMWARE_FILE}
	error_check
	echo "Generated ${FIRMWARE_FILE} successfully"
}

raptor_img_generation () {
	if [ -e $FIP_BIN ]
	then
		if [ $1 = "firmware_B0" ]
		then
			nonsecure_img_generation
		elif [ $1 = "firmware_B0_secure" ]
		then
			secure_B0_img_generation_customer
		else
			echo "!!Error:unknown build command"
			exit 2
		fi
	else
		echo "!!Error:bin files does not exist"
		exit 2
	fi
}

# usage example: fw_option_flags -spl_ver 5 -sha 1 csk_id 5
fw_option_flags () {
	if [ $# -lt  1 ]
	then
		exit 0
	fi

	while [ "$1" != "" ];
	do
		case $1 in

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

		esac
		shift $#
	done

	SPL_VERSION=${SPL_VERSION:-0}
	EDGEQ_SHA_TYPE=${EDGEQ_SHA_TYPE:-0}

	# These are just different strings, no difference in functionality
	if [ ${EDGEQ_SHA_TYPE} -eq 0 ]
	then
		EDGEQ_HASH_ALG_OPENSSL="sha256"
	else
		EDGEQ_HASH_ALG_OPENSSL="sha512"
	fi

	CUST_SHA_TYPE=${CUST_SHA_TYPE:-0}

	if [ ${CUST_SHA_TYPE} -eq 0 ]
	then
		CUST_HASH_ALG_OPENSSL="sha256"
	else
		CUST_HASH_ALG_OPENSSL="sha512"
	fi
}
