#!/bin/bash

count=2

number=0

SYSTEM_LOGS_DIRECTORY="system_logs"

FILE=/logdump/reboot_num

cd /logdump || exit

if [ -e "$FILE" ]; then
	read -r number < "$FILE"
else
	rm /logdump/system_logs/*
	touch $FILE;
	echo "0" > $FILE
fi

number=$(( number + 1 ))

if [ $number -gt $count ]; then
	files_to_keep=$((number - count))
	for ((i = 1; i <= files_to_keep; i++)); do
		system_logs_tar_file="/logdump/system_logs_${i}.tar.gz"
		if [[ -e "$system_logs_tar_file" ]]; then
			echo "Removing file: $system_logs_tar_file"
			rm "$system_logs_tar_file"
		fi

	done
fi

if test -d "$SYSTEM_LOGS_DIRECTORY"; then
	echo "Taking system logs"

	tar -czvf system_logs_${number}.tar.gz $SYSTEM_LOGS_DIRECTORY
	rm -rf $SYSTEM_LOGS_DIRECTORY

	echo "$number" > "$FILE"
fi

exit 0
