#!/bin/sh

# Clean up shared memory file and re-create

echo "Cleaning old shm.loglib file"
rm -f /tmp/shm.loglib
echo "Creating /tmp/shm.loglib"
touch /tmp/shm.loglib
chmod 0666 /tmp/shm.loglib

# Check if logdump directory exists

if [ -d /logdump ]
then
	echo "directory /logdump exists"
else
	echo "Creating directory /logdump"
	mkdir /logdump
fi

/usr/bin/r2_l1logger > /tmp/r2_l1logger.log 2>&1
