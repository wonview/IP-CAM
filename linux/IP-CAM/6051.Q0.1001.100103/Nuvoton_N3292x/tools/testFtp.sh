#!/bin/sh

echo "Start TFTP Test"

#typeset -i c=0
c=`expr 0`

ftpget -u ssv -p ssv 192.168.11.99 0.bin TEST1.mp3
ftpput -u ssv -p ssv 192.168.11.99 0.bin 0.bin
while true
do
	if [ $c -gt 0 ]
	then
	  echo "diff $c.bin"
		diff $c.bin 0.bin
		if [ $? -ne 0 ]
		then
		    echo "ERR:Data File Difference at round $c-th"
		    exit
		fi
		echo "OK! rm $c.bin"
		rm $c.bin
		sync
	fi
	c=`expr $c + 1`
	echo "get $c.bin"
	ftpget -u ssv -p ssv 192.168.11.99 $c.bin 0.bin
	sleep 1
	ftpget -u ssv -p ssv 192.168.11.99 $c.bin 0.bin
#	mv b.bin b$c.bin
	echo "round $c-th"
	sleep 1
#	if [ $c -gt 100000 ]
#	then
#		echo "Stop\n"
#		exit
#	fi
done
