#/bin/bash

KVERSION="`uname -r`"
kern_mod=/lib/modules/$KVERSION/kernel/drivers/net/wireless/ssv6200/ssvdevicetype.ko
type_str=`lsmod | grep "ssvdevicetype"`
cfg_cmds=(`cat sta.cfg  | grep '^[a-zA-Z0-9]' | sed 's/ //g'`)
#echo ${#cfg_cmds[*]}
#echo ${!cfg_cmds[*]}
#echo ${cfg_cmds[1]}

if [ "$type_str" != "" ]; then
    #rmmod ssv6200_sdio
    #rmmod ssv6200s_core
    #rmmod ssv6200_hci
    rmmod ssvdevicetype
fi


if [ -f $kern_mod ]; then
    insmod $kern_mod
    ./cli cfg reset
    for cmd in ${cfg_cmds[*]}
    do
	./cli cfg `echo $cmd | sed 's/=/ = /g'`
    done
fi

