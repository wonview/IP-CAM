#/bin/bash

killall hostapd
ps aux|grep hostapd

./unload_dhcp.sh