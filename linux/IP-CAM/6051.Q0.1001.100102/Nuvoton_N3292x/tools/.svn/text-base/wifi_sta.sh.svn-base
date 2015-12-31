#!/bin/sh
# Copyright (c) Nuvoton Technology Corp. All rights reserved.
# Description:	WiFi connection setup script for RTL USB devices
# Version:	2013-09-05	first release version, support RTL8188/RTL8188AP
#		2013-11-08	support SSID naming with space character
#		2013-11-18	remove router and DNS setting in dnsmasq.conf
#		2014-05-06	support RTL8189/RTL8189AP
#		2014-07-11	support RTL8821/RTL8821AP
#		2014-07-29	confirm has connected to AP before setting IP
# Author:	C.C. Chang	ccchang12@nuvoton.com.tw

NETCONFIG_FILE=network_config
# set ENABLE to YES/NO
STA_ENABLE=`awk '{if ($1=="STA_ENABLE") {print $2}}' $NETCONFIG_FILE`
# set WiFi Chipset to RTL8188/RTL8189/RTL8821
STA_CHIPSET=`awk '{if ($1=="STA_CHIPSET") {print $2}}' $NETCONFIG_FILE`
# set BOOTPROTO to DHCP/STATIC
STA_BOOTPROTO=`awk '{if ($1=="STA_BOOTPROTO") {print $2}}' $NETCONFIG_FILE`
# set IP address, only usful to STATIC IP
STA_IPADDR=`awk '{if ($1=="STA_IPADDR") {print $2}}' $NETCONFIG_FILE`
# set GATEWAY address, only usful to STATIC IP, a space means does not set it
STA_GATEWAY=`awk 'BEGIN { FS=" |" } {if ($1=="STA_GATEWAY") {print $2}}' $NETCONFIG_FILE`
# set NETWORK_TYPE to Infra/Adhoc, only Infra is supported now
STA_NETWORK_TYPE=`awk '{if ($1=="STA_NETWORK_TYPE") {print $2}}' $NETCONFIG_FILE`
# set Wireless AP's SSID
STA_SSID=`awk '{if ($1=="STA_SSID") { print $2 }}' $NETCONFIG_FILE`
if [ "$(echo "$STA_SSID" | cut -c1)" = '"' ]; then
	STA_SSID=`awk -F\" '{if ($1=="STA_SSID ") { print $2 }}' $NETCONFIG_FILE`
fi
# set STA_AUTH_MODE to OPEN/SHARED/WPAPSK/WPA2PSK/WPSPBC/WPSPIN/WPSREG
STA_AUTH_MODE=`awk '{if ($1=="STA_AUTH_MODE") {print $2}}' $NETCONFIG_FILE`
# set ENCRYPT_TYPE to NONE/WEP/TKIP/AES
STA_ENCRYPT_TYPE=`awk '{if ($1=="STA_ENCRYPT_TYPE") {print $2}}' $NETCONFIG_FILE`
# set authentication key to be either
# WEP-HEX example: 4142434445
# WEP-ASCII example: "ABCDE"
# TKIP/AES-ASCII: 8~63 ASCII
STA_AUTH_KEY=`awk '{if ($1=="STA_AUTH_KEY") {print $2}}' $NETCONFIG_FILE`

AP_ENABLE=`awk '{if ($1=="AP_ENABLE") {print $2}}' $NETCONFIG_FILE`
AP_CHIPSET=`awk '{if ($1=="AP_CHIPSET") {print $2}}' $NETCONFIG_FILE`
AP_IPADDR=`awk '{if ($1=="AP_IPADDR") {print $2}}' $NETCONFIG_FILE`
# set AP_GATEWAY address, a space means does not set it
#AP_GATEWAY=`awk 'BEGIN { FS=" |#" } {if ($1=="AP_GATEWAY") {print $2}}' $NETCONFIG_FILE`
AP_SSID=`awk 'BEGIN { FS=" |" } {if ($1=="AP_SSID") {print $2}}' $NETCONFIG_FILE`
if [ "$(echo "$AP_SSID" | cut -c1)" = '"' ]; then
	AP_SSID=`awk -F\" '{if ($1=="AP_SSID ") { print $2 }}' $NETCONFIG_FILE`
fi
# set AP_AUTH_MODE to OPEN/SHARED/WEPAUTO/WPAPSK/WPA2PSK
AP_AUTH_MODE=`awk '{if ($1=="AP_AUTH_MODE") {print $2}}' $NETCONFIG_FILE`
# set AP_ENCRYPT_TYPE to NONE/WEP/TKIP/AES
AP_ENCRYPT_TYPE=`awk '{if ($1=="AP_ENCRYPT_TYPE") {print $2}}' $NETCONFIG_FILE`
AP_AUTH_KEY=`awk 'BEGIN { FS=" |" } {if ($1=="AP_AUTH_KEY") {print $2}}' $NETCONFIG_FILE`
# only usful when STA_ENABLE is not YES
AP_CHANNEL=`awk '{if ($1=="AP_CHANNEL") {print $2}}' $NETCONFIG_FILE`

WIFI_PATH=$PWD

WPA_CTRL=/var/run/wpa_supplicant
STA_IF=wlan0
STA_WPA=$WIFI_PATH/wpa_supplicant_ssv
STA_CLI=$WIFI_PATH/wpa_cli_ssv
STA_CONF=$WIFI_PATH/wpa_supplicant.conf

HOSTAPD_CTRL=/var/run/hostapd
AP_IF=wlan1
AP_CMD=$WIFI_PATH/hostapd
AP_CONF=$WIFI_PATH/hostapd.conf
DHCP_CONF=$WIFI_PATH/dnsmasq.conf

if [ $STA_ENABLE != "YES" ] && [ $AP_ENABLE != "YES" ]; then
	echo "there is no STA or AP device enabled!!"
	exit
fi

ifconfig lo up
if [ $STA_CHIPSET = "RTL8188" ]; then
	echo "insert 8188eu.ko"
	if [ -f $STA_CHIPSET/8188eu.ko ]; then
		if [ "`lsmod | grep wlan`" = "" ]; then
			insmod $STA_CHIPSET/8188eu.ko
		else
			echo "module is exist"
		fi
	else
		echo "cannot find file $STA_CHIPSET/8188eu.ko"
	fi
elif [ $STA_CHIPSET = "RTL8189" ]; then
	echo "insert 8189es.ko"
	if [ -f $STA_CHIPSET/8189es.ko ]; then
		if [ "`lsmod | grep wlan`" = "" ]; then
			insmod $STA_CHIPSET/8189es.ko
		else
			echo "module is exist"
		fi
	else
		echo "cannot find file $STA_CHIPSET/8189es.ko"
	fi
elif [ $STA_CHIPSET = "RTL8821" ]; then
	echo "insert 8821as.ko"
	if [ -f $STA_CHIPSET/8821as.ko ]; then
		if [ "`lsmod | grep wlan`" = "" ]; then
			insmod $STA_CHIPSET/8821as.ko
		else
			echo "module is exist"
		fi
	else
		echo "cannot find file $STA_CHIPSET/8821as.ko"
	fi
elif [ $STA_CHIPSET = "SSV6051" ]; then
  if [ -f ssv6200-sw.bin ]; then
		echo "insert ssv6051.ko"
		if [ -f ssv6051.ko ]; then
			if [ "`lsmod | grep wlan`" = "" ]; then
				insmod ssv6051.ko
			else
				echo "module is exist"
			fi
		else
			echo "cannot find file ssv6051.ko"
			exit
		fi
  else
		echo "cannot find file ssv6200-sw.bin"
		exit
  fi  
fi
	
echo "Up $STA_IF up!!"
ifconfig $STA_IF up
#sleep 1

if [ $STA_ENABLE = "YES" ]; then
	echo "ctrl_interface=$WPA_CTRL"	>  $STA_CONF
	echo "ctrl_interface_group=0"	>> $STA_CONF
	echo "update_config=1"		>> $STA_CONF

	if [ $STA_AUTH_MODE != "WPSPBC" ] && [ $STA_AUTH_MODE != "WPSPIN" ]; then
		echo "network={"		>> $STA_CONF
		echo -e "\tssid=\"$STA_SSID\""	>> $STA_CONF

		if [ $STA_AUTH_MODE = "WPAPSK" ] || [ $STA_AUTH_MODE = "WPA2PSK" ]; then
			echo -e "\tkey_mgmt=WPA-PSK"	>> $STA_CONF
			if [ $STA_AUTH_MODE = "WPAPSK" ]; then
				echo -e "\tproto=WPA"	>> $STA_CONF
			else
				echo -e "\tproto=WPA2"	>> $STA_CONF
			fi
			if [ $STA_ENCRYPT_TYPE = "AES" ]; then
				echo -e "\tpairwise=CCMP"	>> $STA_CONF
			elif [ $STA_ENCRYPT_TYPE = "TKIP" ]; then
				echo -e "\tpairwise=TKIP"	>> $STA_CONF
			else
				"STA_ENCRYPT_TYPE \"$STA_ENCRYPT_TYPE\" does not support !!"
			fi
			echo -e "\tpsk=\"$STA_AUTH_KEY\""	>> $STA_CONF
		else
			echo -e "\tkey_mgmt=NONE"		>> $STA_CONF
			if [ $STA_ENCRYPT_TYPE = "WEP" ]; then
				echo -e "\tauth_alg=SHARED"		>> $STA_CONF
				echo -e "\twep_key0=$STA_AUTH_KEY"	>> $STA_CONF
				echo -e "\twep_tx_keyidx=0"		>> $STA_CONF
			fi
		fi

		echo "}"	>> $STA_CONF
	fi
	sync

	echo "$STA_WPA -i $STA_IF -D wext -c $STA_CONF &"
	#$STA_WPA -i $STA_IF -D wext -c $STA_CONF &
	$STA_WPA -i $STA_IF -D nl80211 -c $STA_CONF &

	sleep 2
	counter=1
	while [ 1 ]; do
		StaStatus=`$STA_CLI -i $STA_IF -p $WPA_CTRL status | awk -F= '{if ($1=="wpa_state") {print $2}}'`
		echo "Wait.. ($counter/10), Status=$StaStatus"
		if [ $StaStatus = "COMPLETED" ]; then
			break
		elif [ $StaStatus = "INACTIVE" ]; then
			echo "STA connect to AP failed!!"
			exit
		fi
		counter=`expr $counter + 1`
		if [ $counter -gt 10 ]; then
			echo "STA connect to AP timeout!!"
			exit
		fi
		sleep 1
	done

	if [ $STA_AUTH_MODE = "WPSPBC" ]; then
		echo "$STA_CLI -i $STA_IF -p $WPA_CTRL wps_pbc"
		$STA_CLI -i $STA_IF -p $WPA_CTRL wps_pbc
	elif [ $STA_AUTH_MODE = "WPSPIN" ]; then
		echo "$STA_CLI -i $STA_IF -p $WPA_CTRL wps_pin any"
		$STA_CLI -i $STA_IF -p $WPA_CTRL wps_pin any
	elif [ $STA_AUTH_MODE = "WPSREG" ]; then
		sleep 2
		#pin=`$STA_CLI -i $STA_IF -p $WPA_CTRL scan_result | awk '{if ($5=="'$STA_SSID'") {printf $1}}'`
		pin=`$STA_CLI -i $STA_IF -p $WPA_CTRL scan_result | awk '{if (NF>5) {for(i=6;i<=NF;i++) $5=$5" "$i} if ($5=="'"$STA_SSID"'") {printf $1}}'`
		#echo "pin=\"$pin\""
		echo "$STA_CLI -i $STA_IF -p $WPA_CTRL wps_reg $pin $STA_AUTH_KEY"
		$STA_CLI -i $STA_IF -p $WPA_CTRL wps_reg $pin $STA_AUTH_KEY
	fi

	if [ $STA_BOOTPROTO = "DHCP" ]; then
		echo "udhcpc -i $STA_IF"
		udhcpc -i $STA_IF -T 1
	else
		echo "ifconfig $STA_IF $STA_IPADDR netmask 255.255.255.0"
		ifconfig $STA_IF $STA_IPADDR netmask 255.255.255.0

		if [ -n "$STA_GATEWAY" ]; then
			echo "route add default gw $STA_GATEWAY $STA_IF"
			route add default gw $STA_GATEWAY $STA_IF
		fi
	fi
fi

