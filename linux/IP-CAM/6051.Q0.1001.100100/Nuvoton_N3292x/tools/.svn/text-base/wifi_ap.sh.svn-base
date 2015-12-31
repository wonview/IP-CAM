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
STA_WPA=$WIFI_PATH/wpa_supplicant
STA_CLI=$WIFI_PATH/wpa_cli
STA_CONF=$WIFI_PATH/wpa_supplicant.conf

HOSTAPD_CTRL=/var/run/hostapd
AP_IF=wlan0
AP_CMD=$WIFI_PATH/hostapd_ssv
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
	echo "insert ssv6051.ko"
	if [ -f ssv6051.ko ]; then
		if [ "`lsmod | grep wlan`" = "" ]; then
			insmod ssv6051.ko
		else
			echo "module is exist"
		fi
	else
		echo "cannot find file ssv6051.ko"
	fi
fi
	
echo "Sleep 1s!!"
sleep 1
echo "Up $STA_IF up!!"
ifconfig $STA_IF up
sleep 1

if [ $AP_ENABLE = "YES" ]; then
	
  ch=$AP_CHANNEL
	echo "ch=$ch"
	
	cat hostapd.conf.default		>  $AP_CONF
	if [ $ch -gt "14" ]; then
		echo "hw_mode=a"		>> $AP_CONF
	else
		echo "hw_mode=g"		>> $AP_CONF
	fi
	echo "ieee80211n=1"		>> $AP_CONF
	echo "ssid=$AP_SSID"			>> $AP_CONF
	echo "interface=$AP_IF"			>> $AP_CONF
	echo "channel=$ch"			>> $AP_CONF
	echo "ht_capab=[SHORT-GI-20]"			>> $AP_CONF

	if [ $AP_AUTH_MODE = "WPA2PSK" ] || [ $AP_AUTH_MODE = "WPAPSK" ]; then
		echo "auth_algs=3"		>> $AP_CONF
		if [ $AP_AUTH_MODE = "WPA2PSK" ]; then
			echo "wpa=2"		>> $AP_CONF
		else
			echo "wpa=1"		>> $AP_CONF
		fi
		echo "wpa_key_mgmt=WPA-PSK"	>> $AP_CONF
		if [ $AP_ENCRYPT_TYPE = "AES" ]; then
			echo "wpa_pairwise=CCMP"	>> $AP_CONF
		elif [ $AP_ENCRYPT_TYPE = "AES" ]; then
	                echo "wpa_pairwise=TKIP" 	>> $AP_CONF
		else
		fi
		echo "wpa_passphrase=$AP_AUTH_KEY"	>> $AP_CONF

	elif [ $AP_AUTH_MODE = "WEPAUTO" ]; then
		echo "auth_algs=3"			>> $AP_CONF
		echo "wep_default_key=0"		>> $AP_CONF
		echo "wep_key0=\"$AP_AUTH_KEY\""	>> $AP_CONF
		echo "wpa=0"				>> $AP_CONF
	elif [ $AP_AUTH_MODE = "OPEN" ] || [ $AP_AUTH_MODE = "SHARED" ]; then
		if [ $AP_AUTH_MODE = "SHARED" ]; then
			echo "auth_algs=2"		>> $AP_CONF
		else
			echo "auth_algs=1"		>> $AP_CONF
		fi
		if [ "$AP_ENCRYPT_TYPE" != "NONE" ]; then
			echo "wep_default_key=0"		>> $AP_CONF
			echo "wep_key0=\"$AP_AUTH_KEY\""	>> $AP_CONF
		fi
		echo "wpa=0"					>> $AP_CONF
	else
		"AP_AUTH_MODE \"$AP_AUTH_MODE\" does not support !!"
	fi
	
	$AP_CMD $AP_CONF -e $WIFI_PATH/entropy.bin &
	
	sleep 2
	echo "ifconfig $AP_IF $AP_IPADDR netmask 255.255.255.0"
	#ifconfig $STA_IF up
	ifconfig $AP_IF $AP_IPADDR netmask 255.255.255.0
	
#	if [ -n "$AP_GATEWAY" ]; then
#		echo "route add default gw $AP_GATEWAY $AP_IF"
#		route add default gw $AP_GATEWAY $AP_IF
#	fi

	SUBNET=`echo $AP_IPADDR | awk 'BEGIN {FS="."} { print $1"."$2"."$3}'`
	cat dnsmasq.conf.default						>  $DHCP_CONF
	echo "interface=$AP_IF"							>> $DHCP_CONF
	echo "dhcp-option=lan,3"						>> $DHCP_CONF
	echo "dhcp-option=lan,6"						>> $DHCP_CONF
	echo "dhcp-option=vendor:MSFT,2,1i"					>> $DHCP_CONF
	echo "dhcp-range=lan,$SUBNET.100,$SUBNET.199,255.255.255.0,14400m"	>> $DHCP_CONF
	echo "./dnsmasq --conf-file=$DHCP_CONF --pid-file=/tmp/dnsmasq.pid --user=root --group=root"
	./dnsmasq --conf-file=$DHCP_CONF --pid-file=/tmp/dnsmasq.pid --user=root --group=root &
fi

