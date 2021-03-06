#!/bin/sh
# ------------------------------
nmcli nm wifi off
sudo rfkill unblock wlan

./unload_ap.sh
 
ifconfig $1 192.168.33.1 netmask 255.255.255.0
cp dhcpd.conf /tmp/dhcpd_$1.conf
dhcpd -cf /tmp/dhcpd_$1.conf -pf /var/run/dhcp-server/dhcpd.pid $1
bash -c "echo 1 >/proc/sys/net/ipv4/ip_forward" 
iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
cat hostapd.conf.AES.template | sed -s s/HOSTAPD_IF/${1}/g  > hostapd.conf
hostapd hostapd.conf

nmcli nm wifi on

echo AP mode stopped
# ------------------------------------
