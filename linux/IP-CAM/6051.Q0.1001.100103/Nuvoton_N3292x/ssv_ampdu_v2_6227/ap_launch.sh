#! /bin/bash

BLUE='\e[1;34m'
GREEN='\e[1;32m'
CYAN='\e[1;36m'
RED='\e[1;31m'
PURPLE='\e[1;35m'
YELLOW='\e[1;33m'
# No Color
NC='\e[0m'



dir=$(pwd)
chmod 777 -R cd ../../../hostapd/hostapd2.0/
cd ../../../hostapd/hostapd2.0/
./unload_ap.sh
PID=$!
wait $PID
sleep 2


cd $dir
echo -e "${YELLOW}Load wireless driver...${NC}"
./load.sh
PID=$!
wait $PID

echo -e "${YELLOW}Detect wireless device...${NC}"
sleep 1
#get driver mac address
macAddr=$(cat sta.cfg| grep "hw_mac"|tr -d ' '|tail -c 18)
echo "device macaddr=[$macAddr]" 

#get driver name(wlan??)
#devName=$(ifconfig | grep $macAddr)
devName=$(cat /etc/udev/rules.d/70-persistent-net.rules | grep -i $macAddr)
devName=`echo $devName| cut -d '"' -f 16`

echo $devName

echo -e "${YELLOW}Config wireless AP...${NC}"
rm -rf load_dhcp.sh
rm -rf hostapd.conf
#relpace wlan@@ to real device name
cp script/template/load_dhcp.sh load_dhcp.sh
#cp script/template/hostapd.conf hostapd.conf
awk 'NF' script/template/hostapd.conf | grep -v '#' >> hostapd.conf
awk 'NF' ap.cfg | grep -v '#' >> hostapd.conf

sed -i "s/wlan@@/$devName/" load_dhcp.sh
sed -i "s/wlan@@/$devName/" hostapd.conf

chmod 777 load_dhcp.sh

#move to right position
mv load_dhcp.sh ../../../hostapd/hostapd2.0/
mv hostapd.conf ../../../hostapd/hostapd2.0/hostapd/


dhcp_config_file="/etc/default/isc-dhcp-server"
dhcp_config=$(grep "$devName" $dhcp_config_file)
if [ "$dhcp_config" == "" ]; then
	echo -en "${YELLOW}Config $dhcp_config_file.....${NC}"
	
	rm -rf tmp
	sed '/INTERFACE/d' /etc/default/isc-dhcp-server >>tmp
	echo "INTERFACES=\"$devName\"" >>tmp	
	rm -rf $dhcp_config_file	
	mv tmp /etc/default/isc-dhcp-server
	
	echo -e "${YELLOW}OK${NC}"
fi
	
	
dir=$(pwd)
echo -e "${YELLOW}Wireless Done. ${NC}"
cd ../../../hostapd/hostapd2.0/
./load_ap.sh




