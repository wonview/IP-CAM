#! /bin/bash

echo "@@Unload hostapd..."
dir=$(pwd)

cd ../../../hostapd/hostapd2.0/
./unload_ap.sh

echo "@@Unload wireless driver..."
sleep 1
cd $dir
./unload.sh




