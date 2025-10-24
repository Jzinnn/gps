#!/bin/bash

export MG_GPS_COMPILE_TIME=$(TZ='Asia/Shanghai' date "+%Y-%m-%d %H:%M:%S")

echo "##################### START SRT ####################"
source build/envsetup.sh
lunch aosp_arm64-ap2a-userdebug
echo "#################### START MAKE ####################"
make gps.default -j16
echo "####################### END $(TZ='Asia/Shanghai' date "+%Y-%m-%d %H:%M:%S") #######################"
