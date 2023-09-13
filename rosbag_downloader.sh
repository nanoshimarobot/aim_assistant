#!/bin/bash

ROSBAG_DIRECTORY="demo_bag"
FILE_ID_0="1u67d0xolqemTfoqINjSp_OVRCkkGR407"
FILE_NAME_0="moving_with_livox.mcap"
FILE_ID_1="14hVktB-mQPTql4I1T-oaPkTM3Kha_3FZ"
FILE_NAME_1="stopping_with_livox.mcap"

download_file(){
    FILE_ID="$1"
    FILE_NAME="$2"
    gdown "https://drive.google.com/uc?id=$FILE_ID" -O "./demo_bag/$FILE_NAME"
}

if [ -e "$ROSBAG_DIRECTORY" ]; then
    echo "rosbags already exists"
else
    mkdir "$ROSBAG_DIRECTORY"
    download_file "$FILE_ID_0" "$FILE_NAME_0"
    download_file "$FILE_ID_1" "$FILE_NAME_1"
fi