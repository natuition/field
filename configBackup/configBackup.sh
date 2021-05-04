#!/bin/bash

last=$(ls | sort -V | tail -n 1)
cd ../config
if [ -s config.py ]; then
    cp config.py ../configBackup/config_`date +"%d_%m_%Y_%H:%M:%S"`.py ;
    echo "Config file are backup in config_`date +"%d_%m_%Y_%H:%M:%S"`.py."
    cd ../configBackup
    ls *.py -1t | tail -n +11 | xargs rm -f
    exit 0
else
    cp ../configBackup/$last config.py
    echo "Take backup of config file : ${last}."
    exit 1
fi