#!/usr/bin/env bash


make
retVal=$?;
if [ $retVal -eq '0' ]
then
    echo "---------------------------------------------------------------"
    echo "                        make: success"
    echo "---------------------------------------------------------------"
else
    echo "---------------------------------------------------------------"
    echo "                     Error during make"
    echo "---------------------------------------------------------------"
fi


if [ "$1" == "bootloader" ]; then
    echo "---------------------------------------------------------------"
    echo "                         flashing bootloader"
    echo "---------------------------------------------------------------"
    sudo dfu-util -a 0 -d 0483:df11 --dfuse-address 0x08000000 -D ../Bootloader/px4flow_bl.bin -v 
fi

if [ $retVal -eq '0' ]
then
    echo "---------------------------------------------------------------"
    echo "                         flashing application"
    echo "---------------------------------------------------------------"
    make upload-usb
fi
    
#sudo dfu-util -a 0 -d 0483:df11 --dfuse-address 0x08000000 -D ../Bootloader/px4flow_bl.bin -v 
