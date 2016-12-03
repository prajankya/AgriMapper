#!/bin/bash

DEV=$(iw dev | awk '/Interface/ {print $2}')

ifconfig $DEV down
iwconfig $DEV mode managed
ifconfig $DEV up
iwconfig $DEV essid 'robotmakershub'
#iwconfig $DEV ap a0:a8:cd:a7:7d:d0

wpa_supplicant -Dnl80211 -i$DEV -c$(pwd)/wpa_supplicant.conf 
