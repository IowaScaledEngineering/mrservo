#!/bin/bash
USBRESET='/home/ndholmes/data/projects/usbreset/usbreset'
PK2DIR='/home/ndholmes/data/projects/pk2cmd/pk2cmd'
MRSDIR='/home/ndholmes/data/projects/mrservo/src/hex'
sudo $USBRESET /dev/bus/usb$(lsusb | grep 04d8:0033 | cut -d ":" -f 1 | sed "s/Bus /\//" | sed "s/ Device /\//"); sudo $PK2DIR/pk2cmd -B$PK2DIR -PPIC10F200 -F$MRSDIR/$1 -M -Y
