#!/bin/bash

VERSION=0.1.0
PACKAGE_NAME=omzlo-samd-$VERSION
ZIPFILE=boards/$PACKAGE_NAME.zip
JSONFILE=package_omzlo.com_index.json
TEMPLATE=package_omzlo.com_index.json.in
LIBRARY=../lib
FILES="$LIBRARY/spi.h $LIBRARY/spi.c $LIBRARY/nocan_ll.h $LIBRARY/nocan_ll.c $LIBRARY/nocan.cpp $LIBRARY/nocan.h"
ARDUINO_H=src/cores/arduino/Arduino.h
LD_SCRIPT=src/variants/omzlo_canzero/linker_scripts/gcc/flash_with_bootloader.ld

red=`tput setaf 1`
green=`tput setaf 2`
reset=`tput sgr0`

read -p "Confirm building Omlzo arduino library version ${green}$VERSION${reset} [y/n]? " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    exit 1
fi

echo -n "Checking source files: "
for f in $TEMPLATE $LIBRARY $FILES $ARDUINO_H $LD_SCRIPT; do
    if [ ! -e "$f" ]; then
        echo "${red}Fail${reset}"
        echo "$f does not exist"
        exit 2
    fi
done
echo "${green}OK${reset}"

echo -n "Copying C/C++ files: "
rm -f src/libraries/NOCAN/*
for f in $FILES; do
    cp "$f" src/libraries/NOCAN/
done
echo "${green}OK${reset}"

echo -n "Removing USB from core files: "
if [ -e "src/cores/arduino/USB/" ]; then
   rm -rf src/cores/arduino/USB
   sed -i '.bak' -e "s/#include \"USB/\/\/#include \"USB/" $ARDUINO_H
   echo "${green}OK${reset}"
else
   echo "${green}Not needed${reset}"
fi 

echo -n "Removing old zip file: "
rm -rf $ZIPFILE
echo "${green}OK${reset}"

echo -n "Creating temporary directory '$VERSION' and archive '$ZIPFILE': "
cp -R src $VERSION
zip -q -r $ZIPFILE $VERSION
echo "${green}OK${reset}"

echo -n "Building $JSONFILE: "
CHECKSUM=`shasum -a 256 $ZIPFILE | cut -d ' ' -f 1`
SIZE=`stat -f "%z" $ZIPFILE`

sed -e "s/\${version}/$VERSION/g" -e "s/\${size}/$SIZE/g" -e "s/\${checksum}/$CHECKSUM/g" -e "s/\${package_name}/$PACKAGE_NAME/g" $TEMPLATE >  $JSONFILE
echo "${green}OK${reset}"

echo -n "Cleaning up, removing temporary directory '$VERSION': "
rm -rf $VERSION/
echo "${green}OK${reset}"

echo "*** Created package version $VERSION with checksum $CHECKSUM ***${reset}"

echo -n "Testing if host is up: "
if ping -q -c 1 paris &> /dev/null; then
    echo "${green}OK${reset}"
else
    echo "${red}Fail${reset}"
    echo "Host unreachable"
    exit 2
fi

echo
echo "SUCCESS! Files are now ready to be uploaded."
echo "e.g. scp $JSONFILE $ZIPFILE foobar@example.com:downloads/"
