#!/bin/bash

mkdir -p ROOT/tmp/RigelDome_X2/
cp "../RigelDome.ui" ROOT/tmp/RigelDome_X2/
cp "../Pulsar.png" ROOT/tmp/RigelDome_X2/
cp "../domelist RigelDome.txt" ROOT/tmp/RigelDome_X2/
cp "../build/Release/libRigelDome.dylib" ROOT/tmp/RigelDome_X2/

if [ ! -z "$installer_signature" ]; then
# signed package using env variable installer_signature
pkgbuild --root ROOT --identifier org.rti-zone.RigelDome_X2 --sign "$installer_signature" --scripts Scripts --version 1.0 RigelDome_X2.pkg
pkgutil --check-signature ./RigelDome_X2.pkg
else
pkgbuild --root ROOT --identifier org.rti-zone.RigelDome_X2 --scripts Scripts --version 1.0 RigelDome_X2.pkg
fi

rm -rf ROOT
