#!/bin/bash

PACKAGE_NAME="DomePro_X2.pkg"
BUNDLE_NAME="org.rti-zone.DomeProX2"

if [ ! -z "$app_id_signature" ]; then
    codesign -f -s "$app_id_signature" --verbose ../build/Release/libDomePro.dylib
fi

mkdir -p ROOT/tmp/DomePro_X2/
cp "../domepro.ui" ROOT/tmp/DomePro_X2/
cp "../domeprodiag.ui" ROOT/tmp/DomePro_X2/
cp "../domeshutter.ui" ROOT/tmp/DomePro_X2/
cp "../dometimeouts.ui" ROOT/tmp/DomePro_X2/
cp "../Astrometric.png" ROOT/tmp/DomePro_X2/
cp "../domelist DomePro.txt" ROOT/tmp/DomePro_X2/
cp "../build/Release/libDomePro.dylib" ROOT/tmp/DomePro_X2/

if [ ! -z "$installer_signature" ]; then
	# signed package using env variable installer_signature
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --sign "$installer_signature" --scripts Scripts --version 1.0 $PACKAGE_NAME
	pkgutil --check-signature ./${PACKAGE_NAME}
else
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --scripts Scripts --version 1.0 $PACKAGE_NAME
fi

rm -rf ROOT
