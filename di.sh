#!/bin/sh

make

cd bin/osx-arm64/Release/osx-arm64/publish
chmod +x Mesen.app
./Mesen
