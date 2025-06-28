#!/bin/sh

make

cd bin/osx-arm64/Debug/osx-arm64/publish
chmod +x Mesen.app
./Mesen
