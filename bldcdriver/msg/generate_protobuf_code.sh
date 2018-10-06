#!/bin/sh
if [ -z $NANOPB ]; then
	echo "Please point the environment variable NANOPB to the nanopb install dir such that \$NANOPB/generator-bin/protoc is the compiler that comes with nanopb"
	exit 1
fi
$NANOPB/generator-bin/protoc --nanopb_out=. motor.proto
protoc --python_out=../pymsg motor.proto
