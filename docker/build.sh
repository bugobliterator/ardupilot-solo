#! /bin/bash
set -e
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
OUTPUT_PATH=$( cd "$DIR/../build" && pwd )
cd $DIR
cd ..
docker build -t ardupilot-build -v $OUTPUT_PATH:/home/dev/build .
