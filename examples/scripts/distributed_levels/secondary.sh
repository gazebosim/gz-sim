#!/usr/bin/env bash

set -e

SCRIPTNAME="$(basename $0)"
HELP="$(cat << EOM

$SCRIPTNAME [--robots N]

  --robots, -r: Number of robots, defaults to 2.
EOM
)
"

ROBOTS="2"

while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -r|--robots)
    ROBOTS="$2"
    shift
    shift
    ;;
    *)
    echo "$HELP"
    exit 0
    ;;
esac
done

FILENAME="${SCRIPTNAME%.*}"
TMPFILE=$(mktemp /tmp/primary.XXXXXX.sdf)
function cleanup {
  rm $TMPFILE
}
trap cleanup EXIT

empy3 -o "$TMPFILE" "$FILENAME.sdf.em" $ROBOTS

# --levels is implied by --network-role
ign gazebo -s -v 4 -z 100000000 --network-role secondary $TMPFILE
