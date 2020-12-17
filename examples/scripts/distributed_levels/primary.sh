#!/usr/bin/env bash

set -e

SCRIPTNAME="$(basename $0)"
HELP="$(cat << EOM

$SCRIPTNAME [--secondaries N] [--robots N]

  --secondaries, -s: Number of secondaries the primary expects, defaults to 2.
  --robots, -r: Number of robots, defaults to 2.
EOM
)
"

ROBOTS="2"
SECONDARIES="2"

while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -s|--secondaries)
    SECONDARIES="$2"
    shift
    shift
    ;;
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

OUTPUT_STATS_FILE="stats_distributed_r${ROBOTS}_s${SECONDARIES}.csv"

if [[ -f $OUTPUT_STATS_FILE ]]; then
  echo "$OUTPUT_STATS_FILE already exists, delete it before running a simulation with the same settings"
  exit 0
fi

empy3 -o "$TMPFILE" "$FILENAME.sdf.em" $ROBOTS

echo "-----------------------"
echo "Launching Gazebo"
echo "-----------------------"
# --levels is implied by --network-role
ign gazebo -v 4 -z 100000000 -r --network-role primary --network-secondaries $SECONDARIES $TMPFILE &

sleep 10s

echo "-----------------------"
echo "Launching ign_imgui"
echo "-----------------------"
ign_imgui -o $OUTPUT_STATS_FILE &

sleep 120s

echo "-----------------------"
echo "Closing"
echo "-----------------------"
kill -SIGINT %1 %2
wait %1 %2
