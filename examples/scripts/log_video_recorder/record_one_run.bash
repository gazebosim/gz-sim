#!/bin/bash


echo "==================================="
echo "Staring Log Playback Video Recorder"
echo "==================================="

if [ -z "$1" ]; then
  echo "Usage: bash ./record_one_run.bash [path_to_log_dir]"
  exit 0
fi

logDirPath=$1

if [ ! -d "$logDirPath" ]; then
  echo "Directory does not exist: $logDirPath"
  exit 0
fi

scriptDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tmpDir="$scriptDir/tmp_record"

echo "Creating tmp dir for recording: $tmpDir"

if [ -d "$tmpDir" ]; then
 rm -fr $tmpDir
fi

ln -s $logDirPath $tmpDir


echo "Starting log playback and video recording"

sdfName="log_video_recorder"
ign gazebo -v 4 $sdfName.sdf &

while [ "$end" == "" ]; do
 end=$(timeout 3 ign topic -e -t /log_video_recorder/status)
 # echo "end: $end"
done

echo "Detect video recording ended. Shutting down playback"

pgrep -f $sdfName | xargs kill -9

videoDir=$(date +%s)
echo "Moving mp4 videos to dir: $videoDir"
mkdir $videoDir
mv *.mp4 $videoDir

# remove tmp dir
if [ -d "$tmpDir" ]; then
 rm -fr $tmpDir
fi
echo "Done"

