#!/bin/bash

HOMEDIR=~
DATADIR="${HOMEDIR}/odometry_raw"
EXECUTABLE=../build/dataset_vo_sun
WINDOW=2

# % 0: 2011_10_03_drive_0027 000000 004540
# % 01: 2011_10_03_drive_0042 000000 001100
# % 02: 2011_10_03_drive_0034 000000 004660
# % 04: 2011_09_30_drive_0016 000000 000270
# % 05: 2011_09_30_drive_0018 000000 002760
# % 06: 2011_09_30_drive_0020 000000 001100
# % 07: 2011_09_30_drive_0027 000000 001100
# % 08: 2011_09_30_drive_0028 001100 005170
# % 09: 2011_09_30_drive_0033 000000 001590
# % 10: 2011_09_30_drive_0034 000000 001200

DATES=(
"2011_10_03"
"2011_10_03"
"2011_10_03"
"2011_09_30"
"2011_09_30"
"2011_09_30"
"2011_09_30"
"2011_09_30"
"2011_09_30"
"2011_09_30"
)

DRIVES=(
"0027"
"0042"
"0034"
"0016"
"0018"
"0020"
"0027"
"0028"
"0033"
"0034"
)

# for ((i=3; i<4; ++i));
for ((i=0; i<${#DRIVES[@]}; ++i));
do
    :
    DRIVESTR="${DATES[i]}_drive_${DRIVES[i]}_sync"
    DRIVEDIR="${DATADIR}/${DATES[i]}/${DRIVESTR}"

    TRACKFILE="${DRIVEDIR}/${DRIVESTR}_viso2.csv"
    REF_SUNFILE="${DRIVEDIR}/sun_dir_ephemeris.csv"

    OBS_SUNFILE_GTSUN="${DRIVEDIR}/sun_dir_gtsun.csv"
    CMD_GTSUN="${EXECUTABLE} ${TRACKFILE} ${REF_SUNFILE} ${OBS_SUNFILE_GTSUN} --window ${WINDOW} --sun-only"
    echo ${CMD_GTSUN}
    ${CMD_GTSUN}

    OBS_SUNFILE_CNN="${DRIVEDIR}/sun_dir_cnn.csv"
    CMD_CNN="${EXECUTABLE} ${TRACKFILE} ${REF_SUNFILE} ${OBS_SUNFILE_CNN} --window ${WINDOW} --sun-only"
    echo ${CMD_CNN}
    # ${CMD_CNN}


done
