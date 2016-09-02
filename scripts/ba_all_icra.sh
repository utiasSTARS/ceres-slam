#!/bin/bash

HOMEDIR=~
DATADIR="${HOMEDIR}/Desktop/odometry_raw"
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

GTFILES=(
"${DATADIR}/2011_10_03/2011_10_03_drive_0027_sync/2011_10_03_drive_0027_sync_viso2_gt.csv"
"${DATADIR}/2011_10_03/2011_10_03_drive_0042_sync/2011_10_03_drive_0042_sync_viso2_gt.csv"
"${DATADIR}/2011_10_03/2011_10_03_drive_0034_sync/2011_10_03_drive_0034_sync_viso2_gt.csv"
"${DATADIR}/2011_09_30/2011_09_30_drive_0016_sync/2011_09_30_drive_0016_sync_viso2_gt.csv"
"${DATADIR}/2011_09_30/2011_09_30_drive_0018_sync/2011_09_30_drive_0018_sync_viso2_gt.csv"
"${DATADIR}/2011_09_30/2011_09_30_drive_0020_sync/2011_09_30_drive_0027_sync_viso2_gt.csv"
"${DATADIR}/2011_09_30/2011_09_30_drive_0028_sync/2011_09_30_drive_0028_sync_viso2_gt.csv"
"${DATADIR}/2011_09_30/2011_09_30_drive_0033_sync/2011_09_30_drive_0033_sync_viso2_gt.csv"
"${DATADIR}/2011_09_30/2011_09_30_drive_0034_sync/2011_09_30_drive_0034_sync_viso2_gt.csv"
)

for ((i=3; i<4; ++i));
# for ((i=0; i<${#OLDFILES[@]}; ++i));
do
    :
    GTCMD="${EXECUTABLE} ${GTFILES[i]} --window ${WINDOW}"
    echo ${GTCMD}
    ${GTCMD}

    CNNCMD="${EXECUTABLE} ${CNNFILES[i]} --window ${WINDOW} --sun-only"
    echo ${CNNCMD}
    # ${CNNCMD}


done
