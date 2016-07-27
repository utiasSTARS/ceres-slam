#!/bin/bash

HOMEDIR=~
EXECUTABLE=../build/dataset_vo_sun
WINDOW=2

OLDFILES=(
"${HOMEDIR}/osx_desktop/datasets/2011_09_26/2011_09_26_drive_0019_sync/2011_09_26_drive_0019_sync_viso2.csv"
"${HOMEDIR}/osx_desktop/datasets/2011_09_26/2011_09_26_drive_0039_sync/2011_09_26_drive_0039_sync_viso2.csv"
"${HOMEDIR}/osx_desktop/datasets/2011_09_30/2011_09_30_drive_0018_sync/2011_09_30_drive_0018_sync_viso2.csv"
"${HOMEDIR}/osx_desktop/datasets/2011_09_30/2011_09_30_drive_0020_sync/2011_09_30_drive_0020_sync_viso2.csv"
"${HOMEDIR}/osx_desktop/datasets/2011_10_03/2011_10_03_drive_0027_sync/2011_10_03_drive_0027_sync_viso2.csv"
)

NEWFILES=(
"${HOMEDIR}/osx_desktop/datasets/2011_09_26/2011_09_26_drive_0019_sync/2011_09_26_drive_0019_sync_viso2_newprior.csv"
"${HOMEDIR}/osx_desktop/datasets/2011_09_26/2011_09_26_drive_0039_sync/2011_09_26_drive_0039_sync_viso2_newprior.csv"
"${HOMEDIR}/osx_desktop/datasets/2011_09_30/2011_09_30_drive_0018_sync/2011_09_30_drive_0018_sync_viso2_newprior.csv"
"${HOMEDIR}/osx_desktop/datasets/2011_09_30/2011_09_30_drive_0020_sync/2011_09_30_drive_0020_sync_viso2_newprior.csv"
"${HOMEDIR}/osx_desktop/datasets/2011_10_03/2011_10_03_drive_0027_sync/2011_10_03_drive_0027_sync_viso2_newprior.csv"
)

CNNFILES=(
"${HOMEDIR}/osx_desktop/datasets/2011_09_26/2011_09_26_drive_0019_sync/2011_09_26_drive_0019_sync_viso2_cnn.csv"
"${HOMEDIR}/osx_desktop/datasets/2011_09_26/2011_09_26_drive_0039_sync/2011_09_26_drive_0039_sync_viso2_cnn.csv"
"${HOMEDIR}/osx_desktop/datasets/2011_09_30/2011_09_30_drive_0018_sync/2011_09_30_drive_0018_sync_viso2_cnn.csv"
"${HOMEDIR}/osx_desktop/datasets/2011_09_30/2011_09_30_drive_0020_sync/2011_09_30_drive_0020_sync_viso2_cnn.csv"
"${HOMEDIR}/osx_desktop/datasets/2011_10_03/2011_10_03_drive_0027_sync/2011_10_03_drive_0027_sync_viso2_cnn.csv"
)

for ((i=2; i<3; ++i));
# for ((i=0; i<${#OLDFILES[@]}; ++i));
do
    :
    OLDCMD="${EXECUTABLE} ${OLDFILES[i]} --window ${WINDOW} --sun-only"
    echo ${OLDCMD}
    # ${OLDCMD}

    NEWCMD="${EXECUTABLE} ${NEWFILES[i]} --window ${WINDOW} --sun-only"
    echo ${NEWCMD}
    ${NEWCMD}

    CNNCMD="${EXECUTABLE} ${CNNFILES[i]} --window ${WINDOW} --sun-only"
    echo ${CNNCMD}
    # ${CNNCMD}
done
