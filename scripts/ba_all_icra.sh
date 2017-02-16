#!/bin/bash

HOME_DIR=~
DATA_DIR="${HOME_DIR}/Desktop/odometry_raw"
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

SUNINTERVAL_DIR=(
"every1"
# "every5"
"every10"
)

OBS_SUNFILE_NAMES=(
"sun_dir_gtsun0.csv"
"sun_dir_gtsun10.csv"
"sun_dir_gtsun20.csv"
"sun_dir_gtsun30.csv"
"sun_dir_starscnn.csv"
"sun_dir_suncnn.csv"
"sun_dir_lalonde.csv"
"sun_dir_lalondevo.csv"
)

for ((i=0; i<1; ++i));
# for ((i=0; i<${#SUNINTERVAL_DIR[@]}; ++i));
do
    :
    for ((j=4; j<5; ++j));
    # for ((j=0; j<${#DRIVES[@]}; ++j));
    do
        :
        DRIVE_STR="${DATES[j]}_drive_${DRIVES[j]}_sync"
        DRIVE_DIR="${DATA_DIR}/${DATES[j]}/${DRIVE_STR}"

        TRACKFILE="${DRIVE_DIR}/${DRIVE_STR}_viso2.csv"
        REF_SUNFILE="${DRIVE_DIR}/sun_dir_ephemeris.csv"

        for ((k=4; k<6; ++k));
        # for ((k=0; k<${#OBS_SUNFILE_NAMES[@]}; ++k));
        do
            :
            OBS_SUNFILE="${DRIVE_DIR}/${SUNINTERVAL_DIR[i]}/${OBS_SUNFILE_NAMES[k]}"
            CMD="${EXECUTABLE} ${TRACKFILE} ${REF_SUNFILE} ${OBS_SUNFILE} --window ${WINDOW}"

            # Only do the no-sun case once
            # if ((j!=0))
            # then
            CMD="${CMD} --sun-only"
            # fi

            # Use a Huber robust cost for non-GT-Sun predictions
            if ((k>3))
            then
                CMD="${CMD} --huber-param 2.0"
            fi

            # # 20 deg (0.05) threshold for CNNs
            # if((k==4 || k==5))
            # then
            #     CMD="${CMD} --az-err-thresh 20"
            #     CMD="${CMD} --zen-err-thresh 10"
            # fi

            # # 20 deg (0.05) threshold for Lalondes
            # if((k==6 || k==7))
            # then
            #     CMD="${CMD} --az-err-thresh 10"
            #     CMD="${CMD} --zen-err-thresh 10"
            # fi

            echo ${CMD}
            ${CMD}
        done

        MVCMD="mv ${DRIVE_DIR}/*_poses.csv ${DRIVE_DIR}/${SUNINTERVAL_DIR[i]}/"
        echo ${MVCMD}
        ${MVCMD}
    done
done
