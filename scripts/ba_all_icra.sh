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
)

OBS_SUNFILE_NAMES=(
"sun_dir_gtsun0.csv"
"sun_dir_gtsun10.csv"
"sun_dir_gtsun20.csv"
"sun_dir_gtsun30.csv"
"sun_dir_starscnn.csv"
"sun_dir_suncnn.csv"
)

for ((k=0; k<${#SUNINTERVAL_DIR[@]}; ++k))
do
    :
    for ((i=0; i<1; ++i));
    # for ((i=0; i<${#DRIVES[@]}; ++i));
    do
        :
        DRIVE_STR="${DATES[i]}_drive_${DRIVES[i]}_sync"
        DRIVE_DIR="${DATA_DIR}/${DATES[i]}/${DRIVE_STR}"

        TRACKFILE="${DRIVE_DIR}/${DRIVE_STR}_viso2.csv"
        REF_SUNFILE="${DRIVE_DIR}/sun_dir_ephemeris.csv"

        for ((j=4; j<5; ++j));
        # for ((j=0; j<${#OBS_SUNFILE_NAMES[@]}; ++j));
        do
            :
            if ((j!=5 || j==5 && (i==0 || i==4 || i==5)))
            then
                OBS_SUNFILE="${DRIVE_DIR}/${SUNINTERVAL_DIR[k]}/${OBS_SUNFILE_NAMES[j]}"
                CMD="${EXECUTABLE} ${TRACKFILE} ${REF_SUNFILE} ${OBS_SUNFILE} --window ${WINDOW}"

                # Only do the no-sun case once
                # if ((j!=0))
                # then
                    CMD="${CMD} --sun-only"
                # fi

                # 20 deg (0.05) threshold for CNNs
                if((j==4 || j==5))
                then
                    CMD="${CMD} --cosine-dist-thresh 0.05"
                fi

                # Sun-CNN only gives azimuth
                if((j==5))
                then
                    CMD="${CMD} --azimuth-only"
                fi

                echo ${CMD}
                ${CMD}
            fi
        done

        MVCMD="mv ${DRIVE_DIR}/*_poses.csv ${DRIVE_DIR}/${SUNINTERVAL_DIR}/"
        echo ${MVCMD}
        ${MVCMD}
    done
done
