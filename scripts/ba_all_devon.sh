#!/bin/bash

HOME_DIR=~
DATA_DIR="${HOME_DIR}/Desktop/Devon-Island/rover-traverse/color-rectified-1280x960"
EXECUTABLE=../build/dataset_vo_sun
WINDOW=2

TRAJECTORIES=(
"s00"
"s01"
# "s02"
"s03"
"s04"
"s05"
"s06"
"s07"
"s08"
"s09"
"s10"
"s11"
"s12"
"s13"
"s14"
"s15"
# "s16"
"s17"
# "s18"
# "s19"
"s20"
"s21"
"s22"
)

SUNINTERVAL_DIR=(
"every1"
# "every5"
)

OBS_SUNFILE_NAMES=(
"sun_dir_sensor.csv"
"sun_dir_lalonde.csv"
"sun_dir_lalondevo.csv"
)

for ((i=0; i<${#TRAJECTORIES[@]}; ++i));
do
    :
    for ((k=0; k<${#SUNINTERVAL_DIR[@]}; ++k))
    do
        :
        TRAJ_DIR="${DATA_DIR}/color-rectified-1280x960-${TRAJECTORIES[i]}/vo"

        TRACKFILE="${TRAJ_DIR}/${TRAJECTORIES[i]}_viso2.csv"
        REF_SUNFILE="${TRAJ_DIR}/sun_dir_ephemeris.csv"

        for ((j=1; j<${#OBS_SUNFILE_NAMES[@]}; ++j));
        do
            :
            OBS_SUNFILE="${TRAJ_DIR}/${SUNINTERVAL_DIR[k]}/${OBS_SUNFILE_NAMES[j]}"
            CMD="${EXECUTABLE} ${TRACKFILE} ${REF_SUNFILE} ${OBS_SUNFILE} --window ${WINDOW}"

            # Only do the no-sun case once
            if ((j!=0))
            then
                CMD="${CMD} --sun-only"
            fi

            echo ${CMD}
            ${CMD}
        done

        MVCMD="mv ${TRAJ_DIR}/*_poses.csv ${TRAJ_DIR}/${SUNINTERVAL_DIR[k]}/"
        echo ${MVCMD}
        ${MVCMD}
    done
done
