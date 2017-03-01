#!/bin/bash

HOME_DIR=~
DATA_DIR="${HOME_DIR}/Desktop/sim_datasets"
EXECUTABLE=../build/dataset_vo_sun
WINDOW=2

TRAJECTORIES=(
# "circle200"
"triangle200"
"square200"
"penta200"
)

SUNINTERVAL_DIR=(
"every1"
# "every5"
# "every10"
)

OBS_SUNFILE_NAMES=(
"sun_dir_gtsun0.csv"
"sun_dir_gtsun10.csv"
"sun_dir_gtsun20.csv"
"sun_dir_gtsun30.csv"
)

# for ((i=0; i<1; ++i));
for ((i=0; i<${#TRAJECTORIES[@]}; ++i));
do
    :
    for ((k=0; k<${#SUNINTERVAL_DIR[@]}; ++k))
    do
        :
        TRAJ_DIR="${DATA_DIR}/${TRAJECTORIES[i]}"

        TRACKFILE="${TRAJ_DIR}/${TRAJECTORIES[i]}.csv"
        REF_SUNFILE="${TRAJ_DIR}/sun_dir_ephemeris.csv"

        for ((j=0; j<${#OBS_SUNFILE_NAMES[@]}; ++j));
        do
            :
            OBS_SUNFILE="${TRAJ_DIR}/${SUNINTERVAL_DIR[k]}/${OBS_SUNFILE_NAMES[j]}"
            CMD="${EXECUTABLE} ${TRACKFILE} ${REF_SUNFILE} ${OBS_SUNFILE} --window ${WINDOW}"

            # Only do the no-sun case once
            if ((j!=0))
            then
                CMD="${CMD} --sun-only"
            fi

            # CMD="${CMD} --huber-param 1.345"
            # CMD="${CMD} --huber-param 0.743" # 1/1.345

            echo ${CMD}
            ${CMD}
        done

        MVCMD="mv ${TRAJ_DIR}/*_poses.csv ${TRAJ_DIR}/${SUNINTERVAL_DIR[k]}/"
        echo ${MVCMD}
        ${MVCMD}
    done
done
