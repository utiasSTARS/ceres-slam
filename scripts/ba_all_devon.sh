#!/bin/bash

HOME_DIR=~
DATA_DIR="${HOME_DIR}/Desktop/Devon-Island/processed"
EXECUTABLE=../build/dataset_vo_sun
WINDOW=2

# TRAJECTORIES=(
# "s00"
# "s01"
# "s02" # Lalonde crash
# "s03"
# "s04"
# "s05"
# "s06"
# "s07"
# "s08"
# "s09"
# "s10"
# "s11"
# "s12"
# "s13"
# "s14"
# "s15"
# "s16" # Lalonde crash
# "s17"
# "s18" # Lalonde crash
# "s19" # Lalonde crash
# "s20"
# "s21"
# "s22"
# )

TRAJECTORIES=(
"c00"
"c01"
"c02"
"c03"
"c04"
"c05"
"c06"
"c07"
"c08"
"c09"
"c10"
)

SUNINTERVAL_DIR=(
"every1"
# "every5"
)

OBS_SUNFILE_NAMES=(
"sun_dir_sensor.csv"
"sun_dir_lalonde.csv"
"sun_dir_lalondevo.csv"
"sun_dir_starscnn.csv"
)

for ((i=9; i<10; ++i))
# for ((i=0; i<${#TRAJECTORIES[@]}; ++i));
do
    :
    for ((k=0; k<${#SUNINTERVAL_DIR[@]}; ++k))
    do
        :
        TRAJ_DIR="${DATA_DIR}/${TRAJECTORIES[i]}"

        TRACKFILE="${TRAJ_DIR}/${TRAJECTORIES[i]}_viso2.csv"
        REF_SUNFILE="${TRAJ_DIR}/sun_dir_ephemeris.csv"

        for ((j=1; j<3; ++j));
        # for ((j=0; j<${#OBS_SUNFILE_NAMES[@]}; ++j));
        do
            :
            OBS_SUNFILE="${TRAJ_DIR}/${SUNINTERVAL_DIR[k]}/${OBS_SUNFILE_NAMES[j]}"
            CMD="${EXECUTABLE} ${TRACKFILE} ${REF_SUNFILE} ${OBS_SUNFILE} --window ${WINDOW}"

            # Only do the no-sun case once
            if ((j!=0))
            then
                CMD="${CMD} --sun-only"
            fi

            # Use a Huber robust loss for non-GT-Sun predictions
            CMD="${CMD} --huber-param 1.345"
            # CMD="${CMD} --huber-param 0.743" # 1/1.345

            echo ${CMD}
            ${CMD}
        done

        MVCMD="mv ${TRAJ_DIR}/*_poses.csv ${TRAJ_DIR}/${SUNINTERVAL_DIR[k]}/"
        echo ${MVCMD}
        ${MVCMD}
    done
done
