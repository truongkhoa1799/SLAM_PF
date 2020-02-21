
*)make sure all env are set correct:

export ROBOTPKG_BASE=/usr/local
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROBOTPKG_BASE/lib
export LTDL_LIBRARY_PATH=$LTDL_LIBRARY_PATH:PATH_TO_CATKIN_WS/devel/lib
export OPRS_DIR=PATH_TO_CATKIN_WS/src/advanced_robotics/asreal_oprs
export OPRS_DATA_PATH=$OPRS_DIR/data
export OPRS_DOC_DIR=$OPRS_DIR/doc
export XFILESEARCHPATH=$OPRS_DIR/%T/%N

export ASRAEL_BASE=/usr/local
export ASRAEL_INTERFACE_INCLUDE_DIR=$ASRAEL_BASE/include/asrael
export ASRAEL_INTERFACE_LIB_DIR=$ASRAEL_BASE/lib

*) start Asrael Server e.g for 64 bit:
./Asrael_Linux_v1.02.x86_64

*)launch interface
roslaunch asreal_oprs asrael.launch 

*)start oprs test plan(move wooden bowl to table)
xoprs localhost -a -A -d $OPRS_DATA_PATH -x $OPRS_DATA_PATH/move.inc

*)post new goal
Oprs->Add Fact Or Goal:
Fact or Goal: (!(is_at WoodenBowl Table))

