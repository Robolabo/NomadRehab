#! /bin/bash

set -e
set -o nounset
set -o pipefail

PREFIXES_TO_CLEAN=$AMENT_PREFIX_PATH
SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
FW_TARGETDIR=$SCRIPTPATH/firmware
PREFIX=$(ros2 pkg prefix micro_ros_setup)
BUILD_DIR=$FW_TARGETDIR/build
RTOS="generic"

# Parse cli arguments
UROS_FAST_BUILD=off
UROS_VERBOSE_BUILD=off
UROS_EXTRA_BUILD_ARGS=""


while getopts "vfo:" o
do
    case "$o" in
        f)
            echo "Fast-Build active, ROS workspace will not be re-built!"
            UROS_FAST_BUILD=on
            ;;
        v)
            echo "Building in verbose mode"
            UROS_VERBOSE_BUILD=on
            ;;

        o)
            BUILD_DIR=${OPTARG}
            echo "Output folder: $BUILD_DIR"
            ;;
        [?])
            echo "Usage: ros2 run micro_ros_setup build_firmware.sh [options] -- [build_args]"
            echo "Options:"
            echo "  -v   Print verbose build output."
            echo "  -f   Activate Fast-Build. Without this, mcu_ws will get rebuilt completely."
            echo "Build args: These options will get directly forwarded to the build system (currently only supported for zephyr)."
            exit 1
            ;;
  esac
done
shift $((OPTIND-1))

if [[ -n "$@" ]]; then
    UROS_EXTRA_BUILD_ARGS=("$@")
fi

export UROS_FAST_BUILD
export UROS_VERBOSE_BUILD
export UROS_EXTRA_BUILD_ARGS

# Checking if firmware exists
if [ ! -d $FW_TARGETDIR ]; then
    echo "Firmware folder not found. Please use ros2 run micro_ros_setup create_firmware_ws.sh to create a new project."
    exit 1
fi

# clean paths
. $PREFIX/lib/micro_ros_setup/clean_env.sh

set +o nounset
. $FW_TARGETDIR/dev_ws/install/setup.bash
set -o nounset


COLCON_META=$FW_TARGETDIR/mcu_ws/colcon.meta
TOOLCHAIN=$SCRIPTPATH/toolchain.cmake 
COLCON_META_EXTRA=$SCRIPTPATH/colcon.meta


pushd $FW_TARGETDIR/mcu_ws >/dev/null

	#rm -rf build install log
	
   	colcon build \
		--merge-install \
		--packages-ignore-regex=.*_cpp \
        --metas "$COLCON_META" "$COLCON_META_EXTRA" \
		--cmake-args \
		"--no-warn-unused-cli" \
		-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=OFF \
		-DTHIRDPARTY=ON \
		-DBUILD_SHARED_LIBS=OFF \
		-DBUILD_TESTING=OFF \
		-DCMAKE_BUILD_TYPE=Release \
		-DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN \
		-DCMAKE_VERBOSE_MAKEFILE=ON; \

    mkdir -p $FW_TARGETDIR/libmicroros; cd $FW_TARGETDIR/libmicroros; \
	for file in $(find $FW_TARGETDIR/mcu_ws/install/lib/ -name '*.a'); do \
		folder=$(echo $file | sed -E "s/(.+)\/(.+).a/\2/"); \
		mkdir -p $folder; cd $folder; ar x $file; \
		for f in *; do \
			mv $f ../$folder-$f; \
		done; \
		cd ..; rm -rf $folder; \
	done ; \
	ar rc libmicroros.a $(ls *.o *.obj 2> /dev/null); mkdir -p $BUILD_DIR; cp libmicroros.a $BUILD_DIR; ranlib $BUILD_DIR/libmicroros.a; \
    cp -R $FW_TARGETDIR/mcu_ws/install/include $BUILD_DIR/; \
	cd ..; rm -rf libmicroros;

popd >/dev/null
