#! /bin/bash

set -e
set -o nounset
set -o pipefail

export FW_TARGETDIR=$(pwd)/firmware
export PREFIX=$(ros2 pkg prefix micro_ros_setup)

function help {
      echo "Configure script need an argument."
      echo "   --transport -t       udp, serial or serial-usb"
      echo "   --ip -i              agent IP in a network-like transport"
      echo "   --port -p            agent port in a network-like transport"
}

# Checking if firmware exists
if [ -d $FW_TARGETDIR ]; then
    RTOS=$(head -n1 $FW_TARGETDIR/PLATFORM)
    PLATFORM=$(head -n2 $FW_TARGETDIR/PLATFORM | tail -n1)
else
    echo "Firmware folder not found. Please use ros2 run micro_ros_setup create_firmware_ws.sh to create a new project."
    exit 1
fi


# Parsing micro-ROS arguments
if [ $# -lt 1 ]; then
  help
  exit 1
fi


while [[ $# -gt 0 ]]; do
  key="$1"

  case $key in
      -t|--transport)
      export UROS_TRANSPORT="$2"
      shift # past argument
      shift # past value
      ;;
      -d|--dev)
      export UROS_AGENT_DEVICE="$2"
      shift # past argument
      shift # past value
      ;;
      -i|--ip)
      export UROS_AGENT_IP="$2"
      shift # past argument
      shift # past value
      ;;
      -p|--port)
      export UROS_AGENT_PORT="$2"
      shift # past argument
      shift # past value
      ;;
      *)    # unknown option
      echo "Unknown argument  $1"
      exit 1
      ;;
  esac
done

# Configure specific firmware folder if needed
. $PREFIX/config/utils.sh

remove_meta "rmw_microxrcedds" "RMW_UXRCE_MAX_NODES"
remove_meta "rmw_microxrcedds" "RMW_UXRCE_MAX_PUBLISHERS"
remove_meta "rmw_microxrcedds" "RMW_UXRCE_MAX_SUBSCRIPTIONS"
remove_meta "rmw_microxrcedds" "RMW_UXRCE_MAX_SERVICES"
remove_meta "rmw_microxrcedds" "RMW_UXRCE_MAX_HISTORY"
remove_meta "rmw_microxrcedds" "RMW_UXRCE_MAX_CLIENTS"

if [ "$UROS_TRANSPORT" == "udp" ]; then

      update_meta "rmw_microxrcedds" "RMW_UXRCE_TRANSPORT="$UROS_TRANSPORT
      update_meta "rmw_microxrcedds" "RMW_UXRCE_DEFAULT_UDP_IP="$UROS_AGENT_IP
      update_meta "rmw_microxrcedds" "RMW_UXRCE_DEFAULT_UDP_PORT="$UROS_AGENT_PORT

      update_meta "microxrcedds_client" "UCLIENT_PROFILE_CUSTOM_TRANSPORT=OFF"
      update_meta "microxrcedds_client" "UCLIENT_PROFILE_SERIAL=OFF"
      update_meta "microxrcedds_client" "UCLIENT_PROFILE_UDP=ON"
      update_meta "microxrcedds_client" "UCLIENT_PROFILE_TCP=OFF"
      update_meta "microxrcedds_client" "UCLIENT_PROFILE_DISCOVERY=ON"
      
      echo "Configured $UROS_TRANSPORT mode with agent at $UROS_AGENT_IP:$UROS_AGENT_PORT"

elif [ "$UROS_TRANSPORT" == "serial" ]; then
      echo "Using serial device USART."

      echo "Please check firmware/freertos_apps/microros_nucleo_f767zi_extensions/Src/main.c"
      echo "for configuring serial device before build."

      update_meta "microxrcedds_client" "UCLIENT_PROFILE_CUSTOM_TRANSPORT=ON"
      update_meta "microxrcedds_client" "UCLIENT_PROFILE_STREAM_FRAMING=ON"
      update_meta "microxrcedds_client" "UCLIENT_PROFILE_SERIAL=OFF"
      update_meta "microxrcedds_client" "UCLIENT_PROFILE_UDP=OFF"
      update_meta "microxrcedds_client" "UCLIENT_PROFILE_TCP=OFF"
      update_meta "microxrcedds_client" "UCLIENT_PROFILE_DISCOVERY=OFF"

      update_meta "rmw_microxrcedds" "RMW_UXRCE_TRANSPORT=custom"

      remove_meta "rmw_microxrcedds" "RMW_UXRCE_DEFAULT_UDP_IP"
      remove_meta "rmw_microxrcedds" "RMW_UXRCE_DEFAULT_UDP_PORT"

      echo "Configured $UROS_TRANSPORT mode with agent at USART"
else
      help
fi
