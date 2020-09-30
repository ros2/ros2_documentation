#!/bin/bash

# first argument must be setup.bash of ROS2
SETUP_FILE=${1}
# if second argument is SERVER it uses Discovery Service
PROTOCOL=${2}

# Prepare environment
echo "source to file: " ${SETUP_FILE}
source ${SETUP_FILE}

# Dump file for capture
DUMP_FILE="simple.pcapng"
if [[ ${PROTOCOL} == "SERVER" ]]
then
    DUMP_FILE="server_client.pcapng"
    echo "Run in Discovery Service mode"
else
    unset ROS_DISCOVERY_SERVER
    echo "Run in Simple Discovery mode"
fi

# time running
RUN_TIME=15

# Start capture
rm -f ${DUMP_FILE} > /dev/null 2>&1
tcpdump -G $((RUN_TIME + 2)) -W 1 -i any -nn -s 0 -w ${DUMP_FILE} > /dev/null 2>&1 &
TCPDUMP_PID=$!

# Start talker in SERVER or SIMPLE mode
if [[ ${PROTOCOL} == "SERVER" ]]
then

    # Start Discovery Server
    fast-discovery-server -i 0 -g > /dev/null &

    # wait until server ready
    sleep 1

    # Env variable to set new nodes to use Discovery Service
    export ROS_DISCOVERY_SERVER=127.0.0.1:11811

    # Run talker
    echo "Spawn talker"
    ros2 run demo_nodes_cpp talker --ros-args --remap __node:=server_talker &

    echo "Spawn first listener 0"
    ros2 run demo_nodes_cpp \
        listener --ros-args --remap __node:=listener_0 &

else

    # Run simple discovery talker
    echo "Spawn talker"
    ros2 run demo_nodes_cpp \
        talker --ros-args --remap __node:=simple_talker &
    
    echo "Spawn first listener 0"
    ros2 run demo_nodes_cpp \
        listener --ros-args --remap __node:=listener_0 &
fi

# Spawn 50 listeners. They will be CLIENTS if ${PROTOCOL} is SERVER, else they will
# be simple participants
for i in {1..50}
do
    ros2 run demo_nodes_cpp \
        listener --ros-args --remap __node:=listener_${i} > /dev/null 2>&1  &
done

# Wait for tcpdump to finish and send ctrl-c to talker and listeners
sleep $RUN_TIME
kill -s SIGINT $(ps -C talker) > /dev/null 2>&1
kill -s SIGINT $(ps -C listener) > /dev/null 2>&1

# ends all discovery servers
if [[ ${PROTOCOL} == "SERVER" ]]
then
    pkill discovery
fi

sleep 1

echo "Traffic capture can be found in: ${DUMP_FILE}"

# Make sure they are killed
pkill talker
pkill listener
