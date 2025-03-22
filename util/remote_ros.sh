#! /bin/sh
#
# This dot-script can be sourced for updating ROS_IP etc.
#
# Can be given with or without HOSTNAME, default is to use this machine's
# hostname.
# > source util/remote_ros [ HOSTNAME ]
#
# Author: Kaj Munhoz Arfvidsson


unset ROS_IP
unset ROS_HOSTNAME

# Get hostname and master host
hostname="$(hostname)"
master="${1:-$hostname}"

ROS_HOSTNAME="$hostname"
ROS_MASTER_URI="http://$master:11311"

export ROS_IP
export ROS_MASTER_URI
export ROS_HOSTNAME

