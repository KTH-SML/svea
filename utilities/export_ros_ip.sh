svea_hostname=svea2
export ROS_MASTER_URI=http://$svea_hostname.local:11311

machine_ip=(`hostname -I`)
export ROS_IP=${machine_ip[0]}

hostname=(`hostname`)
if [ $hostname == $svea_hostname ]
then
  export ROS_HOSTNAME=$svea_hostname.local # For nvidia
else
  export ROS_HOSTNAME=$ROS_IP # For other machine
fi
