#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/parallels/groovy_workspace/oryx/oryx_msgs/devel', type 'exit' to leave"
  . "/home/parallels/groovy_workspace/oryx/oryx_msgs/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/parallels/groovy_workspace/oryx/oryx_msgs/devel'"
else
  . "/home/parallels/groovy_workspace/oryx/oryx_msgs/devel/setup.sh"
  exec "$@"
fi
