#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/parallels/groovy_workspace/oryx/oryx_diagnostics/devel', type 'exit' to leave"
  . "/home/parallels/groovy_workspace/oryx/oryx_diagnostics/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/parallels/groovy_workspace/oryx/oryx_diagnostics/devel'"
else
  . "/home/parallels/groovy_workspace/oryx/oryx_diagnostics/devel/setup.sh"
  exec "$@"
fi
