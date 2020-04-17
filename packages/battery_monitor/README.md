# Package `battery_monitor` 

The new Duckietown batteries output the charge percentage over Serial via USB. This package reads
that data and publishes it as a ROS topic. At startup, the node will try to communicate with the battery.
If it fails, the node will cleanly shut down. This is so that this driver can be started on every duckiebot,
even if it is not using the new battery.

