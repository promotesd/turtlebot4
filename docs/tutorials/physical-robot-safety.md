# Physical-robot evaluation safety

Do not train an exploratory policy on an unattended physical robot. Prepare a clear
area, accessible emergency stop, speed limits, charged robot, reliable network, and a
second person when possible.

Validate in this order: receive sensors without publishing; publish zero velocity;
send one very-low-speed action; run a short human-supervised episode; evaluate a fixed
goal; then evaluate the complete frozen policy. Confirm stale sensor data, command
timeout, exception, terminal state, Ctrl-C, and process shutdown each produce zero
velocity. Never enable simulator teleport/reset code in this mode.

The ROS adapter compares non-zero message header stamps with the node clock, rejects
old or implausibly future samples, and uses monotonic receipt time for watchdogs. Set
`use_sim_time: true` only with a valid `/clock` source. A backward ROS-time jump clears
cached scan/odometry and publishes zero velocity before accepting a new complete pair.
