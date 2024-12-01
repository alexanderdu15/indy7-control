#ifndef KNOT_POINTS 
#define KNOT_POINTS 64
#endif

#ifndef TIMESTEP //timestep between knot points (seconds)
#define TIMESTEP 0.05
#endif

#ifndef STATE_SIZE
#define STATE_SIZE 12
#endif

#ifndef CONTROL_SIZE
#define CONTROL_SIZE 6
#endif

// ---------------------- trajectory_publisher.cpp ----------------------

#ifndef TRAJ_PUBLISH_PERIOD_MS // Wait time for publishing trajectories (trajectory_publisher.cpp)
#define TRAJ_PUBLISH_PERIOD_MS 1000 
#endif


// ---------------------- robot_driver_jpos.cpp ----------------------

#ifndef ROBOT_CONTROL_PERIOD_MS // Wait time for sending controls and reading state (robot_driver_jpos.cpp)
#define ROBOT_CONTROL_PERIOD_MS 20
#endif