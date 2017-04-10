#ifndef SIMULATION_VREP_TASK_TASK_HPP
#define SIMULATION_VREP_TASK_TASK_HPP

#include "simulation_vrep/TaskBase.hpp"
#include <vrep/vrep.hpp>

namespace simulation_vrep
{
    enum MOTOR_NAMES
    {
      DFL,
      DFR,
      DCL,
      DCR,
      DBL,
      DBR,
      SFL,
      SFR,
      SBL,
      SBR,
      GDR,
      GST
    };

    class Task : public TaskBase
    {
  	friend class TaskBase;
    protected:
        vrep::VREP *vrep;

        // Handlers for motors
        static const int motor_number = 10;
        std::vector<int> motor_handles;
        std::vector<std::string> motor_names;

        base::commands::Motion2D motion_command;
        base::samples::Joints joints_commands;
        base::samples::Joints joints_readings;
        base::samples::RigidBodyState gps_pose_samples;
        base::samples::RigidBodyState imu_pose_samples;
        std::vector<base::Waypoint> trajectory;
        base::Waypoint currentWaypoint;
        base::samples::RigidBodyState gps_heading_input;

        float yaw_drift;
        bool gps_rtk_fix;

        // PTU stuff
        double pan_set, tilt_set, pan_angle, tilt_angle;
        int ptu_pan_motor_handler, ptu_tilt_motor_handler;
        base::samples::frame::Frame left_frame, right_frame;
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> ptr_left_frame, ptr_right_frame;
    public:
        Task(std::string const& name = "simulation_vrep::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);
	       ~Task();
        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif
