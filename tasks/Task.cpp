#include "Task.hpp"

using namespace simulation_vrep;

Task::Task(std::string const& name)
    : TaskBase(name)
{

}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{

}

Task::~Task()
{

}

bool Task::configureHook()
{
    if(!TaskBase::configureHook())
    {
        return false;
    }

    vrep = new vrep::VREP();

    if(vrep->getClientId() == -1)
    {
        printf("Could not reach VREP simulation, make sure the simulation is running in VREP\n");
        return false;
    }

    motor_names.push_back("joint_drive_fl");
    motor_names.push_back("joint_drive_fr");
    motor_names.push_back("joint_drive_ml");
    motor_names.push_back("joint_drive_mr");
    motor_names.push_back("joint_drive_bl");
    motor_names.push_back("joint_drive_br");
    motor_names.push_back("joint_steer_fl");
    motor_names.push_back("joint_steer_fr");
    motor_names.push_back("joint_steer_bl");
    motor_names.push_back("joint_steer_br");

    joints_readings.resize(motor_number);

    int handle;
    for(int i = 0; i < motor_number; i++)
    {
        vrep->getObjectHandle(motor_names[i], &handle);
        motor_handles.push_back(handle);
    }

    yaw_drift = 0;

    std::string const message("Controller configured");
    vrep->sendStatusMessage(message.c_str());

    return true;
}

bool Task::startHook()
{
    if(!TaskBase::startHook())
    {
        return false;
    }
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    if(_joints_commands.read(joints_commands) == RTT::NewData)
    {
        for(int i = 0; i < 6; i++)
        {
            if(!isnan(joints_commands.elements[i].speed))
            {
                // Set driving motor speeds one by one, if no speed is defined the value is "nan"
                vrep->setJointVelocity(motor_handles[i], joints_commands.elements[i].speed);
            }

            if(!isnan(joints_commands.elements[GDR].speed))
            {
                // Apply driving group speeds, if no group command is sent the value is "nan"
                vrep->setJointVelocity(motor_handles[i], joints_commands.elements[GDR].speed);
            }
        }

        for(int i = 6; i < motor_number; i++)
        {
            if(!isnan(joints_commands.elements[i].position))
            {
                // Set steering motor positions one by one, if no speed is defined the value is "nan"
                vrep->setJointPosition(motor_handles[i], joints_commands.elements[i].position);
            }

            if(!isnan(joints_commands.elements[GST].position))
            {
                // Apply steering group positions, if no position is defined the value is "nan"
                vrep->setJointPosition(motor_handles[i], joints_commands.elements[GST].position);
            }
        }
    }

    // Get the joint readings to publish them to ROCK
    float motor_position, motor_speed;
    for(int i = 0; i < motor_number; i++)
    {
        vrep->getJointPosition(motor_handles[i], &motor_position);
        vrep->getJointVelocity(motor_handles[i], &motor_speed);
        joints_readings.elements[i].position = (double)motor_position;
        joints_readings.elements[i].speed = (double)motor_speed;
    }

    joints_readings.time = base::Time::now();
    _joints_readings.write(joints_readings);

    // GPS output
    // This is a terrible way to populate the RigidBodyState message
    gps_pose_samples.time = base::Time::now();
    float position[3] = {0};
    vrep->getPosition((std::string)"pose", "", position);
    gps_pose_samples.position.x() = position[0];
    gps_pose_samples.position.y() = position[1];
    gps_pose_samples.position.z() = position[2];
    _gps_pose_samples.write(gps_pose_samples);

    // IMU output, actually the only interesting component is the gyro Z-axis
    imu_pose_samples.time = base::Time::now();
    // Get the euler angles of the absolute base orientation
    float orientation[3] = {0};
    vrep->getOrientation((std::string)"pose", "", orientation);

    // Add drift to the yaw angle
    yaw_drift += _yaw_drift.value();
    // Wrap around PI
    yaw_drift = fmod(yaw_drift, M_PI);
    float yaw = orientation[2] + yaw_drift;

    Eigen::Quaterniond quaternion(cos(0.5 * yaw), 0.0, 0.0, sin(0.5 * yaw));
    imu_pose_samples.orientation = quaternion;
    _imu_pose_samples.write(imu_pose_samples);

    // GPS raw data to simulate the loss of GPS fix
    gnss_trimble::Solution gps_raw_data;
    if(vrep->getToggleButtonState("GPS_UI", 2))
    {
        gps_raw_data.positionType = gnss_trimble::RTK_FIXED;
    }
    else
    {
        gps_raw_data.positionType = gnss_trimble::RTK_FLOAT;
    }
    _gps_raw_data.write(gps_raw_data);

    // Publish the dummies representing the waypoints
    static bool trajectoryDummies = false;
    if(_trajectory.read(trajectory) == RTT::NewData && !trajectoryDummies)
    {
        int dummyHandler;
        for(int i = 0; i < trajectory.size(); i++)
        {
            const unsigned char color[3] = {255, 0, 0};
            dummyHandler = vrep->createDummy(0.5, color);
            float position[3] = {(float)trajectory[i].position.x(), (float)trajectory[i].position.y(), 0.0f};
            vrep->setPosition(dummyHandler, position);
        }
        trajectoryDummies = true;
    }

    // Publish the dummies representing the waypoints
    if(_currentWaypoint.read(currentWaypoint) == RTT::NewData)
    {
        static int waypoint_handler = -1;
        if(waypoint_handler == -1)
        {
            const unsigned char color[3] = {0, 255, 0};
            waypoint_handler = vrep->createDummy(0.5, color);
        }
        float position[3] = {(float)currentWaypoint.position.x(), (float)currentWaypoint.position.y(), 0.0f};
        vrep->setPosition(waypoint_handler, position);
    }

    // Show the calculated GPS heading position
    if(_gps_heading_input.read(gps_heading_input) == RTT::NewData)
    {
        static int gps_heading_handler = -1;
        if(gps_heading_handler == -1)
        {
            const unsigned char color[3] = {0, 0, 255};
            gps_heading_handler = vrep->createDummy(0.2, color);
        }

        // Place the GPS heading dummy
        float position[3] = {(float)gps_heading_input.position.x(), (float)gps_heading_input.position.y(), (float)gps_heading_input.position.z()};
        vrep->setPosition(gps_heading_handler, position);

        // Transform quaternion to Euler angles
        Eigen::Vector3f euler = Eigen::Quaternionf(
          gps_heading_input.orientation.x(),
          gps_heading_input.orientation.y(),
          gps_heading_input.orientation.z(),
          gps_heading_input.orientation.w()).toRotationMatrix().eulerAngles(2, 1, 0);
        // Orient the GPS heading dummy
        float orientation[3] = {-euler(0), -euler(1), -euler(2)};
        vrep->setOrientation(gps_heading_handler, orientation);
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();

    // Stop all the motors
    for(int i = 0; i < motor_number; i++)
    {
        vrep->setJointVelocity(motor_handles[i], 0.0f);
    }
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
