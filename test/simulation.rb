#!/usr/bin/env ruby

require 'orocos'
#require 'rock/bundle'
require 'readline'
include Orocos

# Initialize bundles to find the configurations for the packages
Orocos.initialize
# Point to the configuration folder in the bundles
Orocos.conf.load_dir('../../../../bundles/hdpr/config/orogen/')

# Execute the deployement
Orocos::Process.run 'hdpr_simulation' do

    simulation = Orocos.name_service.get 'simulation'
    Orocos.conf.apply(simulation, ['default'], :override => true)
    simulation.configure

    locomotion_control = Orocos.name_service.get 'locomotion_control'
    Orocos.conf.apply(locomotion_control, ['default'], :override => true)
    locomotion_control.configure

    joystick = Orocos.name_service.get 'joystick'
    #Orocos.conf.apply(simulation, ['default'], :override => true)
    joystick.configure

    waypoint_navigation = Orocos.name_service.get 'waypoint_navigation'
    Orocos.conf.apply(waypoint_navigation, ['default', 'hdpr'], :override => true)
    waypoint_navigation.configure

    # Setup command arbiter
    command_arbiter = Orocos.name_service.get 'command_arbiter'
    Orocos.conf.apply(command_arbiter, ['default'], :override => true)
    command_arbiter.configure

    # Add the trajectory generation component
    trajectoryGen = Orocos.name_service.get 'trajectoryGen'
    Orocos.conf.apply(trajectoryGen, ['hdprtest','decos'], :override => true)
    trajectoryGen.configure

    motion_translator = Orocos.name_service.get 'motion_translator'
    motion_translator.speedRatioStep = 1
    motion_translator.configure

    # Configure
    gps_heading = TaskContext.get 'gps_heading'
    Orocos.conf.apply(gps_heading, ['default'], :override => true)
    gps_heading.configure

    joystick.raw_command.connect_to                     motion_translator.raw_command
    joystick.raw_command.connect_to                     command_arbiter.raw_command

    motion_translator.motion_command.connect_to         command_arbiter.joystick_motion_command
    waypoint_navigation.motion_command.connect_to       command_arbiter.follower_motion_command
    command_arbiter.motion_command.connect_to           locomotion_control.motion_command
    locomotion_control.joints_commands.connect_to       simulation.joints_commands
    simulation.joints_readings.connect_to               locomotion_control.joints_readings

    trajectoryGen.trajectory.connect_to                 waypoint_navigation.trajectory
    trajectoryGen.trajectory.connect_to                 simulation.trajectory
    waypoint_navigation.currentWaypoint.connect_to      simulation.currentWaypoint

    simulation.gps_pose_samples.connect_to              gps_heading.gps_pose_samples
    simulation.imu_pose_samples.connect_to              gps_heading.imu_pose_samples
    command_arbiter.motion_command.connect_to           gps_heading.motion_command
    simulation.gps_raw_data.connect_to                  gps_heading.gps_raw_data
    gps_heading.pose_samples_out.connect_to             waypoint_navigation.pose
    gps_heading.pose_samples_out.connect_to             simulation.gps_heading_input

    trajectoryGen.start
    waypoint_navigation.start

    simulation.start
    locomotion_control.start
    command_arbiter.start
    motion_translator.start
    joystick.start
    gps_heading.start

    Readline::readline("Press Enter to exit\n") do
    end
end
