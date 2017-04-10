#!/usr/bin/env ruby

require 'orocos'
#require 'rock/bundle'
require 'readline'
include Orocos

# Initialize bundles to find the configurations for the packages
Orocos.initialize
# Point to the configuration folder in the bundles
Orocos.conf.load_dir('../../../../bundles/hdpr/config/orogen/')

$distance_360_picture = 30

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
    Orocos.conf.apply(trajectoryGen, ['decos'], :override => true)
    trajectoryGen.configure

    motion_translator = Orocos.name_service.get 'motion_translator'
    motion_translator.speedRatioStep = 1
    motion_translator.configure

    pancam_panorama = Orocos.name_service.get 'pancam_panorama'
    Orocos.conf.apply(pancam_panorama, ['default'], :override => true)
    pancam_panorama.configure

    # Configure
    gps_heading = TaskContext.get 'gps_heading'
    Orocos.conf.apply(gps_heading, ['default'], :override => true)
    gps_heading.configure

    pancam_360 = Orocos.name_service.get 'pancam_360'
    Orocos.conf.apply(pancam_360, ['default'], :override => true)
    pancam_360.configure

    #logger = Orocos.name_service.get 'hdpr_simulation_Logger'
    #logger.file = "simulation.log"
    #logger.log(command_arbiter.)
    #logger.start

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

    pancam_panorama.pan_angle_out.connect_to            simulation.pan_set
    pancam_panorama.tilt_angle_out.connect_to           simulation.tilt_set
    simulation.pan_angle.connect_to                     pancam_panorama.pan_angle_in
    simulation.tilt_angle.connect_to                    pancam_panorama.tilt_angle_in
    simulation.ptu_left_frame.connect_to                pancam_panorama.left_frame_in
    simulation.ptu_right_frame.connect_to               pancam_panorama.right_frame_in

    pancam_360.pan_angle_out.connect_to                 simulation.pan_set
    pancam_360.tilt_angle_out.connect_to                simulation.tilt_set
    simulation.pan_angle.connect_to                     pancam_360.pan_angle_in
    simulation.tilt_angle.connect_to                    pancam_360.tilt_angle_in
    simulation.ptu_left_frame.connect_to                pancam_360.left_frame_in
    simulation.ptu_right_frame.connect_to               pancam_360.right_frame_in

    simulation.start
    locomotion_control.start
    command_arbiter.start
    motion_translator.start
    joystick.start
    gps_heading.start

    puts "Move HDPR forward to calubrate the heading"
    while gps_heading.state != :RUNNING
        sleep 1
    end

    # Trigger the trajectory generator while waypoint navigation is already working
    waypoint_navigation.start
    trajectoryGen.start
    trajectoryGen.trigger
    waypoint_navigation.stop

    reader_gps_position = gps_heading.pose_samples_out.reader

    # Initialise GPS position
    $last_gps_position = 0
    $distance = 0

    while true
        if pancam_360.state == :RUNNING
            puts "Still taking a picture, waiting 1 seconds"
            sleep 1
        elsif pancam_360.state == :STOPPED
            puts "360 degree picture done"
            pancam_panorama.start

            # Start waypoint navigation
            waypoint_navigation.start

            # Wait for the rover to move for a defined distance in meters
            while $distance < $distance_360_picture or $distance.nan?
                sample = reader_gps_position.read_new
                if sample
                    # Initialise GPS position
                    if $last_gps_position == 0
                        $last_gps_position = sample
                    end

                    # Evaluate distance from last position
                    dx = sample.position[0] - $last_gps_position.position[0]
                    dy = sample.position[1] - $last_gps_position.position[1]
                    # Cumulative distance
                    $distance = Math.sqrt(dx*dx + dy*dy)
                    #puts "Distance: #{$distance}"
                end
            end
            $last_gps_position = sample
            $distance = 0

            # Stop sending waypoint navigation commands, this should also stop the rover now so the following lines are not required
            waypoint_navigation.stop

            # Stop the panorama taking
            pancam_panorama.stop
            puts "Taking new 360 degree picture"
            pancam_360.start
        end
    end
end
