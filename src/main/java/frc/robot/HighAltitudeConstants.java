// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.resources.Human_Drivers.HumanDrivers;

/** Add your docs here. */
public class HighAltitudeConstants {

        ////////////////////////// SWERVE //////////////////////////

        public static final double MAX_VOLTAGE = 12;
        // ponlo a 3 para pruebas

        /// CONSTANTS FOR MK4i L4 Config DRIVE MOTOR ///
        // In meters
        public static final double SWERVE_WHEEL_DIAMETER = 4.0 * 0.0254;
        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. (Driven / Driver) //pinion
        public static final double SWERVE_DRIVE_GEAR_RATIO = (50.0 * 16.0 * 45.0) / (16.0 * 28.0 * 15.0);
        
        // ft/s //ft -> in //im -> m
        public static final double SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND = 19.5 * 12 * 0.0254;

        /////////// KINEMATICS
        // Distance left - right (meters)
        public static final double SWERVE_TRACK_WIDTH = 26 * 0.0254; // este es de llanta a llanta
        // Distance front - back (meters)
        public static final double SWERVE_WHEEL_BASE = 23 * 0.0254;

        // FL, FR, BL, BR. Remember these cartesian coordinates consider the x axis to
        // be headed where the robot is pointing to. The y-axis direction could be a
        // source of problems...
        // WPILib says "Positive x values represent moving toward the front of the robot
        // whereas positive y values represent moving toward the left of the robot."
        // The example I saw uses the raw yaw reported by the navx and switches the
        // position of the left and right wheels in the kinematics.
        // I will use CCW and the allegedly correct x y coordinates.
        // For some reason, that did not work. The kinematics seem to work correctly
        // when "left" is negative
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                        new Translation2d(SWERVE_WHEEL_BASE / 2, SWERVE_TRACK_WIDTH / 2),
                        new Translation2d(SWERVE_WHEEL_BASE / 2, -SWERVE_TRACK_WIDTH / 2),
                        new Translation2d(-SWERVE_WHEEL_BASE / 2, SWERVE_TRACK_WIDTH / 2),
                        new Translation2d(-SWERVE_WHEEL_BASE / 2, -SWERVE_TRACK_WIDTH / 2));

        // Arbitrary. Higher numbers will cause the swerve to react more violently to
        // joysitck inputs and may not be ideal. Lower numbers will cause the swerve to
        // have a very slow reaction to joystick inputs, and may not be ideal.
        public static final double SWERVE_MAX_ACCELERATION_UNITS_PER_SECOND = 20.0;
        public static final double SWERVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 20.0;

        // Other

        public static final double SWERVE_ABSOLUTE_ENCODER_PULSES_PER_REVOLUTION = 1f;
        // encoder * this value = radians
        public static final double SWERVE_ABSOLUTE_ENCODER_RADIANS_PER_PULSE = (2.0 * Math.PI)
                        / SWERVE_ABSOLUTE_ENCODER_PULSES_PER_REVOLUTION;

        /////////// DRIVING MOTOR /////////// TODO: DRIVING MOTOR

        // HOW TO GET THE VALUES //
        /*
         * Necesitas la gráfica de velocidad del encoder del driveMotor
         * 
         * PASO 1:
         * 1. PID en 0
         * 2. kS dejarla en 0
         * 3. Tunear kV hasta que la velocidad esté en target
         * 
         * PASO 2:
         * 4. Ya no mueves el feedforward
         * 5. Poner la kP lo más grande que pueda sin que se pase del target
         * 6. Poner la kD lo más alto que pueda, sin que empiece a dar picos extraños,
         * que quede smooth
         */

        // FEEDBACK //
        public static final double SWERVE_DRIVE_kP = 0.0;
        public static final double SWERVE_DRIVE_kI = 0.0;
        public static final double SWERVE_DRIVE_kD = 0.0;

        // FEEDFORWARD //
        public static final double SWERVE_DRIVE_kS = 0;
        public static final double SWERVE_DRIVE_kV = 2.0;

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double SWERVE_DRIVE_VELOCITY_SAMPLE_RATE_MS = 100.0;

        // Use this constants to convert from encoder position to meters
        // encoder position * this constant = meters
        public static final double SWERVE_DRIVE_METERS_PER_REVOLUTION = (Math.PI * SWERVE_WHEEL_DIAMETER)
                        / (SWERVE_DRIVE_GEAR_RATIO);

        // Use this constant to convert from motor velocity to meters per second
        // encoder velocity * this constant = meters/second
        public static final double SWERVE_DRIVE_METERS_PER_SEC_PER_VELOCITY_UNITS = SWERVE_DRIVE_METERS_PER_REVOLUTION;

        public static final double SWERVE_VELOCITY_IN_METERS_PER_SEC = (10.0 / 2048.0)
                        * (SWERVE_DRIVE_METERS_PER_SEC_PER_VELOCITY_UNITS);

        public static final double SWERVE_DRIVE_CLEANUP_MODE_SPEED_METERS_PER_SECOND = SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND
                        * 0.8;

        // Arbitrary to make controlling the swerve easier in teleop
        /*
         * public static final double SWERVE_DRIVE_TELEOP_MAX_SPEED_METERS_PER_SECOND =
         * SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND
         * 0.8;
         */

        /////////// DIRECTION MOTOR ///////////

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double SWERVE_DIRECTION_PULSES_PER_REVOLUTION = 1.0;
        public static final double SWERVE_DIRECTION_VELOCITY_SAMPLE_RATE_MS = 100.0;

        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. (Driven / Driver)
        public static final double SWERVE_DIRECTION_GEAR_RATIO = 150.0 / 7.0;

        // Use this constants to convert from encoder position to meters
        // encoder position * this constant = radians
        public static final double SWERVE_DIRECTION_RADIANS_PER_PULSE = Math.PI * 2
                        / (SWERVE_DIRECTION_PULSES_PER_REVOLUTION * SWERVE_DIRECTION_GEAR_RATIO);

        // Use this constant to convert from motor velocity to meters per second
        // encoder velocity * this constant = radians/second
        public static final double SWERVE_DIRECTION_RADIANS_PER_SEC_PER_VELOCITY_UNITS = (1000
                        * SWERVE_DIRECTION_RADIANS_PER_PULSE)
                        / SWERVE_DIRECTION_VELOCITY_SAMPLE_RATE_MS;

        /*
         * public static final double
         * SWERVE_DIRECTION_TELEOP_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI *
         * 0.75;
         */

        //// DIRECTION PID ////

        /// PROFILED PID CONTROLLER FOR SWERVE DIRECTION ///

        // CONSTRAINTS //
        public static final double SWERVE_DIRECTION_MAX_VELOCITY = 40;
        public static final double SWERVE_DIRECTION_MAX_ACCELERATION = 40;

        // HOW TO GET THE VALUES //  //TODO: cambiar esto
        /*
         * Necesitas las graficas: a) Gráfica del angúlo
         * b) Gráfica de velocidad del encoder
         * c) Gráfica de aceleración
         * d) Setpoint del ángulo
         * e) Setpoint de la velocidad
         * f) Setpoint de aceleración
         * 
         * PASO 1:
         * 1. PID en 0
         * 2. kS dejarla en 0
         * 3. Tunear kV hasta que la velocidad esté en target
         * 
         * PASO 2:
         * 4. Ya no mueves el feedforward
         * 5. Poner la kP lo más grande que pueda sin que se pase del target
         * 6. Poner la kD lo más alto que pueda, sin que empiece a dar picos extraños,
         * que quede smooth
         */
      
        // FEEDBACK //

        public static final double SWERVE_DIRECTION_kP = 12; // 0.128
        public static final double SWERVE_DIRECTION_kI = 0; // 0.01
        public static final double SWERVE_DIRECTION_kD = 0.1; // 0.0128



        public static final PPHolonomicDriveController pathFollowerConfig = new // TODO: PathPlanner Constants
        PPHolonomicDriveController(new PIDConstants(0.9, 0, 0.000025),
                        new PIDConstants(2.0, 0, 0));

        //// SpeedReduction constants

        public static final double SWERVE_TURN_BRAKE_DISTANCE = 32; // 32.0;
        public static final double SWERVE_TURN_ARRIVE_OFFSET = 3; // 3.0;

        //// VISION

        public static final double YAW_CORRECTION = 0.15;
        public static final double YAW_OFFSET = 5.72;

        public static final double DISTANCE_CORRECTION = 0.5;

        //////////////////////// DRIVERS ////////////////////////

        public static final HumanDrivers CURRENT_PILOT = HumanDrivers.Joakin;
        public static final HumanDrivers CURRENT_COPILOT = HumanDrivers.Joakin;

        ////////////////////////////// Pathfinding ////////////////////////}

        public static final double PATHFINDING_MAX_LINEAR_SPEED = 1;
        public static final double PATHFINDING_MAX_LINEAR_ACCELERATION = 1.5;
        public static final double PATHFINDING_MAX_ANGULAR_SPEED = Math.PI / 2;
        public static final double PATHFINDING_MAX_ANGULAR_ANGULAR_ACCELERATION = Math.PI;

        // Reef positions for pathfinding, in meteres, measured as blue alliance
        // (automatically mirrored).

        // Back (closer to the driver station)
        public static final Pose2d REEF_BL = new Pose2d(3.695, 5.439, Rotation2d.fromDegrees(-60));
        public static final Pose2d REEF_BC = new Pose2d(2.963, 4.015, Rotation2d.fromDegrees(0));
        public static final Pose2d REEF_BR = new Pose2d(3.704, 2.668, Rotation2d.fromDegrees(60));
        // Front (opposite to the driver station)
        public static final Pose2d REEF_FR = new Pose2d(5.282, 2.620, Rotation2d.fromDegrees(120));
        public static final Pose2d REEF_FC = new Pose2d(5.975, 3.987, Rotation2d.fromDegrees(180));
        public static final Pose2d REEF_FL = new Pose2d(5.253, 5.439, Rotation2d.fromDegrees(-120));

}
