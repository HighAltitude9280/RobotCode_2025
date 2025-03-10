// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder.TriFunction;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.resources.Human_Drivers.HumanDrivers;

/** Add your docs here. */
public class HighAltitudeConstants {

        /**
         * Enum used to indicate the height to grab/leave game pieces.
         * Bottom corresponds to L1 for coral or processor for algae.
         * Top corresponds to L4 for coral or net for algae.
         */
        public enum REEF_HEIGHT {
                BOTTOM(0), L2(1), L3(2), TOP(3);

                private int id;

                public int getID() {
                        return id;
                }

                private REEF_HEIGHT(int id) {
                        this.id = id;
                }
        }

        ////////////////////////// LIFT //////////////////////////
        public static final double LIFT_UP_SPEED = 0.2;
        public static final double LIFT_DOWN_SPEED = -0.1;

        public static final double LIFT_UP_CONTROL_ADDED_VALUE = 0.03;
        public static final double LIFT_DOWN_CONTROL_ADDED_VALUE = -0.03;

        /*
         * Necesitas la gráfica de velocidad del encoder del Lift
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
        public static final double LIFT_kS = 0.0; // 0.025
        public static final double LIFT_kA = 0.0;
        public static final double LIFT_kG = 0.0625; // 0.055
        public static final double LIFT_kV = 0.1625; // 0.5

        public static final double LIFT_kP = 3.7; // 1.5 //0.5
        public static final double LIFT_kI = 0.0;
        public static final double LIFT_kD = 0.0; // 0.1 //0.25

        public static final double LIFT_MAX_VELOCITY = 0.65; // en m/s //0.65
        public static final double LIFT_MAX_ACCELERATION = 1.3; // en m/s^2 // 1.5

        public static final double LIFT_ARRIVE_OFFSET = 0.025; // 0.025
                                                               // NO DEJAR EN CERO EL OFFSET

        public static final double LIFT_REV_PER_NEO_PULSE = 1.0;
        public static final double LIFT_RATIO = 14.0 / 70.0;
        public static final double LIFT_SPROCKET_REVS_PER_PULSE = LIFT_REV_PER_NEO_PULSE * LIFT_RATIO;
        public static final double LIFT_INCHES_PER_SPROCKET_REV = 4.5;
        public static final double LIFT_METERS_PER_PULSE = LIFT_INCHES_PER_SPROCKET_REV
                        * LIFT_SPROCKET_REVS_PER_PULSE * 0.0254;

        // In the same order as the enum: L1, L2, L3, L4
        public static final double[] LIFT_CORAL_POSITIONS = { 0.0, 0.16, 0.35, 0.7 }; // TODO: tunear esto en
                                                                                      // competencia
        // In the same order as the enum: Processor, L2, L3, Net
        public static final double[] LIFT_ALGAE_POSITIONS = { 0.1, 0.28, 0.47, 0.75 };

        public static final double LIFT_ALGAE_INTAKE_POSITION = 0.2;

        public static final double LIFT_TRANSITION_POSITION = 0.47;

        public static final double LIFT_MAX_POWER = 4;

        ////////////////////////// GRIPPER //////////////////////////
        public static final double GRIPPER_OUT_SPEED = 0.2;
        public static final double GRIPPER_IN_SPEED = -0.4;
        public static final double GRIPPER_INTAKE_SPEED = -0.2;
        public static final double GRIPPER_HOLD_SPEED = 0.1;

        ////////////////////////// WRIST //////////////////////////
        // TODO: Wrist Constants
        public static final double WRIST_UP_SPEED = 0.1;
        public static final double WRIST_DOWN_SPEED = -0.1;

        public static final double WRIST_UP_CONTROL_ADDED_VALUE = 10;
        public static final double WRIST_DOWN_CONTROL_ADDED_VALUE = -10;

        public static final double WRIST_DRIVE_SPEED = 0.15;

        public static final double WRIST_kP = 0.1;
        public static final double WRIST_kI = 0.0;
        public static final double WRIST_kD = 0.001;

        public static final double WRIST_RATIO = (1.0 * 12.0) / (15.0 * 32.0);
        public static final double WRIST_NEO_ENCODER_UNITS_PER_REV = 1.0;
        public static final double WRIST_NEO_ENCODER_UNITS_PER_WRIST_REV = WRIST_RATIO
                        / WRIST_NEO_ENCODER_UNITS_PER_REV;
        public static final double WRIST_ARRIVE_OFFSET = 2.0;
        public static final double WRIST_DEGREES_PER_PULSE = 360 * WRIST_NEO_ENCODER_UNITS_PER_WRIST_REV;

        public static final double WRIST_ZERO_ANGLE = 0.0;

        // In the same order as the enum: L1, L2, L3, L4
        public static final double[] WRIST_CORAL_POSITIONS = { 0, 0, 0, 32.5 };
        // In the same order as the enum: Processor, L2, L3, Net
        // public static final double[] WRIST_ALGAE_POSITIONS = { 145, 145, 145, 145 };
        public static final double[] WRIST_ALGAE_POSITIONS = { 145, 145, 145, 115 };

        public static final double WRIST_ALGAE_INTAKE_POSITION = 0;

        // After grabbing algae, wrist should return to this position to keep in inside
        // FP.
        public static final double WRIST_ALGAE_SAFE_POSITION = 0;
        public static final double WRIST_ALGAE_POSITION = 145;

        ////////////////////////// SWERVE //////////////////////////

        public static final double MAX_VOLTAGE = 11.5;
        // ponlo a 3 para pruebas

        /// CONSTANTS FOR MK4i L4 Config DRIVE MOTOR ///
        // In meters
        public static final double SWERVE_WHEEL_DIAMETER = 4.0 * 0.0254;
        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. (Driven / Driver) //pinion
        public static final double SWERVE_DRIVE_GEAR_RATIO = (50.0 * 16.0 * 45.0) / (16.0 * 28.0 * 15.0);

        // ft/s //ft -> in //im -> m
        public static final double SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND = 19.5 * 12 * 0.0254;

        public static final double SWERVE_DIRECTION_MAX_ANGULAR_SPEED_RADS_PER_SECOND = 5.0;

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
        public static final double SWERVE_MAX_ACCELERATION_UNITS_PER_SECOND = 15.0; // 20.0
        public static final double SWERVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 10.0;

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
        public static final double SWERVE_DRIVE_kP = 0.125;
        public static final double SWERVE_DRIVE_kI = 0.0;
        public static final double SWERVE_DRIVE_kD = 0.0;

        // FEEDFORWARD //
        public static final double SWERVE_DRIVE_kS = 0;
        public static final double SWERVE_DRIVE_kV = 2.05;

        // The reported encoder position after one revolution, check encoder
        // specifications.
        public static final double SWERVE_DRIVE_VELOCITY_SAMPLE_RATE_MS = 100.0;

        // Use this constants to convert from encoder position to meters
        // encoder position * this constant = meters
        public static final double SWERVE_DRIVE_METERS_PER_REV = (Math.PI * SWERVE_WHEEL_DIAMETER)
                        / (SWERVE_DRIVE_GEAR_RATIO);

        // encoder position * this constant = meters
        public static final double SWERVE_DRIVE_PER_VELOCITY_UNITS = SWERVE_DRIVE_METERS_PER_REV;

        public static final double SWERVE_DRIVE_PRECISION_MODE_SPEED_METERS_PER_SECOND = SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND
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
        public static final double SWERVE_DIRECTION_MAX_VELOCITY = 6;
        public static final double SWERVE_DIRECTION_MAX_ACCELERATION = 15;

        // HOW TO GET THE VALUES //
        /*
         * Necesitas las graficas:
         * a) Gráfica del ángulo del CANCoder
         * d) Setpoint del ángulo del CANCoder
         * 
         * PASO 1:
         * 1. PID en 0
         * 2. Poner la kP lo más grande que pueda sin que se pase del target
         * 
         * PASO 2:
         * 3. Poner la kD lo más alto que pueda, sin que empiece a dar picos extraños,
         * que quede smooth
         */

        // FEEDBACK //

        public static final double SWERVE_DIRECTION_kP = 5; // 12
        public static final double SWERVE_DIRECTION_kI = 0; // 0.01
        public static final double SWERVE_DIRECTION_kD = 0.075; // 0.0128

        public static final PPHolonomicDriveController pathFollowerConfig = new // TODO: PathPlanner Constants
        PPHolonomicDriveController(new PIDConstants(0.9, 0, 0.000025),
                        new PIDConstants(2.0, 0, 0));

        //// SpeedReduction constants

        public static final double SWERVE_TURN_BRAKE_DISTANCE = 32; // 32.0;

        public static final double SWERVE_TURN_ARRIVE_OFFSET = 3; // 3.0;

        public static final double SWERVE_TURN_WHEELS_RADIANS_ARRIVE_OFFSET = 3 * (Math.PI / 180);
        // degrees to radians

        //// Dynamic acceleration limiter

        public static final boolean ENABLE_DYNAMIC_ACCELERATION_LIMITER = true;

        // The height threshold above which the drivetrain acceleration will be limited.
        public static final double DAL_MIN_HEIGHT = 0.8;

        // Factor used to reduce acceleration as height increases beyond the minimum
        // threshold.
        // The acceleration percentage is adjusted based on (1 - (height -
        // DAL_MIN_HEIGHT) * DAL_HEIGHT_MULTIPLIER)
        // So the higher this constant, the higher the DAL effect.
        public static final double DAL_HEIGHT_MULTIPLIER = 0.5;

        public static final double SWERVE_DISTANCE_kP = 4.0;
        public static final double SWERVE_DISTANCE_kD = 0;
        public static final double SWERVE_DRIVE_DISTANCE_ARRIVE_OFFSET = 0.0254;

        //////////////////////////// VISION //////////////////////////////

        // TODO: CONFIGURE THESE CONSTANTS

        public static final double VISION_YAW_OFFSET_TARGET_LEFT = 24.89;
        public static final double VISION_YAW_OFFSET_TARGET_RIGHT = -24.89;

        public static final double VISION_AREA_TARGET = 7.56;

        // Speed reduction constants for aligning with apriltags.
        public static final double VISION_TURN_ARRIVE_OFFSET = 0.1;// 3;
        public static final double VISION_TURN_kP = 0;
        public static final double VISION_TURN_kI = 0;
        public static final double VISION_TURN_kD = 0;


        public static final double VISION_STRAFE_kP = 0;
        public static final double VISION_STRAFE_kI = 0;
        public static final double VISION_STRAFE_kD = 0;
        public static final double VISION_STRAFE_ARRIVE_OFFSET = 0.1;


        public static final double VISION_SPEED_kP = 0;
        public static final double VISION_SPEED_kI = 0;
        public static final double VISION_SPEED_kD = 0;
        public static final double VISION_SPEED_ARRIVE_OFFSET = 0.05;// 0.05;

        public static final double VISION_TURN_MAX_POWER = 0.1;
        public static final double VISION_STRAFE_MAX_POWER = 0.0; // 0.1
        public static final double VISION_SPEED_MAX_POWER = 0.0; // 0.1

        public static final double SWERVE_METERS_DISTANCE_ALIGN_TO_REEF = 0.5;

        //////////////////////// DRIVERS ////////////////////////

        public static final HumanDrivers CURRENT_PILOT = HumanDrivers.Joakin;
        public static final HumanDrivers CURRENT_COPILOT = HumanDrivers.Carlos;

        //////////////////////// CLIMBER ////////////////////////
        public static final double CLIMBER_FOLD_SPEED = 0.1;
        public static final double CLIMBER_EXTEND_SPEED = -0.1;

        ////////////////////////////// Pathfinding ////////////////////////}

        public static final double PATHFINDING_MAX_LINEAR_SPEED = 1;
        public static final double PATHFINDING_MAX_LINEAR_ACCELERATION = 1.5;
        public static final double PATHFINDING_MAX_ANGULAR_SPEED = Math.PI / 2;
        public static final double PATHFINDING_MAX_ANGULAR_ANGULAR_ACCELERATION = Math.PI;

        // Reef positions for pathfinding, in meteres

        // Note that this array should be in the same order as the enum
        // i.e. PATHFINDING_REEF_POS[REEF_POSITION.BC] should correspond to BC.
        public static final Pose2d[] PATHFINDING_BLUE_REEF_POS = {
                        // Back (closer to the driver station)
                        new Pose2d(3.695, 5.439, Rotation2d.fromDegrees(-60)),
                        new Pose2d(2.963, 4.015, Rotation2d.fromDegrees(0)),
                        new Pose2d(3.704, 2.668, Rotation2d.fromDegrees(60)),
                        // Front (opposite to the driver station)
                        new Pose2d(5.282, 2.620, Rotation2d.fromDegrees(120)),
                        new Pose2d(5.975, 3.987, Rotation2d.fromDegrees(180)),
                        new Pose2d(5.253, 5.439, Rotation2d.fromDegrees(-120))
        };
        public static final Pose2d[] PATHFINDING_RED_REEF_POS = {
                        // Back (closer to the driver station)
                        new Pose2d(13.846, 2.668, Rotation2d.fromDegrees(120)),
                        new Pose2d(14.587, 4.015, Rotation2d.fromDegrees(180)),
                        new Pose2d(13.855, 5.439, Rotation2d.fromDegrees(-120)),
                        // Front (opposite to the driver station)
                        new Pose2d(12.297, 5.439, Rotation2d.fromDegrees(-60)),
                        new Pose2d(11.5, 3.987, Rotation2d.fromDegrees(0)),
                        new Pose2d(12.268, 2.620, Rotation2d.fromDegrees(60))
        };

        public static final int[] BLUE_APRILTAG_IDS = { 19, 18, 17, 22, 21, 20 };
        public static final int[] RED_APRILTAG_IDS = { 6, 7, 8, 9, 10, 11 };

        public enum REEF_POSITION {
                BL(0), BC(1), BR(2), FR(3), FC(4), FL(5);

                int id;

                private REEF_POSITION(int id) {
                        this.id = id;
                }

                public int getID() {
                        return id;
                }
        }

        public enum REEF_SIDE {
                LEFT(REEF_POSITION.BL, REEF_POSITION.FL),
                CENTER(REEF_POSITION.BC, REEF_POSITION.FC),
                RIGHT(REEF_POSITION.BR, REEF_POSITION.FR);

                private REEF_POSITION back, front;

                REEF_SIDE(REEF_POSITION back, REEF_POSITION front) {
                        this.back = back;
                        this.front = front;
                }

                public REEF_POSITION getPosition(boolean front) {
                        return front ? this.front : this.back;
                }
        }

}
