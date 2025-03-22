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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
        public static final double LIFT_kS = 0.097455; // 0.0
        public static final double LIFT_kA = 0.0; // 1.2565
        public static final double LIFT_kG = 0.72598; // 0.0624 //0.6
        public static final double LIFT_kV = 5.1535; // 0.1625

        public static final double LIFT_kP = 1.1653; // 3.7 //1.1653
        public static final double LIFT_kI = 0.0;
        public static final double LIFT_kD = 0.921640625; // 0.1 //5.8985

        public static final double LIFT_MAX_VELOCITY = 0.65; // en m/s //0.65
        public static final double LIFT_MAX_ACCELERATION = 1.3; // en m/s^2 // 1.3

        public static final double LIFT_ARRIVE_OFFSET = 0.025; // 0.025
                                                               // NO DEJAR EN CERO EL OFFSET

        public static final double LIFT_MAX_POWER = 7; // 4

        public static final double LIFT_REV_PER_NEO_PULSE = 1.0;
        public static final double LIFT_RATIO = 14.0 / 70.0;
        public static final double LIFT_SPROCKET_REVS_PER_PULSE = LIFT_REV_PER_NEO_PULSE * LIFT_RATIO;
        public static final double LIFT_INCHES_PER_SPROCKET_REV = 4.5;
        public static final double LIFT_METERS_PER_PULSE = LIFT_INCHES_PER_SPROCKET_REV
                        * LIFT_SPROCKET_REVS_PER_PULSE * 0.0254;

        // In the same order as the enum: L1, L2, L3, L4
        public static final double[] LIFT_CORAL_POSITIONS = { 0.0, 0.14, 0.35, 0.73 }; // TODO: tunear esto en
                                                                                       // competencia
        // In the same order as the enum: Processor, L2, L3, Net
        public static final double[] LIFT_ALGAE_POSITIONS = { 0.1, 0.28, 0.47, 0.75 };

        public static final double LIFT_ALGAE_INTAKE_POSITION = 0.2;

        public static final double LIFT_TRANSITION_POSITION = 0.47;

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
        public static final double[] WRIST_CORAL_POSITIONS = { 0, 0, 0, 40 };
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
        public static final double SWERVE_TRACK_WIDTH = 21 * 0.0254; // este es de llanta a llanta
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
        PPHolonomicDriveController(new PIDConstants(0.125, 0, 0.00),
                        new PIDConstants(0.875, 0, 0.000));

        //// SpeedReduction constants

        public static final double SWERVE_TURN_BRAKE_DISTANCE = 32; // 32.0;

        public static final double SWERVE_TURN_ARRIVE_OFFSET = 3; // 3.0;

        public static final double SWERVE_TURN_WHEELS_RADIANS_ARRIVE_OFFSET = 4 * (Math.PI / 180);
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

        public static final double SWERVE_DISTANCE_kP = 3.5;
        public static final double SWERVE_DISTANCE_kD = 0;
        public static final double SWERVE_DRIVE_DISTANCE_ARRIVE_OFFSET = 0.015;

        //////////////////////////// VISION //////////////////////////////

        // Keep the order of the cameras consistent across the arrays.
        public static final String[] CAMERA_NAMES = { "ArducamFront", "ArducamBack", "Limelight3",
                        "Limelight2Pi" };
        public static final Transform3d[] CAMERA_POSITIONS = {
                        // ArducamFront
                        new Transform3d(new Translation3d(0.212571011152, 0.262729491994, 0.227090042532),
                                        new Rotation3d(Math.toRadians(0f), Math.toRadians(55),
                                                        Math.toRadians(145.02940176))),
                        // ArducamBack
                        new Transform3d(new Translation3d(-0.261676313324, -0.272904897166, 0.215),
                                        new Rotation3d(0f, Math.toRadians(-61.875), Math.toRadians(-149.52786828))),
                        // Limelight3
                        new Transform3d(new Translation3d(0.20908042075, -0.271998758978, 0.19710410922),
                                        new Rotation3d(Math.toRadians(0), Math.toRadians(70), Math.toRadians(30))),
                        // Limelight2Pi
                        new Transform3d(new Translation3d(-0.037639273664, 0.0235341, 0.989082539932),
                                        new Rotation3d(Math.toRadians(0f), Math.toRadians(-30), Math.toRadians(180)))
        };

        // The indexes in the previous arrays of the cameras that will be used for
        // alignment.
        public static final int[] ALIGNMENT_CAMERAS = { 2, 0 };

        public static final double VISION_POSE_ESTIMATOR_MAX_DISTANCE = 3.5;
        public static final double VISION_POSE_ESTIMATOR_MAX_AMBIGUITY = 0.15;

        // TODO: CONFIGURE THESE CONSTANTS

        public static final double VISION_YAW_OFFSET_TARGET_LEFT = 17.82;
        public static final double VISION_YAW_OFFSET_TARGET_RIGHT = -25.31;

        public static final double VISION_AREA_TARGET = 7.42;

        // Speed reduction constants for aligning with apriltags.
        public static final double VISION_TURN_kP = 0.05;
        public static final double VISION_TURN_kI = 0;
        public static final double VISION_TURN_kD = 0;
        public static final double VISION_TURN_ARRIVE_OFFSET = 2;// 3;

        public static final double VISION_STRAFE_kP = 0.0085;
        public static final double VISION_STRAFE_kI = 0;
        public static final double VISION_STRAFE_kD = 0;
        public static final double VISION_STRAFE_ARRIVE_OFFSET = 0.8;

        public static final double VISION_SPEED_kP = 0.05;
        public static final double VISION_SPEED_kI = 0;
        public static final double VISION_SPEED_kD = 0;
        public static final double VISION_SPEED_ARRIVE_OFFSET = 0.4;// 0.8;

        public static final double VISION_TURN_MAX_POWER = 0.3; // 0.3
        public static final double VISION_STRAFE_MAX_POWER = 0.05; // 0.3
        public static final double VISION_SPEED_MAX_POWER = 0.3; // 0.3

        public static final double SWERVE_METERS_DISTANCE_ALIGN_TO_REEF = 0.33;

        // Pos alignment

        public static final double VISION_POSE_kP = 0;
        public static final double VISION_POSE_kI = 0;
        public static final double VISION_POSE_kD = 0;
        // In meters
        public static final double VISION_POSE_ARRIVE_OFFSET = 0.01;

        public static final double VISION_POSE_TURN_kP = 0;
        public static final double VISION_POSE_TURN_kI = 0;
        public static final double VISION_POSE_TURN_kD = 0;
        // In radians
        public static final double VISION_POSE_TURN_ARRIVE_OFFSET = Math.toRadians(2);

        //////////////////////// DRIVERS ////////////////////////

        public static final HumanDrivers CURRENT_PILOT = HumanDrivers.Joakin;
        public static final HumanDrivers CURRENT_COPILOT = HumanDrivers.Carlos;

        //////////////////////// CLIMBER ////////////////////////
        public static final double CLIMBER_FOLD_SPEED = 0.2;
        public static final double CLIMBER_EXTEND_SPEED = -0.2;

        ////////////////////////////// Pathfinding ////////////////////////}

        public static final double PATHFINDING_MAX_LINEAR_SPEED = 1;
        public static final double PATHFINDING_MAX_LINEAR_ACCELERATION = 1.5;
        public static final double PATHFINDING_MAX_ANGULAR_SPEED = Math.PI / 2;
        public static final double PATHFINDING_MAX_ANGULAR_ANGULAR_ACCELERATION = Math.PI;

        // Reef positions for pathfinding, in meteres

        // Note that this array should be in the same order as the enum
        // i.e. PATHFINDING_REEF_POS[REEF_POSITION.BC] should correspond to BC.
        public static final Pose2d[] PATHFINDING_BLUE_REEF_POS = {

                        new Pose2d(2.963, 4.015, Rotation2d.fromDegrees(0)), // BC
                        new Pose2d(3.704, 2.668, Rotation2d.fromDegrees(60)), // BR
                        new Pose2d(5.282, 2.620, Rotation2d.fromDegrees(120)), // FR
                        new Pose2d(5.975, 3.987, Rotation2d.fromDegrees(180)), // FC
                        new Pose2d(5.253, 5.439, Rotation2d.fromDegrees(-120)), // FL
                        new Pose2d(3.695, 5.439, Rotation2d.fromDegrees(-60))// BL
        };
        public static final Pose2d[] PATHFINDING_RED_REEF_POS = {
                        new Pose2d(14.587, 4.015, Rotation2d.fromDegrees(180)), // BC
                        new Pose2d(13.855, 5.439, Rotation2d.fromDegrees(-120)), // BR
                        new Pose2d(12.297, 5.439, Rotation2d.fromDegrees(-60)), // FR
                        new Pose2d(11.5, 3.987, Rotation2d.fromDegrees(0)), // FC
                        new Pose2d(12.268, 2.620, Rotation2d.fromDegrees(60)), // FL
                        new Pose2d(13.846, 2.668, Rotation2d.fromDegrees(120))// BL
        };

        public static final int[] BLUE_APRILTAG_IDS = { 18, 17, 22, 21, 20, 19 };
        public static final int[] RED_APRILTAG_IDS = { 7, 8, 9, 10, 11, 6 };

        public static final Pose2d PATHFINDING_LEFT_BLUE_FEEDER = new Pose2d(1.38, 7.09,
                        Rotation2d.fromDegrees(-54.2));
        public static final Pose2d PATHFINDING_RIGHT_BLUE_FEEDER = new Pose2d(1.56, 0.80,
                        Rotation2d.fromDegrees(54.2));

        public static final Pose2d PATHFINDING_LEFT_RED_FEEDER = new Pose2d(15.98, 0.77, Rotation2d.fromDegrees(125));
        public static final Pose2d PATHFINDING_RIGHT_RED_FEEDER = new Pose2d(15.92, 7.301,
                        Rotation2d.fromDegrees(-125));

        // Order as in game manual A, B, C,...
        public static final Pose2d PATHFINDING_BLUE_BRANCHES[] = {
                        new Pose2d(3.18, 4.2, Rotation2d.fromDegrees(0)), // A
                        new Pose2d(3.18, 3.85, Rotation2d.fromDegrees(0)), // B
                        new Pose2d(3.7, 3, Rotation2d.fromDegrees(60)), // C
                        new Pose2d(3.98, 2.83, Rotation2d.fromDegrees(60)), // D
                        new Pose2d(4.98, 2.86, Rotation2d.fromDegrees(120)), // E
                        new Pose2d(5.25, 2.99, Rotation2d.fromDegrees(120)), // F
                        new Pose2d(5.78, 3.86, Rotation2d.fromDegrees(180)), // G
                        new Pose2d(5.78, 4.17, Rotation2d.fromDegrees(180)), // H
                        new Pose2d(5.26, 5.05, Rotation2d.fromDegrees(-120)), // I
                        new Pose2d(4.99, 5.20, Rotation2d.fromDegrees(-120)), // J
                        new Pose2d(3.98, 5.25, Rotation2d.fromDegrees(-60)), // K
                        new Pose2d(3.70, 5.04, Rotation2d.fromDegrees(-60)) // L
        };
        public static final Pose2d PATHFINDING_RED_BRANCHES[] = {
                        new Pose2d(14.35, 3.86, Rotation2d.fromDegrees(180)), // A
                        new Pose2d(14.35, 4.18, Rotation2d.fromDegrees(180)), // B
                        new Pose2d(13.85, 5.03, Rotation2d.fromDegrees(-120)), // C
                        new Pose2d(13.56, 5.22, Rotation2d.fromDegrees(-120)), // D
                        new Pose2d(12.54, 5.20, Rotation2d.fromDegrees(-60)), // E
                        new Pose2d(12.28, 5.05, Rotation2d.fromDegrees(-60)), // F
                        new Pose2d(11.77, 4.19, Rotation2d.fromDegrees(0)), // G
                        new Pose2d(11.77, 3.85, Rotation2d.fromDegrees(0)), // H
                        new Pose2d(12.27, 3.00, Rotation2d.fromDegrees(60)), // I
                        new Pose2d(12.55, 2.82, Rotation2d.fromDegrees(60)), // J
                        new Pose2d(13.55, 2.81, Rotation2d.fromDegrees(120)), // K
                        new Pose2d(13.85, 2.99, Rotation2d.fromDegrees(120)) // L
        };

        public enum REEF_POSITION {
                BC(0), BR(1), FR(2), FC(3), FL(4), BL(5);

                int id;

                private REEF_POSITION(int id) {
                        this.id = id;
                }

                public int getID() {
                        return id;
                }

                public int getBranchID(boolean left) {
                        return left ? 2 * id : 2 * id + 1;
                }
        }

        public static REEF_POSITION[] REEF_POSITIONS = { REEF_POSITION.BC, REEF_POSITION.BR, REEF_POSITION.FR,
                        REEF_POSITION.FC, REEF_POSITION.FL, REEF_POSITION.BL };

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
