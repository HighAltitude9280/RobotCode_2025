// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Robot.GameMode;
import frc.robot.resources.Human_Drivers.HumanDrivers;

/** Add your docs here. */
public class HighAltitudeConstants {

        public static final double caralho_var = 0.15;

        //////////////////////// DRIVERS ////////////////////////

        public static final HumanDrivers CURRENT_PILOT = HumanDrivers.OneDriver;

        public static final HumanDrivers CURRENT_COPILOT = HumanDrivers.Pato;


        /**
         * Enum used to indicate the height to grab/leave game pieces. Bottom corresponds to L1 for
         * coral or processor for algae. Top corresponds to L4 for coral or net for algae.
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

        public static final double LIFT_UP_CONTROL_ADDED_VALUE = 0.05; // 0.03
        public static final double LIFT_DOWN_CONTROL_ADDED_VALUE = -0.05; // 0.03

        /*
         * Necesitas la gráfica de velocidad del encoder del Lift
         * 
         * PASO 1: 1. PID en 0 2. kS dejarla en 0 3. Tunear kV hasta que la velocidad esté en target
         * 
         * PASO 2: 4. Ya no mueves el feedforward 5. Poner la kP lo más grande que pueda sin que se
         * pase del target 6. Poner la kD lo más alto que pueda, sin que empiece a dar picos
         * extraños, que quede smooth
         */
        public static final double LIFT_kS = 0.065;// 0.057012; // 0.03148;
        public static final double LIFT_kA = 0.34997; // 0.28027;
        public static final double LIFT_kG = 0.4602675;// 0.49398; // 0.44903;
        public static final double LIFT_kV = 5.0644; // 5.0705;

        public static final double LIFT_kP = 3.988; // 3.0959
        public static final double LIFT_kI = 0;
        public static final double LIFT_kD = 0.23765; // 0.2683

        public static final double LIFT_MAX_VELOCITY = 1.5;// 1.25 // en m/s //0.65
        public static final double LIFT_MAX_ACCELERATION = 3.0; // en m/s^2 // 1.3

        public static final double LIFT_ARRIVE_OFFSET = 0.02; // 0.025
                                                              // NO DEJAR EN CERO EL OFFSET

        public static final double LIFT_MAX_POWER = 7.5; // 4
        public static final double LIFT_MAX_POWER_GOING_DOWN = 7.0;

        public static final double LIFT_REV_PER_NEO_PULSE = 1.0;
        public static final double LIFT_RATIO = 14.0 / 70.0;
        public static final double LIFT_SPROCKET_REVS_PER_PULSE =
                        LIFT_REV_PER_NEO_PULSE * LIFT_RATIO;
        public static final double LIFT_INCHES_PER_SPROCKET_REV = 4.5;
        public static final double LIFT_METERS_PER_PULSE =
                        LIFT_INCHES_PER_SPROCKET_REV * LIFT_SPROCKET_REVS_PER_PULSE * 0.0254;

        // In the same order as the enum: L1, L2, L3, L4
        public static final double[] LIFT_CORAL_POSITIONS = {0.0, 0.32, 0.51, 0.71}; // TODO: tunear
                                                                                     // esto en
                                                                                     // competencia
        // In the same order as the enum: Processor, L2, L3, Net
        public static final double[] LIFT_ALGAE_POSITIONS = {0.0, 0.28, 0.47, 0.73};

        public static final double LIFT_ALGAE_INTAKE_POSITION = 0.2;

        public static final double LIFT_TRANSITION_POSITION = 0.47;

        ////////////////////////// GRIPPER //////////////////////////
        public static final double GRIPPER_OUT_SPEED = 0.35;
        public static final double GRIPPER_IN_SPEED = -0.5625;
        public static final double GRIPPER_INTAKE_SPEED = -0.3; // -0.2
        public static final double GRIPPER_HOLD_SPEED = 0.2;

        ////////////////////////// WRIST //////////////////////////
        // TODO: Wrist Constants
        public static final double WRIST_UP_SPEED = 0.1;
        public static final double WRIST_DOWN_SPEED = -0.1;

        public static final double WRIST_UP_CONTROL_ADDED_VALUE = 10;
        public static final double WRIST_DOWN_CONTROL_ADDED_VALUE = -10;

        public static final double WRIST_DRIVE_SPEED = 0.225;

        public static final double WRIST_kP = 0.1; // 0.1
        public static final double WRIST_kI = 0.0; // 0.0
        public static final double WRIST_kD = 0.0; // 0.001

        public static final double WRIST_ARRIVE_OFFSET = 1.5; // 2.0

        public static final double WRIST_RATIO = (1.0 * 12.0) / (15.0 * 48.0);

        public static final double WRIST_NEO_ENCODER_UNITS_PER_REV = 1.0;
        public static final double WRIST_NEO_ENCODER_UNITS_PER_WRIST_REV =
                        WRIST_RATIO / WRIST_NEO_ENCODER_UNITS_PER_REV;
        public static final double WRIST_DEGREES_PER_PULSE =
                        360 * WRIST_NEO_ENCODER_UNITS_PER_WRIST_REV;

        public static final double WRIST_ZERO_ANGLE = -83;

        // In the same order as the enum: L1, L2, L3, L4
        public static final double[] WRIST_CORAL_POSITIONS = {0, 0, 0, 20}; // 40 in comp
        // In the same order as the enum: Processor, L2, L3, Net
        // public static final double[] WRIST_ALGAE_POSITIONS = { 145, 145, 145, 145 };
        public static final double[] WRIST_ALGAE_POSITIONS = {40, 40, 40, 40};

        public static final double WRIST_ALGAE_INTAKE_POSITION = 0;

        // After grabbing algae, wrist should return to this position to keep in inside
        // FP.
        public static final double WRIST_ALGAE_SAFE_POSITION = 0;
        public static final double WRIST_ALGAE_POSITION = 80;

        ////////////////////////// SWERVE //////////////////////////

        public static final double MAX_VOLTAGE = 11.5;
        // ponlo a 3 para pruebas

        /// CONSTANTS FOR MK4i L4 Config DRIVE MOTOR ///
        // In meters
        public static final double SWERVE_WHEEL_DIAMETER = 4.0 * 0.0254;
        // NEVER, ABSOLUTELY NEVER APPROXIMATE THIS, USE ONLY FRACTIONS WITH WHOLE
        // NUMBERS. (Driven / Driver) //pinion
        public static final double SWERVE_DRIVE_GEAR_RATIO =
                        (50.0 * 16.0 * 45.0) / (16.0 * 28.0 * 15.0);

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
        public static final double SWERVE_ABSOLUTE_ENCODER_RADIANS_PER_PULSE =
                        (2.0 * Math.PI) / SWERVE_ABSOLUTE_ENCODER_PULSES_PER_REVOLUTION;

        /////////// DRIVING MOTOR /////////// TODO: DRIVING MOTOR

        // HOW TO GET THE VALUES //
        /*
         * Necesitas la gráfica de velocidad del encoder del driveMotor
         * 
         * PASO 1: 1. PID en 0 2. kS dejarla en 0 3. Tunear kV hasta que la velocidad esté en target
         * 
         * PASO 2: 4. Ya no mueves el feedforward 5. Poner la kP lo más grande que pueda sin que se
         * pase del target 6. Poner la kD lo más alto que pueda, sin que empiece a dar picos
         * extraños, que quede smooth
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
        public static final double SWERVE_DRIVE_METERS_PER_REV =
                        (Math.PI * SWERVE_WHEEL_DIAMETER) / (SWERVE_DRIVE_GEAR_RATIO);

        // encoder position * this constant = meters
        public static final double SWERVE_DRIVE_PER_VELOCITY_UNITS = SWERVE_DRIVE_METERS_PER_REV;

        public static final double SWERVE_DRIVE_PRECISION_MODE_SPEED_METERS_PER_SECOND =
                        SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND * 0.8;

        // Arbitrary to make controlling the swerve easier in teleop
        /*
         * public static final double SWERVE_DRIVE_TELEOP_MAX_SPEED_METERS_PER_SECOND =
         * SWERVE_DRIVE_MAX_SPEED_METERS_PER_SECOND 0.8;
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
        public static final double SWERVE_DIRECTION_RADIANS_PER_SEC_PER_VELOCITY_UNITS =
                        (1000 * SWERVE_DIRECTION_RADIANS_PER_PULSE)
                                        / SWERVE_DIRECTION_VELOCITY_SAMPLE_RATE_MS;

        /*
         * public static final double SWERVE_DIRECTION_TELEOP_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND =
         * 2 * Math.PI * 0.75;
         */

        //// DIRECTION PID ////

        /// PROFILED PID CONTROLLER FOR SWERVE DIRECTION ///

        // CONSTRAINTS //
        public static final double SWERVE_DIRECTION_MAX_VELOCITY = 6;
        public static final double SWERVE_DIRECTION_MAX_ACCELERATION = 15;

        // HOW TO GET THE VALUES //
        /*
         * Necesitas las graficas: a) Gráfica del ángulo del CANCoder d) Setpoint del ángulo del
         * CANCoder
         * 
         * PASO 1: 1. PID en 0 2. Poner la kP lo más grande que pueda sin que se pase del target
         * 
         * PASO 2: 3. Poner la kD lo más alto que pueda, sin que empiece a dar picos extraños, que
         * quede smooth
         */

        // FEEDBACK //
        public static final double SWERVE_DIRECTION_kP = 5.0;
        public static final double SWERVE_DIRECTION_kI = 0; // 0.01
        public static final double SWERVE_DIRECTION_kD = 0.075; // 0.0128

        public static final PPHolonomicDriveController pathFollowerConfig = new // TODO: PathPlanner
                                                                                // Constants
        PPHolonomicDriveController(new PIDConstants(4, 0, 0.00), new PIDConstants(1.2, 0, 0.000));

        //// SpeedReduction constants

        public static final double SWERVE_TURN_BRAKE_DISTANCE = 32; // 32.0;

        public static final double SWERVE_TURN_ARRIVE_OFFSET = 3; // 3.0;

        public static final double SWERVE_TURN_WHEELS_RADIANS_ARRIVE_OFFSET = 4 * (Math.PI / 180);
        // degrees to radians

        //// Dynamic acceleration limiter

        public static final boolean ENABLE_DYNAMIC_ACCELERATION_LIMITER = true;

        // The height threshold above which the drivetrain acceleration will be limited.
        public static final double DAL_MIN_HEIGHT = 0.2;

        // Factor used to reduce acceleration as height increases beyond the minimum
        // threshold.
        // The acceleration percentage is adjusted based on (1 - (height -
        // DAL_MIN_HEIGHT) * DAL_HEIGHT_MULTIPLIER)
        // So the higher this constant, the higher the DAL effect.
        public static final double DAL_HEIGHT_MULTIPLIER = 0.5;

        public static final double SWERVE_DISTANCE_kP = 3.5;
        public static final double SWERVE_DISTANCE_kD = 0;
        public static final double SWERVE_DRIVE_DISTANCE_ARRIVE_OFFSET = 0.01;

        //////////////////////////// VISION //////////////////////////////

        // Keep the order of the cameras consistent across the arrays.
        public static final String[] CAMERA_NAMES = {"ArducamFront2", "limelight2plus"};
        public static final Transform3d[] CAMERA_POSITIONS = {
                        // ArducamFront2
                        new Transform3d(new Translation3d(0.223774, 0.261112, 0.20917466),
                                        new Rotation3d(Math.toRadians(0f), Math.toRadians(-20),
                                                        Math.toRadians(-24.97059824))),

                        // Limelight2PP
                        new Transform3d(new Translation3d(0.202692, -0.27051, 0.21686527),
                                        new Rotation3d(Math.toRadians(0), Math.toRadians(-20),
                                                        Math.toRadians(30)))};

        // The indexes in the previous arrays of the cameras that will be used for
        // alignment.
        public static final int[] ALIGNMENT_CAMERAS = {0, 1};

        public static final double VISION_POSE_ESTIMATOR_MAX_DISTANCE = 2.5;
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

        // TODO: Pos alignment

        public static final double VISION_POSE_kP = 6.0;
        public static final double VISION_POSE_kI = 0;
        public static final double VISION_POSE_kD = 0.1;
        // In meters
        public static final double VISION_POSE_ARRIVE_OFFSET = 0.1; // 0.01

        public static final double VISION_POSE_TURN_kP = 4.0;
        public static final double VISION_POSE_TURN_kI = 0;
        public static final double VISION_POSE_TURN_kD = 0;
        // In radians
        public static final double VISION_POSE_TURN_ARRIVE_OFFSET = Math.toRadians(3);// 2

        public static final double VISION_POSE_MAX_TURN = 1.0;
        public static final double VISION_POSE_MAX_SPEED = 1.0;

        //////////////////////// ALIGN WITH TARGET POSE ////////////////////////

        /*
         * 
         */
        // Prioridades: confiable (fail-soft) y rápido (reactivo con límites)
        /*
         * ==== LATCHED_POSE_MAX_AGE_SEC (TTL de la última pose válida) ==== Paso 1: arranca en 0.35
         * s (auto: 0.45–0.60 s). Paso 2: si corta con pérdidas cortas (100–150 ms) → súbela +0.05
         * s. Paso 3: si durante la pérdida entra chueco → bájala −0.05 s. Sugerido: teleop
         * 0.30–0.40 s, auto 0.40–0.60 s.
         */

        /*
         * ==== LATCHED_POSE_MAX_TRANSLATION_DRIFT_M (deriva lineal permitida con latched) ==== Paso
         * 1: arranca en 0.07 m. Paso 2: si aborta “antes de tiempo” y aún estabas bien alineado →
         * súbela +0.01–0.03 m (hasta ~0.10 m). Paso 3: si entra chueco durante la pérdida → bájala
         * −0.01–0.02 m (hasta ~0.05–0.06 m). Regla: mecanismo estricto 0.05–0.07 m, tolerante
         * 0.07–0.10 m.
         */

        /*
         * ==== LATCHED_POSE_MAX_HEADING_DRIFT_DEG (deriva angular permitida con latched) ==== Paso
         * 1: arranca en 3.0° (estrecho: 2–2.5°, tolerante: 4–5°). Paso 2: si corta muy pronto y aún
         * “entrabas” → súbela +0.5–1.0°. Paso 3: si sale ladeado al reenganchar → bájala −0.5–1.0°.
         */

        /*
         * ==== COMMAND_TIMEOUT_SEC (corte de seguridad por tiempo) ==== Paso 1: arranca en 1.8 s.
         * Paso 2: si corta justo antes de completar seguido → súbelo +0.2 s (o usa timeout dinámico
         * por distancia). Paso 3: si nunca se acerca al límite → bájalo −0.2 s para ser más
         * estricto.
         */

        /*
         * ==== TAG_DETECTION_LOCK_CYCLES (anti “ping-pong” de AprilTag) ==== Paso 1: arranca en 2
         * ciclos (≈40 ms a 50 Hz). Paso 2: si cambia de rama por frames ruidosos → súbelo a 3. Paso
         * 3: si tarda en decidir objetivo → bájalo a 1 (con más riesgo de ping-pong).
         */
        public static final double LATCHED_POSE_MAX_AGE_SEC = 0.35; // frescura de pose latched
        public static final double LATCHED_POSE_MAX_TRANSLATION_DRIFT_M = 0.07; // deriva lineal
                                                                                // permitida
        public static final double LATCHED_POSE_MAX_HEADING_DRIFT_DEG = 3.0; // deriva angular
                                                                             // permitida
        public static final double COMMAND_TIMEOUT_SEC = 1.8; // timeout duro
        public static final int TAG_DETECTION_LOCK_CYCLES = 2; // detección estable para lock

        ////////////////////////////// Pathfinding ////////////////////////

        public static final double PATHFINDING_MAX_LINEAR_SPEED = 2;
        public static final double PATHFINDING_MAX_LINEAR_ACCELERATION = 8;
        public static final double PATHFINDING_MAX_ANGULAR_SPEED = Math.PI;
        public static final double PATHFINDING_MAX_ANGULAR_ANGULAR_ACCELERATION = 2 * Math.PI;

        public static final double PATHFINDING_APPROACH_OFFSET = 0.9;

        public static final double CORAL_BACKOFF_M =
                        edu.wpi.first.math.util.Units.inchesToMeters(4.5);
        public static final double ALGAE_APPROACH_OFFSET_M = 0.25;
        public static final double ALGAE_RETRACT_OFFSET_M = -0.45;

        // Tiempo máx para pruebas de drive-to (s)
        public static final double DRIVE_TO_POSE_TIMEOUT_S = 5.0;
        public static final double ALGAE_INTAKE_TIMEOUT_S = 3.0;



        public static final class LedColors {
                public static final int[] CORAL_L1 = {255, 10, 10};
                public static final int[] CORAL_LX = {255, 127, 80};
                public static final int[] ALGA = {0, 200, 100};
                public static final int[] MANUAL = {0, 120, 255};
                public static final int[] ERROR = {255, 0, 30};
        }

        public static int[] ledFor(GameMode mode) {
                return switch (mode) {
                        case CORAL_L1 -> LedColors.CORAL_L1;
                        case CORAL_LX -> LedColors.CORAL_LX;
                        case ALGAE -> LedColors.ALGA;
                        case MANUAL -> LedColors.MANUAL;
                };
        }

        // Orden FIJO (13):
        // 0: IntakeRear
        // 1: L1 Intake
        // 2: L1 Score
        // 3: L2
        // 4: L3
        // 5: L4
        // 6: Algae Intake Floor
        // 7: Algae Hold
        // 8: Processor Scoring
        // 9: Net PrePos
        // 10: Net Scoring
        // 11: L2 Algae Removal
        // 12: L3 Algae Removal

        public static final class PoseIdx {
                public static final int INTAKE_REAR = 0;
                public static final int L1_INTAKE = 1;
                public static final int L1_SCORE = 2;
                public static final int L2 = 3;
                public static final int L3 = 4;
                public static final int L4 = 5;
                public static final int ALGAE_INTAKE_FLOOR = 6;
                public static final int ALGAE_HOLD = 7;
                public static final int PROCESSOR_SCORE = 8;
                public static final int NET_PREPOS = 9;
                public static final int NET_SCORE = 10;
                public static final int ALGAE_REMOVE_L2 = 11;
                public static final int ALGAE_REMOVE_L3 = 12;
                public static final int COUNT = 13;
        }

        // Arreglos ÚNICOS (en el mismo orden de arriba)
        // lift en metros, wrist en grados (ajusta tus valores reales)
        public static final double[] LIFT_POSE = new double[PoseIdx.COUNT];
        public static final double[] WRIST_POSE = new double[PoseIdx.COUNT];


        static {
                // TODO: ajustar en comp
                // Coral
                // Lift
                LIFT_POSE[PoseIdx.INTAKE_REAR] = 0.0;
                LIFT_POSE[PoseIdx.L1_INTAKE] = 0.20;
                LIFT_POSE[PoseIdx.L1_SCORE] = 0.0;
                LIFT_POSE[PoseIdx.L2] = 0.32;
                LIFT_POSE[PoseIdx.L3] = 0.51;
                LIFT_POSE[PoseIdx.L4] = 0.71;

                // Wrist
                WRIST_POSE[PoseIdx.INTAKE_REAR] = 0.0;
                WRIST_POSE[PoseIdx.L1_INTAKE] = 20.0;
                WRIST_POSE[PoseIdx.L1_SCORE] = 20.0;
                WRIST_POSE[PoseIdx.L2] = 0.0;
                WRIST_POSE[PoseIdx.L3] = 0.0;
                WRIST_POSE[PoseIdx.L4] = 20.0;

                // Algae
                // Lift
                LIFT_POSE[PoseIdx.ALGAE_INTAKE_FLOOR] = 0.0;
                LIFT_POSE[PoseIdx.ALGAE_HOLD] = 0.0;

                LIFT_POSE[PoseIdx.PROCESSOR_SCORE] = 0.0;
                LIFT_POSE[PoseIdx.NET_PREPOS] = 0.6;
                LIFT_POSE[PoseIdx.NET_SCORE] = 0.7;

                LIFT_POSE[PoseIdx.ALGAE_REMOVE_L2] = 0.2;
                LIFT_POSE[PoseIdx.ALGAE_REMOVE_L3] = 0.4;

                // Wrist
                WRIST_POSE[PoseIdx.ALGAE_INTAKE_FLOOR] = 67.5;
                WRIST_POSE[PoseIdx.ALGAE_HOLD] = 0.0;

                WRIST_POSE[PoseIdx.PROCESSOR_SCORE] = 20.0; // “Processor” de tu array
                WRIST_POSE[PoseIdx.NET_PREPOS] = 0.0;
                WRIST_POSE[PoseIdx.NET_SCORE] = 40.0; // “Net” de tu array

                WRIST_POSE[PoseIdx.ALGAE_REMOVE_L2] = 42.5;
                WRIST_POSE[PoseIdx.ALGAE_REMOVE_L3] = 42.5;

        }

        // Helpers (con nombres claros)
        public static double liftForPose(int idx) {
                return LIFT_POSE[idx];
        }

        public static double wristForPose(int idx) {
                return WRIST_POSE[idx];
        }

}
