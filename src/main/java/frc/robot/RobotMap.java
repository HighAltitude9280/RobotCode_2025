// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.resources.components.speedController.HighAltitudeMotor.TypeOfMotor;

/** Add your docs here. */
public class RobotMap {

    ////////////////////////// SWERVE //////////////////////////

    ///// FRONT LEFT
    // DRIVE
    public static final int SWERVE_FRONT_LEFT_DRIVE_MOTOR_PORT = 12;
    public static final TypeOfMotor SWERVE_FRONT_LEFT_DRIVE_MOTOR_TYPE = TypeOfMotor.TALON_FX;
    public static final boolean SWERVE_FRONT_LEFT_DRIVE_MOTOR_INVERTED = false;
    public static final boolean SWERVE_FRONT_LEFT_DRIVE_ENCODER_INVERTED = false;
    // DIRECTION
    public static final int SWERVE_FRONT_LEFT_DIRECTION_MOTOR_PORT = 17;
    public static final TypeOfMotor SWERVE_FRONT_LEFT_DIRECTION_MOTOR_TYPE = TypeOfMotor.SPARK_MAX_BRUSHLESS;
    public static final boolean SWERVE_FRONT_LEFT_DIRECTION_MOTOR_INVERTED = true;
    public static final boolean SWERVE_FRONT_LEFT_DIRECTION_ENCODER_INVERTED = false;
    // CANCODER
    public static final int SWERVE_FRONT_LEFT_ENCODED_TALON_PORT = 39;
    public static final double SWERVE_FRONT_LEFT_DIRECTION_ENCODER_OFFSET_PULSES = -0.11474609375;
    public static final boolean SWERVE_FRONT_LEFT_ENCODED_TALON_INVERTED = false;

    ///// FRONT RIGHT
    // DRIVE
    public static final int SWERVE_FRONT_RIGHT_DRIVE_MOTOR_PORT = 10;
    public static final TypeOfMotor SWERVE_FRONT_RIGHT_DRIVE_MOTOR_TYPE = TypeOfMotor.TALON_FX;
    public static final boolean SWERVE_FRONT_RIGHT_DRIVE_MOTOR_INVERTED = true;
    public static final boolean SWERVE_FRONT_RIGHT_DRIVE_ENCODER_INVERTED = false;
    // DIRECTION
    public static final int SWERVE_FRONT_RIGHT_DIRECTION_MOTOR_PORT = 11;
    public static final TypeOfMotor SWERVE_FRONT_RIGHT_DIRECTION_MOTOR_TYPE = TypeOfMotor.SPARK_MAX_BRUSHLESS;
    public static final boolean SWERVE_FRONT_RIGHT_DIRECTION_MOTOR_INVERTED = true;
    public static final boolean SWERVE_FRONT_RIGHT_DIRECTION_ENCODER_INVERTED = false;
    // CANCODER
    public static final int SWERVE_FRONT_RIGHT_ENCODED_TALON_PORT = 40;
    public static final double SWERVE_FRONT_RIGHT_DIRECTION_ENCODER_OFFSET_PULSES = 0.084716796875;
    public static final boolean SWERVE_FRONT_RIGHT_ENCODED_TALON_INVERTED = false;

    ///// BACK LEFT
    // DRIVE
    public static final int SWERVE_BACK_LEFT_DRIVE_MOTOR_PORT = 14;
    public static final TypeOfMotor SWERVE_BACK_LEFT_DRIVE_MOTOR_TYPE = TypeOfMotor.TALON_FX;
    public static final boolean SWERVE_BACK_LEFT_DRIVE_MOTOR_INVERTED = false;
    public static final boolean SWERVE_BACK_LEFT_DRIVE_ENCODER_INVERTED = false;
    // DIRECTION
    public static final int SWERVE_BACK_LEFT_DIRECTION_MOTOR_PORT = 15;
    public static final TypeOfMotor SWERVE_BACK_LEFT_DIRECTION_MOTOR_TYPE = TypeOfMotor.SPARK_MAX_BRUSHLESS;
    public static final boolean SWERVE_BACK_LEFT_DIRECTION_MOTOR_INVERTED = true;
    public static final boolean SWERVE_BACK_LEFT_DIRECTION_ENCODER_INVERTED = false;
    // CANCODER
    public static final int SWERVE_BACK_LEFT_ENCODED_TALON_PORT = 38;
    public static final double SWERVE_BACK_LEFT_DIRECTION_ENCODER_OFFSET_PULSES = 0.208251953125;
    public static final boolean SWERVE_BACK_LEFT_ENCODED_TALON_INVERTED = false;

    ///// BACK RIGHT
    // DRIVE
    public static final int SWERVE_BACK_RIGHT_DRIVE_MOTOR_PORT = 16;
    public static final TypeOfMotor SWERVE_BACK_RIGHT_DRIVE_MOTOR_TYPE = TypeOfMotor.TALON_FX;
    public static final boolean SWERVE_BACK_RIGHT_DRIVE_MOTOR_INVERTED = true;
    public static final boolean SWERVE_BACK_RIGHT_DRIVE_ENCODER_INVERTED = false;
    // DIRECTION
    public static final int SWERVE_BACK_RIGHT_DIRECTION_MOTOR_PORT = 13;
    public static final TypeOfMotor SWERVE_BACK_RIGHT_DIRECTION_MOTOR_TYPE = TypeOfMotor.SPARK_MAX_BRUSHLESS;
    public static final boolean SWERVE_BACK_RIGHT_DIRECTION_MOTOR_INVERTED = true;
    public static final boolean SWERVE_BACK_RIGHT_DIRECTION_ENCODER_INVERTED = false;
    // CANCODER
    public static final int SWERVE_BACK_RIGHT_ENCODED_TALON_PORT = 37;
    public static final double SWERVE_BACK_RIGHT_DIRECTION_ENCODER_OFFSET_PULSES = 0.055419921875;
    public static final boolean SWERVE_BACK_RIGHT_ENCODED_TALON_INVERTED = false;

    ////////////////////////// lIFT //////////////////////////

    public static final int[] LIFT_MOTOR_PORTS = { 21, 22 };
    public static final int[] LIFT_INVERTED_MOTORS_PORTS = { 21, 22 };
    public static final TypeOfMotor[] LIFT_MOTOR_TYPES = { TypeOfMotor.SPARK_MAX_BRUSHLESS,
            TypeOfMotor.SPARK_MAX_BRUSHLESS };
    public static final boolean LIFT_ENCODER_IS_INVERTED = false;

    public static final boolean LIFT_TOP_LIMIT_SWITCH_IS_AVAILABLE = true;
    public static final int LIFT_TOP_LIMIT_SWITCH_PORT = 9;

    public static final boolean LIFT_BOTTOM_LIMIT_SWITCH_IS_AVAILABLE = true;
    public static final int LIFT_BOTTOM_LIMIT_SWITCH_PORT = 8;

    ////////////////////////// GRIPPER //////////////////////////

    public static final int[] GRIPPER_MOTOR_PORTS = { 31 };
    public static final int[] GRIPPER_INVERTED_MOTORS_PORTS = {};
    public static final TypeOfMotor[] GRIPPER_MOTOR_TYPES = { TypeOfMotor.SPARK_MAX_BRUSHLESS };
    public static final boolean GRIPPER_ENCODER_IS_INVERTED = false;

    public static final boolean GRIPPER_DIGITAL_INPUT_IS_AVAILABLE = false;
    public static final int GRIPPER_DIGITAL_INPUT_PORT = 7;
    public static final boolean GRIPPER_I2C_PORT_IS_AVAILABLE = true;

    ////////////////////////// WRIST //////////////////////////
    public static final int[] WRIST_MOTOR_PORTS = { 41 };
    public static final int[] WRIST_INVERTED_MOTORS_PORTS = { 41 };
    public static final TypeOfMotor[] WRIST_MOTOR_TYPES = { TypeOfMotor.SPARK_MAX_BRUSHLESS };
    public static final boolean WRIST_ENCODER_IS_INVERTED = false;

}