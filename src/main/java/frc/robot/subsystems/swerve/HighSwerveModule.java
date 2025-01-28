// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HighAltitudeConstants;
import frc.robot.resources.components.speedController.HighAltitudeMotor;
import frc.robot.resources.components.speedController.HighAltitudeMotor.TypeOfMotor;
import frc.robot.resources.math.Math;
import edu.wpi.first.math.controller.ProfiledPIDController;

/** Add your docs here. */
public class HighSwerveModule {
  // another High Altitude Swerve Module Class, this time made by Gomez & Joaquín

  /// DIRECTION ///
  private HighAltitudeMotor directionMotor;
  private boolean isDirectionEncoderReversed;

  private final ProfiledPIDController directionProfiledPIDController;

  private double directionPIDAngleTarget = 0;
  private double directionPIDAngleSetPoint = 0;
  private double directionPIDVelocitySetPoint = 0;

  private SimpleMotorFeedforward directionFeedforward;

  /// DRIVE ///
  private HighAltitudeMotor driveMotor;

  private PIDController drivePIDController;
  private SimpleMotorFeedforward driveFeedforward;

  private double encoderOffSetPulses;

  private boolean isTalonEncoderReversed;
  private CANcoder absoluteEncoderController;

  public HighSwerveModule(int driveMotorPort, TypeOfMotor driveTypeOfMotor,
      boolean isDriveMotorReversed, boolean isDriveEncoderReversed,

      int directionMotorPort, TypeOfMotor directionTypeOfMotor,
      boolean isDirectionMotorReversed, boolean isDirectionEncoderReversed,

      int encodedTalonPort, double encoderOffsetPulses, boolean isTalonEncoderReversed) {

    directionMotor = new HighAltitudeMotor(directionMotorPort, directionTypeOfMotor);
    directionMotor.setInverted(isDirectionMotorReversed);
    directionMotor.setBrakeMode(true);
    this.isDirectionEncoderReversed = isDirectionEncoderReversed;

    // DIRECTION CONTROL //
    // 0.128, 0.01, 0.0128
    directionProfiledPIDController = new ProfiledPIDController(HighAltitudeConstants.SWERVE_DIRECTION_kP,
        HighAltitudeConstants.SWERVE_DIRECTION_kI, HighAltitudeConstants.SWERVE_DIRECTION_kD,
        new TrapezoidProfile.Constraints(HighAltitudeConstants.SWERVE_DIRECTION_MAX_VELOCITY,
            HighAltitudeConstants.SWERVE_DIRECTION_MAX_ACCELERATION));

    // enableContinousInput() calculates the route with less error
    directionProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

    directionFeedforward = new SimpleMotorFeedforward(HighAltitudeConstants.SWERVE_DIRECTION_kS,
        HighAltitudeConstants.SWERVE_DIRECTION_kV,
        HighAltitudeConstants.SWERVE_DIRECTION_kA);

    // DRIVE CONTROL //
    drivePIDController = new PIDController(HighAltitudeConstants.SWERVE_DRIVE_kP,
        HighAltitudeConstants.SWERVE_DRIVE_kI, HighAltitudeConstants.SWERVE_DRIVE_kD);

    driveFeedforward = new SimpleMotorFeedforward(HighAltitudeConstants.SWERVE_DRIVE_kS,
        HighAltitudeConstants.SWERVE_DRIVE_kV);

    absoluteEncoderController = new CANcoder(encodedTalonPort);
    this.isTalonEncoderReversed = isTalonEncoderReversed;
    this.encoderOffSetPulses = encoderOffsetPulses;

    // DRIVE MOTOR //
    driveMotor = new HighAltitudeMotor(driveMotorPort, driveTypeOfMotor);
    driveMotor.setInverted(isDriveMotorReversed);
    driveMotor.setBrakeMode(true);
  }

  ///// CANCODER /////

  public double getAbsoluteEncoderRAD() {
    return (getAbsoluteEncoderRaw() - encoderOffSetPulses) *
        HighAltitudeConstants.SWERVE_ABSOLUTE_ENCODER_RADIANS_PER_PULSE
        * (isTalonEncoderReversed ? -1.0 : 1.0);
  }

  public double getAbsoluteEncoderRaw() {
    return absoluteEncoderController.getPosition().getValueAsDouble();
  }

  /**
   * @return direction CANCoder velocity in radians / s
   */
  public double getEncoderVelocity() {
    return absoluteEncoderController.getVelocity().getValueAsDouble()
        * HighAltitudeConstants.SWERVE_ABSOLUTE_ENCODER_RADIANS_PER_PULSE
        * (isTalonEncoderReversed ? -1.0 : 1.0);

  }

  ///// MOTOR ENCODERS /////

  public void resetEncoders() {
    driveMotor.setEncoderPosition(0);
    directionMotor.setEncoderPosition(0);
  }

  public double getDriveEncoder() {
    return driveMotor.getEncPosition();
  }

  /**
   * @return drive encoder distance in meters.
   */
  public double getDriveDistance() {
    return driveMotor.getEncPosition() * HighAltitudeConstants.SWERVE_DRIVE_METERS_PER_REVOLUTION;
  }

  /**
   * @return drive encoder velocity in meters per second.
   */
  public double getDriveVelocity() {
    return driveMotor.getEncVelocity() * HighAltitudeConstants.SWERVE_DRIVE_METERS_PER_SEC_PER_VELOCITY_UNITS;
  }


  public double getDirectionEncoder() {
    return directionMotor.getEncPosition() * (isDirectionEncoderReversed ? -1.0 : 1.0);
  }

  public double getDirection() {
    return directionMotor.getEncPosition() * HighAltitudeConstants.SWERVE_DIRECTION_RADIANS_PER_PULSE
        * (isDirectionEncoderReversed ? -1.0 : 1.0);
  }

  // Getters for the motor objects themselves

  public HighAltitudeMotor getDriveMotor() {
    return driveMotor;
  }

  public HighAltitudeMotor getDirectionMotor() {
    return directionMotor;
  }

  // Getters for the position and state of the module

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveDistance(), Rotation2d.fromRadians(getAbsoluteEncoderRAD()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRadians(getAbsoluteEncoderRAD()));
  }

  // STATE SETTER

  public void setState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
      stop();
      return;
    }

    state.optimize(getState().angle);

    controlSwerveDirection(state.angle.getRadians());
    controlSwerveSpeed(state.speedMetersPerSecond);
  }

  public void controlSwerveSpeed(double mps) {
    double feedforward = driveFeedforward.calculate(mps);

    double pidOutput = drivePIDController.calculate(getDriveVelocity(), mps);

    double driveOutput = pidOutput + feedforward;

    driveOutput = Math.clamp(driveOutput, -HighAltitudeConstants.MAX_VOLTAGE, HighAltitudeConstants.MAX_VOLTAGE);

    driveMotor.setVoltage(driveOutput);
  }

  /**
   * Moves the swerve module to the desired angle.
   * @param angleTarget the target angle in radians.
   */
  public void controlSwerveDirection(double angleTarget) {
    double pidVal = directionProfiledPIDController.calculate(getAbsoluteEncoderRAD(),
        angleTarget);
    double targetSpeed = directionProfiledPIDController.getSetpoint().velocity;

    double feedforwardVal = directionFeedforward.calculate(targetSpeed);
    double directionOutput = pidVal + feedforwardVal;

    Math.clamp(directionOutput, -HighAltitudeConstants.MAX_VOLTAGE, HighAltitudeConstants.MAX_VOLTAGE);

    directionMotor.setVoltage(directionOutput);

    
    directionPIDAngleTarget = angleTarget;
    directionPIDAngleSetPoint = getDirectionPIDController().getSetpoint().position;
    directionPIDVelocitySetPoint = getDirectionPIDController().getSetpoint().velocity;
  }

  public ProfiledPIDController getDirectionPIDController() {
    return directionProfiledPIDController;
  }

  public PIDController getDrivePIDController() {
    return drivePIDController;
  }

  public void stop() {
    driveMotor.set(0);
    directionMotor.set(0);
  }

  // Smartdashboard prints for debugging.

  public void putProcessedValues(String identifier) {
    SmartDashboard.putNumber(identifier + "DrivePos", getDriveDistance());
    SmartDashboard.putNumber(identifier + "DirPos", getDirection());
    SmartDashboard.putNumber(identifier + "AbsPos", getAbsoluteEncoderRAD());
    SmartDashboard.putNumber(identifier + "AbsRawPos", absoluteEncoderController.getPosition().getValueAsDouble());
  }

  public void putControlTunningValues(String identifier) {
    // This is what you should print:
    // 1. Velocity of the DriveMotorEnc
    SmartDashboard.putNumber(identifier + "DriveVelocity", getDriveVelocity());

    // 2. Graphic of the CANCoder Angle
    SmartDashboard.putNumber(identifier + "CANCoder Angle", getAbsoluteEncoderRAD());

    // 3. Graphic of the CANCoder Velocity
    SmartDashboard.putNumber(identifier + "CANCoder Velocity", getEncoderVelocity());

    // 5. Setpoint of the ProfiledPIDController Angle
    SmartDashboard.putNumber(identifier + "Direction Angle target", directionPIDAngleTarget);
    SmartDashboard.putNumber(identifier + "Direction Angle SetPoint", directionPIDAngleSetPoint);

    // 6. Setpoint of the ProfiledPIDController Velocity
    SmartDashboard.putNumber(identifier + "Direction Velocity SetPoint", directionPIDVelocitySetPoint);

  }

  public void putEncoderValuesInvertedApplied(String identifier) {
    SmartDashboard.putNumber(identifier + "DriveEncPos", driveMotor.getEncPosition());
    SmartDashboard.putNumber(identifier + "DirEncPos", getDirectionEncoder());
    SmartDashboard.putNumber(identifier + "AbsEncPos",
        absoluteEncoderController.getPosition().getValueAsDouble() * (isTalonEncoderReversed ? -1.0 : 1.0));
  }
}