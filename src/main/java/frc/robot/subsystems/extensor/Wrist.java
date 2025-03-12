// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;
import frc.robot.resources.math.Math;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  HighAltitudeMotorGroup wristMotors;
  double wristEncoderPosition, wristPositionDegrees, wristPositionRawEncoder;

  private double currentTarget = 0;

  private PIDController pidController;

  private boolean onTarget = false;

  /** Creates a new Wrist. */
  public Wrist() {
    wristMotors = new HighAltitudeMotorGroup(RobotMap.WRIST_MOTOR_PORTS,
        RobotMap.WRIST_INVERTED_MOTORS_PORTS,
        RobotMap.WRIST_MOTOR_TYPES);

    wristMotors.setEncoderInverted(RobotMap.WRIST_ENCODER_IS_INVERTED);
    wristMotors.setBrakeMode(true);

    pidController = new PIDController(HighAltitudeConstants.WRIST_kP, HighAltitudeConstants.WRIST_kI,
        HighAltitudeConstants.WRIST_kD);
    // resetEncoders();
  }

  public void driveWrist(double speed) {
    wristMotors.setAll(speed);
  }

  public void stopMotors() {
    wristMotors.setAll(0);
  }

  public double getWristPosDegrees() {
    return (RobotMap.WRIST_ENCODER_IS_INVERTED ? 1.0 : 1.0) * getWristEncoderPosition()
        * HighAltitudeConstants.WRIST_DEGREES_PER_PULSE - HighAltitudeConstants.WRIST_ZERO_ANGLE;
  }

  public double getWristEncoderPosition() {
    return wristMotors.getEncoderPosition();
  }

  private double getCurrentTarget() {
    return currentTarget;
  }

  /**
   * Sets the target of the wrist.
   * 
   * @param target the target angle in degrees
   */
  public void setCurrentTarget(double target) {
    this.currentTarget = target;
  }

  public void mantainTarget(double maxPower) {

    double power = pidController.calculate(getWristPosDegrees(), getCurrentTarget());
    power = Math.clamp(power * maxPower, -maxPower, maxPower);
    driveWrist(power);

    double delta = getCurrentTarget() - getWristPosDegrees();
    this.onTarget = Math.abs(delta) < HighAltitudeConstants.WRIST_ARRIVE_OFFSET;
    if (onTarget)
      power = 0.0;
  }

  // Explica q

  public void addToTarget(double meters) {
    this.currentTarget += meters;
  }

  public boolean onTarget() {
    return this.onTarget;
  }

  public void putTuningValues()
  {
    SmartDashboard.putNumber("Wrist Encoder Target", getCurrentTarget());
    SmartDashboard.putNumber("Wrist Encoder Position", getWristEncoderPosition());
    SmartDashboard.putNumber("Wrist Encoder Angle", getWristPosDegrees());
    SmartDashboard.putBoolean("Wrist OnTarget", onTarget);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
