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

  private double encoderTarget = 0.0;

  private PIDController pidController;

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
    return HighAltitudeConstants.WRIST_ZERO_ANGLE
        + (RobotMap.WRIST_ENCODER_IS_INVERTED ? 1.0 : 1.0) * getWristEncoderPosition()
            * HighAltitudeConstants.WRIST_DEGREES_PER_PULSE;
  }

  public double getWristEncoderPosition() {
    return wristMotors.getEncoderPosition();
  }

  public double getEncoderTarget() {
    return encoderTarget;
  }

  public void setAngleTarget(double targetDegrees) {
    this.encoderTarget = angleToEncoder(targetDegrees);
  }

  public double angleToEncoder(double angleDegrees) {
    return (RobotMap.WRIST_ENCODER_IS_INVERTED ? -1.0 : 1.0) * (angleDegrees - HighAltitudeConstants.WRIST_ZERO_ANGLE)
        / HighAltitudeConstants.WRIST_DEGREES_PER_PULSE;
  }

  public boolean mantainTarget(double maxPower) {

    double power = pidController.calculate(getWristEncoderPosition(), getEncoderTarget());
    power = Math.clamp(power * maxPower, -maxPower, maxPower);
    driveWrist(power);

    double sigma = getEncoderTarget() - getWristEncoderPosition();
    return Math.abs(sigma) < HighAltitudeConstants.WRIST_ARRIVE_OFFSET;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Encoder Target", getEncoderTarget());
    SmartDashboard.putNumber("Wrist Encoder Position", getWristEncoderPosition());

    SmartDashboard.putNumber("Wrist Encoder Angle", getWristPosDegrees());
  }
}
