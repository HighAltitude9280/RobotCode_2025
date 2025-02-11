// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  HighAltitudeMotorGroup wristMotors;
  double wristEncoderPosition, wristPositionDegrees;

  /** Creates a new Wrist. */
  public Wrist() {
    wristMotors = new HighAltitudeMotorGroup(RobotMap.WRIST_MOTOR_PORTS, RobotMap.WRIST_INVERTED_MOTORS_PORTS,
        RobotMap.WRIST_MOTOR_TYPES);
    wristMotors.setEncoderInverted(RobotMap.WRIST_ENCODER_IS_INVERTED);
    wristMotors.setBrakeMode(true);

    // resetEncoders();
  }

  public void driveWrist(double speed) {
    wristMotors.setAll(speed);
  }

  public void stopMotors() {
    wristMotors.setAll(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
