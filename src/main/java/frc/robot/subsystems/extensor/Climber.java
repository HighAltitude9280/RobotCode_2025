// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;

public class Climber extends SubsystemBase {
  HighAltitudeMotorGroup climberMotors;

  /** Creates a new Climber. */
  public Climber() {
    climberMotors = new HighAltitudeMotorGroup(RobotMap.CLIMBER_MOTOR_PORTS,
        RobotMap.CLIMBER_INVERTED_MOTORS_PORTS,
        RobotMap.CLIMBER_MOTOR_TYPES);
  }

  public void driveClimber(double speed) {
    climberMotors.setAll(speed);
  }

  public void stopClimber() {
    climberMotors.setAll(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
