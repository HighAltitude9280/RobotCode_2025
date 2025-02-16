// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extensor;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;

public class Lift extends SubsystemBase {
  HighAltitudeMotorGroup liftMotors;

  ElevatorFeedforward liftFeedforward;

  /** Creates a new Lift. */
  public Lift() {

    liftMotors = new HighAltitudeMotorGroup(RobotMap.LIFT_MOTOR_PORTS,
        RobotMap.LIFT_INVERTED_MOTORS_PORTS,
        RobotMap.LIFT_MOTOR_TYPES);
    liftMotors.setEncoderInverted(RobotMap.LIFT_ENCODER_IS_INVERTED);
    liftMotors.setBrakeMode(true);

    liftFeedforward = new ElevatorFeedforward(HighAltitudeConstants.LIFT_kS, HighAltitudeConstants.LIFT_kG,
        HighAltitudeConstants.LIFT_kV, HighAltitudeConstants.LIFT_kA);
    // resetEncoders();
  }

  public void driveLift(double speed) {
    // System.out.println("ExtensorPower: " + speed);
    liftMotors.setAll(speed);
  }

  public void setHeight(double velocity, double acceleration) {
    double ffOutput = liftFeedforward.calculate(velocity, acceleration);

    driveLift(ffOutput);
  }

  public void liftUp() {
    liftMotors.setAll(HighAltitudeConstants.LIFT_UP_SPEED);
  }

  public void liftDown() {
    liftMotors.setAll(HighAltitudeConstants.LIFT_DOWN_SPEED);
  }

  public void stopLift() {
    liftMotors.setAll(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
