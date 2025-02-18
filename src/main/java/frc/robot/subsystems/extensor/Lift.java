// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extensor;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;
import frc.robot.resources.math.Math;

public class Lift extends SubsystemBase {
  private HighAltitudeMotorGroup liftMotors;

  private final ElevatorFeedforward liftFeedforward;
  private final ProfiledPIDController liftProfiledPIDController;

  private double liftPIDMetersSetPoint = 0;
  private double liftPIDVelocityTarget = 0;

  private double liftOutput;
  private double currentTarget = 0;

  private boolean onTarget = false;

  /** Creates a new Lift. */
  public Lift() {

    liftMotors = new HighAltitudeMotorGroup(RobotMap.LIFT_MOTOR_PORTS,
        RobotMap.LIFT_INVERTED_MOTORS_PORTS,
        RobotMap.LIFT_MOTOR_TYPES);
    liftMotors.setEncoderInverted(RobotMap.LIFT_ENCODER_IS_INVERTED);
    liftMotors.setBrakeMode(true);

    liftFeedforward = new ElevatorFeedforward(HighAltitudeConstants.LIFT_kS, HighAltitudeConstants.LIFT_kG,
        HighAltitudeConstants.LIFT_kV, HighAltitudeConstants.LIFT_kA);

    liftProfiledPIDController = new ProfiledPIDController(HighAltitudeConstants.LIFT_kP, HighAltitudeConstants.LIFT_kI,
        HighAltitudeConstants.LIFT_kD, new TrapezoidProfile.Constraints(HighAltitudeConstants.LIFT_MAX_VELOCITY,
            HighAltitudeConstants.LIFT_MAX_ACCELERATION));
    // resetEncoders();
  }

  public void driveLift(double speed) {
    // System.out.println("ExtensorPower: " + speed);
    liftMotors.setAll(speed);
  }

  public double getLiftEncoderPosition() {
    return liftMotors.getEncoderPosition();
  }

  public double getLiftPosMeters() {
    return getLiftEncoderPosition()
        * HighAltitudeConstants.LIFT_METERS_PER_PULSE;
  }

  public double getLiftVelocityMPS() { // de RPM a Metersper Pulse
    return liftMotors.getEncoderVelocity() / 60 * HighAltitudeConstants.LIFT_METERS_PER_PULSE;
  }

  public ProfiledPIDController getLiftPIDController() {
    return liftProfiledPIDController;
  }

  public void controlPosition(double metersTarget, double maxVoltage) {
    double pidVal = liftProfiledPIDController.calculate(getLiftPosMeters(), metersTarget);
    double targetSpeed = liftProfiledPIDController.getSetpoint().velocity;

    double feedforwardVal = liftFeedforward.calculate(targetSpeed);
    double liftOutput = pidVal + feedforwardVal;

    liftOutput = Math.clamp(liftOutput, -maxVoltage, maxVoltage);

    currentTarget = metersTarget;
    liftPIDMetersSetPoint = getLiftPIDController().getSetpoint().position;
    liftPIDVelocityTarget = getLiftPIDController().getSetpoint().velocity;

    liftMotors.setAll(liftOutput);

    double delta = getTarget() - getLiftPosMeters();
    this.onTarget = Math.abs(delta) < HighAltitudeConstants.LIFT_ARRIVE_OFFSET;

    this.liftOutput = liftOutput;

  }

  public void controlPosition(double maxVoltage) {
    controlPosition(currentTarget, maxVoltage);
  }

  /**
   * Sets the target of the lift.
   * 
   * @param target the target angle in meters
   */
  public void setTarget(double target) {
    this.currentTarget = target;
  }

  public double getTarget() {
    return currentTarget;
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
    SmartDashboard.putNumber("Lift Encoder Target", getTarget());

    SmartDashboard.putNumber("Lift Encoder Position", getLiftEncoderPosition());

    SmartDashboard.putNumber("Lift Encoder Meters", getLiftPosMeters());

    SmartDashboard.putNumber("Lift PID Output", this.liftOutput);

    SmartDashboard.putNumber("Lift Velocity MPS", getLiftVelocityMPS());

    SmartDashboard.putNumber("Lift Setpoint Position", getLiftPIDController().getSetpoint().position);

    SmartDashboard.putNumber("Lift Velocity Target", getLiftPIDController().getSetpoint().velocity);

  }
}
