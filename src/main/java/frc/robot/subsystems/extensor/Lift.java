// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extensor;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearAcceleration;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.HighAltitudeConstants;
import frc.robot.RobotMap;
import frc.robot.resources.components.speedController.HighAltitudeMotorGroup;
import frc.robot.resources.math.Math;

public class Lift extends SubsystemBase {
  private HighAltitudeMotorGroup liftMotors;

  private final ElevatorFeedforward liftFeedforward;
  private final ProfiledPIDController liftProfiledPIDController;

  private double liftOutput;
  private double currentTarget = 0;

  private double lastSpeedSetpoint;
  private double lastSetpointTimestamp;

  private boolean onTarget = false;

  DigitalInput topLimitSwitch, bottomLimitSwitch;

  SysIdRoutine sysIdRoutine;

  MutVoltage voltage = Volts.mutable(0);
  MutDistance distance = Meters.mutable(0);
  MutLinearVelocity velocity = MetersPerSecond.mutable(0);

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

    if (RobotMap.LIFT_TOP_LIMIT_SWITCH_IS_AVAILABLE)
      topLimitSwitch = new DigitalInput(RobotMap.LIFT_TOP_LIMIT_SWITCH_PORT);
    if (RobotMap.LIFT_BOTTOM_LIMIT_SWITCH_IS_AVAILABLE)
      bottomLimitSwitch = new DigitalInput(RobotMap.LIFT_BOTTOM_LIMIT_SWITCH_PORT);

    resetEncoders();

    lastSpeedSetpoint = 0;
    lastSetpointTimestamp = Timer.getFPGATimestamp();

    sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(Volts.of(1).per(Seconds),
        Volts.of(1.75), Seconds.of(10)),
        new SysIdRoutine.Mechanism(this::driveSysID, this::logSysId, this));
  }

  public void driveLift(double speed) {
    if (getBottomLimitSwitch()) {
      liftMotors.resetEncoder();
      liftMotors.setAll(Math.max(0, speed));
    } else if (getTopLimitSwitch()) {
      liftMotors.setAll(Math.min(speed, 0));
    }
  }

  public double getLiftEncoderPosition() {
    return liftMotors.getEncoderPosition();
  }

  public void resetEncoders() {
    liftMotors.resetEncoder();
  }

  public boolean getTopLimitSwitch() {
    if (RobotMap.LIFT_TOP_LIMIT_SWITCH_IS_AVAILABLE)
      return topLimitSwitch.get();
    return false;
  }

  public boolean getBottomLimitSwitch() {
    if (RobotMap.LIFT_BOTTOM_LIMIT_SWITCH_IS_AVAILABLE)
      return bottomLimitSwitch.get();
    return false;
  }

  public double getLiftPosMeters() {
    return getLiftEncoderPosition()
        * HighAltitudeConstants.LIFT_METERS_PER_PULSE;
  }

  public double getLiftVelocityMPS() { // de RPM a Metersper Pulse
    return liftMotors.getEncoderVelocity() * HighAltitudeConstants.LIFT_METERS_PER_PULSE;
  }

  public ProfiledPIDController getLiftPIDController() {
    return liftProfiledPIDController;
  }

  public void controlPosition(double metersTarget, double maxVoltage, double arriveOffset) {
    double pidVal = liftProfiledPIDController.calculate(getLiftPosMeters(), metersTarget);

    double targetSpeed = liftProfiledPIDController.getSetpoint().velocity;

    double currentTime = Timer.getFPGATimestamp();
    double targetAcceleration = (targetSpeed - lastSpeedSetpoint) / (currentTime - lastSetpointTimestamp);

    double feedforwardVal = liftFeedforward.calculate(targetSpeed, targetAcceleration);

    double liftOutput = pidVal + feedforwardVal;

    liftOutput = Math.clamp(liftOutput, -maxVoltage, maxVoltage);

    double delta = getTarget() - getLiftPosMeters();
    this.onTarget = Math.abs(delta) < arriveOffset;

    if (onTarget) {
      liftOutput = HighAltitudeConstants.LIFT_kG;
    }

    liftMotors.setVoltage(liftOutput);
    // liftMotors.setAll(liftOutput);

    currentTarget = metersTarget;
    lastSpeedSetpoint = targetSpeed;
    lastSetpointTimestamp = currentTime;
    this.liftOutput = liftOutput;

  }

  public void controlPosition(double maxVoltage, double arriveOffset) {
    controlPosition(currentTarget, maxVoltage, arriveOffset);
  }

  /**
   * Sets the target of the lift.
   * 
   * @param target the target angle in meters
   */
  public void setTarget(double target) {
    this.currentTarget = target;
  }

  public void addToTarget(double meters) {
    this.currentTarget += meters;
  }

  public double getTarget() {
    return currentTarget;
  }

  public boolean onTarget() {
    return this.onTarget;
  }

  public void setSpeed(double velocity, double acceleration) {
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

  public void setBrakeModeAllMotors(boolean brake) {
    liftMotors.setBrakeMode(brake);
  }

  private void driveSysID(Voltage voltage) {
    liftMotors.setVoltage(voltage.magnitude());
  }

  private void logSysId(SysIdRoutineLog log) {
    var motor = liftMotors.getMotors().get(0);

    voltage.mut_replace(motor.get() * motor.getBusVoltage(), Volts);
    distance.mut_replace(getLiftPosMeters(), Meters);
    velocity.mut_replace(getLiftVelocityMPS(), MetersPerSecond);

    log.motor("lift").voltage(voltage).linearPosition(distance).linearVelocity(velocity);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public void putTuningValues() {
    SmartDashboard.putNumber("Lift Encoder Target", getTarget());
    SmartDashboard.putNumber("Lift Encoder Meters", getLiftPosMeters());
    SmartDashboard.putNumber("Lift PID Output", this.liftOutput);
    SmartDashboard.putNumber("Lift Setpoint Position", getLiftPIDController().getSetpoint().position);
    SmartDashboard.putBoolean("Lift OnTarget", onTarget);
    SmartDashboard.putNumber("Lift Encoder Position", getLiftEncoderPosition());
    SmartDashboard.putNumber("Lift Velocity MPS", getLiftVelocityMPS());
    SmartDashboard.putNumber("Lift Velocity Target", lastSpeedSetpoint);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotMap.LIFT_BOTTOM_LIMIT_SWITCH_IS_AVAILABLE && getBottomLimitSwitch())
      resetEncoders();

    SmartDashboard.putNumber("Lift Encoder Target", getTarget());
    SmartDashboard.putNumber("Lift Encoder Meters", getLiftPosMeters());
    SmartDashboard.putNumber("Lift PID Output", this.liftOutput);
    SmartDashboard.putNumber("Lift Setpoint Position", getLiftPIDController().getSetpoint().position);
    SmartDashboard.putBoolean("Lift OnTarget", onTarget);
    SmartDashboard.putNumber("Lift Encoder Position", getLiftEncoderPosition());
    SmartDashboard.putNumber("Lift Velocity MPS", getLiftVelocityMPS());
    SmartDashboard.putNumber("Lift Velocity Target", lastSpeedSetpoint);
  }
}
