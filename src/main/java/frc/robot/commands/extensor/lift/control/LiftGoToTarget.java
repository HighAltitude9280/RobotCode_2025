// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extensor.lift.control;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LiftGoToTarget extends Command {
  /** Creates a new LiftMantainTarget. */

  double maxPower, target, arriveOffset;

  public LiftGoToTarget(double maxPower, double arriveOffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.getRobotContainer().getLift());

    this.target = Double.NaN;
    this.maxPower = maxPower;
    this.arriveOffset = arriveOffset;
  }

  public LiftGoToTarget(double maxPower, double targetMeters, double arriveOffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.getRobotContainer().getLift());

    this.target = targetMeters;
    this.maxPower = maxPower;
    this.arriveOffset = arriveOffset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!Double.isNaN(target))
      Robot.getRobotContainer().getLift().setTarget(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.getRobotContainer().getLift().controlPosition(maxPower, arriveOffset);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getLift().stopLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.getRobotContainer().getLift().onTarget();
  }
}
