// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist.control;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristMantainTarget extends Command {
  /** Creates a new WristMantainTo. */

  double maxPower, target;

  public WristMantainTarget(double maxPower) {
    addRequirements(Robot.getRobotContainer().getWrist());

      this.target = Double.NaN;
      this.maxPower = maxPower;
  }

  public WristMantainTarget(double angleTarget, double maxPower) {
    addRequirements(Robot.getRobotContainer().getWrist());

    this.maxPower = maxPower;
    this.target = angleTarget;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!Double.isNaN(target))
      Robot.getRobotContainer().getWrist().setTarget(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.getRobotContainer().getWrist().mantainTarget(maxPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getWrist().stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
