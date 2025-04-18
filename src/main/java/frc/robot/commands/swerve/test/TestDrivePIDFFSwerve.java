// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TestDrivePIDFFSwerve extends Command {

  double mps = 0;

  public TestDrivePIDFFSwerve(double mps) {
    this.mps = mps;
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());
  }

  @Override
  public void initialize() {
    Robot.getRobotContainer().getSwerveDriveTrain().stopModules();

  }

  @Override
  public void execute() {
    Robot.getRobotContainer().getSwerveDriveTrain().setModulesStates(mps, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}