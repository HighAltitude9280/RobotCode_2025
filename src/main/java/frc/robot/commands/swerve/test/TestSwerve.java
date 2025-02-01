// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.test;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.HighSwerveModule;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

public class TestSwerve extends Command {
  SwerveDriveTrain swerveDriveTrain;
  ArrayList<HighSwerveModule> modules;
  double turn = 0.1;
  double speed = 0;

  /** Creates a new TestSwerve. */
  public TestSwerve() {
    swerveDriveTrain = Robot.getRobotContainer().getSwerveDriveTrain();
    addRequirements(swerveDriveTrain);

    modules = swerveDriveTrain.getModules();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turn = 0.0;
    speed = 0.1;
    for (HighSwerveModule swerveModule : modules) {
      swerveModule.getDriveMotor().set(speed);
      swerveModule.getDirectionMotor().set(turn);
    }
    //System.out.println("AAAAAAAAAAAAAAAAAAAAA");
    /*
     * for (HighAltitudeSwerveModule swerveModule : modules) {
     * swerveModule.getDriveMotor().set(0);
     * swerveModule.getDirectionMotor().set(0.08);
     * }
     */
    // swerveDriveTrain.getBackRight().getDirectionMotor().set(turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turn = 0;
    speed = 0;
    for (HighSwerveModule swerveModule : modules) {
      swerveModule.getDriveMotor().set(speed);
      swerveModule.getDirectionMotor().set(turn);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * if(swerveDriveTrain.{
     * return true;
     * } else {
     */
    return false;

  }
}
