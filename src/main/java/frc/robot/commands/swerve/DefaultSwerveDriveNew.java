// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

public class DefaultSwerveDriveNew extends Command {
  /** Creates a new DefaultSwerveDriveNew. */
  public DefaultSwerveDriveNew() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());
  }

  // Called when the command is initially scheduled.
  SwerveDriveTrain swerveDriveTrain;

  @Override
  public void initialize() {
    swerveDriveTrain = Robot.getRobotContainer().getSwerveDriveTrain();
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed, strafe, turn;

    if (swerveDriveTrain.getIsOnCompetitiveField() == true) {

      // 1. Read input

      speed = -OI.getInstance().getDefaultSwerveDriveSpeed();
      strafe = -OI.getInstance().getDefaultSwerveDriveStrafe();
      turn = -OI.getInstance().getDefaultSwerveDriveTurn();

    } else {

      // 1. Read input

      speed = OI.getInstance().getDefaultSwerveDriveSpeed();
      strafe = OI.getInstance().getDefaultSwerveDriveStrafe();
      turn = OI.getInstance().getDefaultSwerveDriveTurn();

    }

    swerveDriveTrain.defaultDrive(speed, strafe, turn);
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
