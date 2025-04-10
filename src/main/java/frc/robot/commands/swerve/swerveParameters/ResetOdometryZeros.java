// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.swerveParameters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetOdometryZeros extends InstantCommand {
  SwerveDriveTrain swerveDriveTrain;

  public ResetOdometryZeros() {
    swerveDriveTrain = Robot.getRobotContainer().getSwerveDriveTrain();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDriveTrain.resetPose(new Pose2d());
  }
}
