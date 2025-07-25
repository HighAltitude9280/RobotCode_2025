// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestAlignWithPose extends Command {
  SwerveDriveTrain swerve;

  /** Creates a new TestAlignWithPose. */
  public TestAlignWithPose() {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = Robot.getRobotContainer().getSwerveDriveTrain();
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.WW
  @Override
  public void end(boolean interrupted) {
    System.out.println("AAAA");
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerve.AlignWithTargetPose(new Pose2d(15.5, 4.0, Rotation2d.fromDegrees(179.99)),
        HighAltitudeConstants.VISION_POSE_MAX_SPEED,
        HighAltitudeConstants.VISION_POSE_MAX_TURN);

  }
}
