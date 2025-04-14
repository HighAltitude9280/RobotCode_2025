// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.autonomous.feeder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants.REEF_POSITION;
import frc.robot.HighAltitudeConstants.REEF_SIDE;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToCoralStation extends Command {

  private Pose2d pos;
  private Boolean left;
  private final double maxLinearVelocity, maxAngularVelocity;
  private Pose2d targetPose;

  private boolean isFinished = false;

  /**
   * Command to drive the robot to the CoralStation using pose.
   *
   * @param position           The position to drive the robot to (Near, Middle or
   *                           Far).
   * @param left               True for left feeder, false for feeder.
   * @param maxLinearVelocity  Max linear velocity in m/s.
   * @param maxAngularVelocity Max angular velocity in rad/
   */
  public DriveToCoralStation(Pose2d position, Boolean left,
      double maxLinearVelocity, double maxAngularVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());

    this.pos = position;
    this.left = left;
    this.maxAngularVelocity = maxAngularVelocity;
    this.maxLinearVelocity = maxLinearVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
