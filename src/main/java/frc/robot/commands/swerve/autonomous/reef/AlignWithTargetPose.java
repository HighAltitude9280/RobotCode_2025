// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.autonomous.reef;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.HighAltitudeConstants.REEF_POSITION;
import frc.robot.HighAltitudeConstants.REEF_SIDE;;

public class AlignWithTargetPose extends Command {
  
  private REEF_POSITION pos;
  private REEF_SIDE side;
  private Boolean left;
  private final double maxLinearVelocity, maxAngularVelocity;
  private Pose2d targetPose;

  /**
   * Command to align the robot using pose.
   *
   * @param position           The position to align the robot to (nullable for
   *                           detection mode).
   * @param side               The reef side (nullable for detection mode).
   * @param left               True for left branch, false for right (nullable for
   *                           detection mode).
   * @param maxLinearVelocity  Max linear velocity in m/s.
   * @param maxAngularVelocity Max angular velocity in rad/
   */
  public AlignWithTargetPose(REEF_POSITION position, REEF_SIDE side, Boolean left,
      double maxLinearVelocity, double maxAngularVelocity) {
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());

    this.pos = position;
    this.left = left;
    this.maxAngularVelocity = maxAngularVelocity;
    this.maxLinearVelocity = maxLinearVelocity;
    this.side = side;
  }

  @Override
  public void initialize() {
    if (pos == null && side != null)
      this.pos = side.getPosition(Robot.isFrontMode());

    left = left != null ? left : Robot.isLeftMode();
    determineTarget();
  }

  private void determineTarget() {

    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    int[] reefIDs = (alliance == DriverStation.Alliance.Red)
        ? HighAltitudeConstants.RED_APRILTAG_IDS
        : HighAltitudeConstants.BLUE_APRILTAG_IDS;

    var branches = (alliance == DriverStation.Alliance.Red)
        ? HighAltitudeConstants.PATHFINDING_BLUE_BRANCHES
        : HighAltitudeConstants.PATHFINDING_RED_BRANCHES;

    if (pos == null) {
      var targetID = Robot.getRobotContainer().getVision().getTargetID();
      if (targetID == -1)
        return;

      for (int i = 0; i < reefIDs.length; i++) {
        if (targetID == reefIDs[i]) {
          pos = HighAltitudeConstants.REEF_POSITIONS[i];
          break;
        }
      }
    }
    
    targetPose = branches[pos.getBranchID(left)];

  }

  @Override
  public void execute() {
    if (pos == null) {
      determineTarget();
      if (pos == null)
        return;
    }
  }

  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getSwerveDriveTrain().stopModules();
  }

  @Override
  public boolean isFinished() {
    return Robot.getRobotContainer().getSwerveDriveTrain().AlignWithTargetPose(targetPose, maxLinearVelocity,
        maxAngularVelocity);
  }
}