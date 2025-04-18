// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.autonomous.reef;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.Robot;
import frc.robot.HighAltitudeConstants.REEF_POSITION;
import frc.robot.HighAltitudeConstants.REEF_SIDE;

public class AlignWithTargetPose extends Command {

  private REEF_POSITION pos;
  private REEF_SIDE side;
  private Boolean left;
  private final double maxLinearVelocity, maxAngularVelocity;
  private Pose2d targetPose;
  private boolean isFinished = false;

  /**
   * Command to align the robot using pose.
   *
   * @param position           The position to align the robot to (nullable for
   *                           detection mode).
   * @param side               The reef side (nullable for detection mode).
   * @param left               True for left branch, false for right (nullable for
   *                           detection mode).
   * @param maxLinearVelocity  Max linear velocity in m/s.
   * @param maxAngularVelocity Max angular velocity in rad/s.
   */
  public AlignWithTargetPose(REEF_POSITION position, REEF_SIDE side, Boolean left,
      double maxLinearVelocity, double maxAngularVelocity) {
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());
    this.pos = position;
    this.side = side;
    this.left = left;
    this.maxAngularVelocity = maxAngularVelocity;
    this.maxLinearVelocity = maxLinearVelocity;
  }

  @Override
  public void initialize() {
    // Determine branch side: use provided or default
    left = (left != null) ? left : Robot.isLeftMode();
    // Determine pos from side if needed
    if (pos == null && side != null) {
      pos = side.getPosition(Robot.isFrontMode());
    }

    // Log alliance, branch, and side
    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    SmartDashboard.putString("Align/alliance", alliance.toString());
    SmartDashboard.putBoolean("Align/leftBranch", left);
    SmartDashboard.putString("Align/reefSide", side != null ? side.name() : "null");

    // Determine the target pose
    determineTarget();

    // If no valid target, finish immediately
    if (pos == null || targetPose == null) {
      determineTarget();
      SmartDashboard.putString("Align/status", "No valid target, ending");
    } else {
      SmartDashboard.putString("Align/status", "Target locked");
    }
  }

  private void determineTarget() {
    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    int[] reefIDs = (alliance == DriverStation.Alliance.Red)
        ? HighAltitudeConstants.RED_APRILTAG_IDS
        : HighAltitudeConstants.BLUE_APRILTAG_IDS;
    var branches = (alliance == DriverStation.Alliance.Red)
        ? HighAltitudeConstants.PATHFINDING_RED_BRANCHES
        : HighAltitudeConstants.PATHFINDING_BLUE_BRANCHES;

    // Log detection mode
    SmartDashboard.putString("Align/determine/alliance", alliance.toString());
    SmartDashboard.putBoolean("Align/determine/leftBranch", left);
    SmartDashboard.putString("Align/determine/reefSide", side != null ? side.name() : "null");

    // If position not given, detect via AprilTag
    if (pos == null) {
      int targetID = Robot.getRobotContainer().getVision().getTargetID();
      SmartDashboard.putNumber("Align/determine/targetID", targetID);
      for (int i = 0; i < reefIDs.length; i++) {
        if (targetID == reefIDs[i]) {
          pos = HighAltitudeConstants.REEF_POSITIONS[i];
          break;
        }
      }
    }

    if (pos != null) {
      int branchIndex = pos.getBranchID(left);
      targetPose = branches[branchIndex];
      System.out.println(pos);
    } else {
      System.out.println("AAAAAAAAAAAA");
      targetPose = null;
    }
    System.out.println("INIT:" + targetPose);
    System.out.println("INIT:" + Robot.getRobotContainer().getVision().getTargetID());
  }

  @Override
  public void execute() {
    if (isFinished) {
      return;
    }
    // Ensure we have a valid target on each execute
    if (pos == null || targetPose == null) {
      determineTarget();
      if (pos == null || targetPose == null) {
        determineTarget();
        return;
      }
    }
    // Align with target
    isFinished = Robot.getRobotContainer()
        .getSwerveDriveTrain()
        .AlignWithTargetPose(targetPose, maxLinearVelocity, maxAngularVelocity);
    System.out.println("AAAAA Tupu");
    System.out.println(targetPose);
    System.out.println(Robot.getRobotContainer().getVision().getTargetID());
  }

  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getSwerveDriveTrain().stopModules();
    SmartDashboard.putString("Align/status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {

    return isFinished;
  }
}
