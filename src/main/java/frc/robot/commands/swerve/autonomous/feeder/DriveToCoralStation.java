// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.autonomous.feeder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.CORAL_STATION_POSITION;
import frc.robot.Robot;

public class DriveToCoralStation extends Command {

  // Configurable command parameters
  private CORAL_STATION_POSITION pos;
  /**
   * This parameter, if not null, forces the selection:
   * true = left option;
   * false = right option.
   * If null, the option closest to the current robot pose is chosen (teleoperated
   * mode).
   */
  private Boolean left;
  private final double maxLinearVelocity, maxAngularVelocity;

  // Target pose to drive the robot to
  private Pose2d targetPose;
  private boolean isFinished = false;

  /**
   * Constructs a command to drive the robot to the Coral Station.
   *
   * @param position           The desired position (FAR, MIDDLE, or NEAR).
   *                           Defaults to MIDDLE if null.
   * @param left               If specified, forces the left (true) or right
   *                           (false) option.
   *                           If null, the closest option is selected
   *                           automatically.
   * @param maxLinearVelocity  Maximum linear velocity in m/s.
   * @param maxAngularVelocity Maximum angular velocity in rad/s.
   */
  public DriveToCoralStation(CORAL_STATION_POSITION position, Boolean left,
      double maxLinearVelocity, double maxAngularVelocity) {
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());
    this.pos = position;
    this.left = left;
    this.maxLinearVelocity = maxLinearVelocity;
    this.maxAngularVelocity = maxAngularVelocity;
  }

  @Override
  public void initialize() {
    // Assign default position if none is specified
    if (pos == null) {
      pos = CORAL_STATION_POSITION.MIDDLE;
    }
    // Determine the target pose using the modularized method
    targetPose = determineTargetPose();
    if (targetPose == null) {
      // In case of error, use the fallback pose and log the error
      System.err.println("DriveToCoralStation: targetPose is null, using fallback.");
      targetPose = fallbackTargetPose();
    }
    // Publish tuning values to SmartDashboard for debugging and tuning
    putTunningValues();
  }

  /**
   * Determines the target pose based on the configuration and current state of
   * the robot.
   * It uses the position parameter to get the corresponding index and,
   * based on the alliance, looks up the two candidate poses (left and right).
   * If 'left' is null, the candidate that is closest to the current robot pose is
   * chosen.
   *
   * @return The target pose or null if an error occurs.
   */
  private Pose2d determineTargetPose() {
    int index = pos.getID();
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    Pose2d candidateLeft = null;
    Pose2d candidateRight = null;
    try {
      switch (alliance) {
        case Blue:
          if (index < HighAltitudeConstants.PATHFINDING_BLUE_LEFT_CORAL_STATION.length) {
            candidateLeft = HighAltitudeConstants.PATHFINDING_BLUE_LEFT_CORAL_STATION[index];
          }
          if (index < HighAltitudeConstants.PATHFINDING_BLUE_RIGHT_CORAL_STATION.length) {
            candidateRight = HighAltitudeConstants.PATHFINDING_BLUE_RIGHT_CORAL_STATION[index];
          }
          break;
        case Red:
          if (index < HighAltitudeConstants.PATHFINDING_RED_LEFT_CORAL_STATION.length) {
            candidateLeft = HighAltitudeConstants.PATHFINDING_RED_LEFT_CORAL_STATION[index];
          }
          if (index < HighAltitudeConstants.PATHFINDING_RED_RIGHT_CORAL_STATION.length) {
            candidateRight = HighAltitudeConstants.PATHFINDING_RED_RIGHT_CORAL_STATION[index];
          }
          break;
        default:
          // If the alliance is not recognized, default to Blue
          if (index < HighAltitudeConstants.PATHFINDING_BLUE_LEFT_CORAL_STATION.length) {
            candidateLeft = HighAltitudeConstants.PATHFINDING_BLUE_LEFT_CORAL_STATION[index];
          }
          if (index < HighAltitudeConstants.PATHFINDING_BLUE_RIGHT_CORAL_STATION.length) {
            candidateRight = HighAltitudeConstants.PATHFINDING_BLUE_RIGHT_CORAL_STATION[index];
          }
          break;
      }
    } catch (Exception e) {
      System.err.println("Error obtaining pose candidates: " + e.getMessage());
      return null;
    }

    if (candidateLeft == null && candidateRight == null) {
      System.err.println("No candidate poses available for alliance " + alliance);
      return null;
    }

    // If the 'left' parameter is specified, force the selection.
    if (left != null) {
      return left ? candidateLeft : candidateRight;
    } else {
      // Teleoperated mode: select the candidate closest to the current robot pose.
      Pose2d currentPose = Robot.getRobotContainer().getSwerveDriveTrain().getPose();
      if (currentPose == null) {
        System.err.println("Current robot pose is null; cannot compare distances.");
        return candidateLeft != null ? candidateLeft : candidateRight;
      }
      double distLeft = candidateLeft != null
          ? candidateLeft.getTranslation().getDistance(currentPose.getTranslation())
          : Double.MAX_VALUE;
      double distRight = candidateRight != null
          ? candidateRight.getTranslation().getDistance(currentPose.getTranslation())
          : Double.MAX_VALUE;
      return (distLeft <= distRight) ? candidateLeft : candidateRight;
    }
  }

  /**
   * Fallback to obtain a target pose in case determineTargetPose() fails.
   *
   * @return A backup pose; in this case, the left option for the determined
   *         alliance.
   */
  private Pose2d fallbackTargetPose() {
    int index = pos.getID();
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    try {
      switch (alliance) {
        case Blue:
          if (index < HighAltitudeConstants.PATHFINDING_BLUE_LEFT_CORAL_STATION.length) {
            return HighAltitudeConstants.PATHFINDING_BLUE_LEFT_CORAL_STATION[index];
          }
          break;
        case Red:
          if (index < HighAltitudeConstants.PATHFINDING_RED_LEFT_CORAL_STATION.length) {
            return HighAltitudeConstants.PATHFINDING_RED_LEFT_CORAL_STATION[index];
          }
          break;
        default:
          if (index < HighAltitudeConstants.PATHFINDING_BLUE_LEFT_CORAL_STATION.length) {
            return HighAltitudeConstants.PATHFINDING_BLUE_LEFT_CORAL_STATION[index];
          }
          break;
      }
    } catch (Exception e) {
      System.err.println("Error in fallbackTargetPose: " + e.getMessage());
    }
    // As a last resort, return a default pose at (0,0,0)
    return new Pose2d();
  }

  /**
   * Sends the target pose values to SmartDashboard to aid in tuning and
   * debugging.
   * Publishes: X, Y, rotation (in degrees), and a full string representation of
   * the pose.
   * This method is called in initialize() so the values appear as soon as the
   * command starts.
   */
  private void putTunningValues() {
    if (targetPose == null)
      return;
    SmartDashboard.putNumber("CoralStation TargetPoseX", targetPose.getX());
    SmartDashboard.putNumber("CoralStation TargetPoseY", targetPose.getY());
    SmartDashboard.putNumber("CoralStation TargetPoseRotationDegrees", targetPose.getRotation().getDegrees());
    SmartDashboard.putString("CoralStation TargetPoseFull", targetPose.toString());
  }

  @Override
  public void execute() {
    // Drive towards targetPose using the swerve drive alignment method.
    isFinished = Robot.getRobotContainer().getSwerveDriveTrain()
        .AlignWithTargetPose(targetPose, maxLinearVelocity, maxAngularVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop all drive modules to avoid residual movement.
    Robot.getRobotContainer().getSwerveDriveTrain().stopModules();
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
