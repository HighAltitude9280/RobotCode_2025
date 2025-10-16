// src/main/java/frc/robot/commands/swerve/autonomous/offSeason/DriveToCoralStation.java
package frc.robot.commands.swerve.autonomous.offSeason;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstantsPose;
import frc.robot.HighAltitudeConstantsPose.CORAL_STATION_POSITION;
import frc.robot.Robot;

/**
 * Drives to a Coral Station approach pose chosen from field constants.
 * - Alliance-based lookup (Blue/Red).
 * - Left/Right station side selection (forced or auto by proximity).
 * - Position index within the station (FAR/MIDDLE/NEAR).
 *
 * This command trusts the pre-baked Pose2d targets; no vision/determineTarget.
 */
public class DriveToCoralStation extends Command {

  // Parameters
  private CORAL_STATION_POSITION pos;
  /**
   * If not null: true = left station, false = right station.
   * If null: picks whichever is closer to the current robot pose.
   */
  private final Boolean left;
  private final double maxLinearVelocity;
  private final double maxAngularVelocity;

  // State
  private Pose2d targetPose;
  private boolean done;

  /**
   * Original constructor:
   * 
   * @param position           FAR, MIDDLE, or NEAR (defaults to MIDDLE if null).
   * @param left               Force left(true)/right(false); if null, auto-pick
   *                           closest. AUTO ONLY
   * @param maxLinearVelocity  Max linear speed (m/s).
   * @param maxAngularVelocity Max angular speed (rad/s).
   */
  public DriveToCoralStation(CORAL_STATION_POSITION position, Boolean left,
      double maxLinearVelocity, double maxAngularVelocity) {
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());
    this.pos = position;
    this.left = left;
    this.maxLinearVelocity = maxLinearVelocity;
    this.maxAngularVelocity = maxAngularVelocity;
  }

  /**
   * Convenience constructor (For TeleOp):
   * Only specify left/right. Position defaults to MIDDLE.
   * 
   * @param left               true = left station, false = right station.
   * @param maxLinearVelocity  Max linear speed (m/s).
   * @param maxAngularVelocity Max angular speed (rad/s).
   */
  public DriveToCoralStation(boolean left,
      double maxLinearVelocity, double maxAngularVelocity) {
    this(CORAL_STATION_POSITION.MIDDLE, Boolean.valueOf(left), maxLinearVelocity, maxAngularVelocity);
  }

  @Override
  public void initialize() {
    done = false;

    // Default position if not provided
    if (pos == null)
      pos = CORAL_STATION_POSITION.MIDDLE;

    targetPose = pickTargetPose();
    if (targetPose == null) {
      System.err.println("[DriveToCoralStation] targetPose is null, falling back to (0,0,0).");
      targetPose = new Pose2d(); // very last resort
    }

    pushDebug();
  }

  @Override
  public void execute() {
    done = Robot.getRobotContainer().getSwerveDriveTrain()
        .AlignWithTargetPose(targetPose, maxLinearVelocity, maxAngularVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getSwerveDriveTrain().stopModules();
  }

  @Override
  public boolean isFinished() {
    return done;
  }

  // ---------------- Internals ----------------

  /**
   * Selects the station approach pose:
   * - Reads alliance to choose tables.
   * - Retrieves left/right candidate by position index.
   * - If 'left' is null, auto-picks closest to current pose.
   * - If forced side is null in tables, falls back to the other side.
   */
  private Pose2d pickTargetPose() {
    int idx = pos.getID();
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    Pose2d candLeft = null, candRight = null;

    switch (alliance) {
      case Red:
        if (idx < HighAltitudeConstantsPose.PATHFINDING_RED_LEFT_CORAL_STATION.length)
          candLeft = HighAltitudeConstantsPose.PATHFINDING_RED_LEFT_CORAL_STATION[idx];
        if (idx < HighAltitudeConstantsPose.PATHFINDING_RED_RIGHT_CORAL_STATION.length)
          candRight = HighAltitudeConstantsPose.PATHFINDING_RED_RIGHT_CORAL_STATION[idx];
        break;

      case Blue:
      default:
        if (idx < HighAltitudeConstantsPose.PATHFINDING_BLUE_LEFT_CORAL_STATION.length)
          candLeft = HighAltitudeConstantsPose.PATHFINDING_BLUE_LEFT_CORAL_STATION[idx];
        if (idx < HighAltitudeConstantsPose.PATHFINDING_BLUE_RIGHT_CORAL_STATION.length)
          candRight = HighAltitudeConstantsPose.PATHFINDING_BLUE_RIGHT_CORAL_STATION[idx];
        break;
    }

    // If both are missing, we cannot proceed.
    if (candLeft == null && candRight == null)
      return null;

    // Forced side
    if (left != null) {
      if (left.booleanValue()) {
        return (candLeft != null) ? candLeft : candRight; // fallback if left absent
      } else {
        return (candRight != null) ? candRight : candLeft; // fallback if right absent
      }
    }

    // Auto side by proximity to current pose
    Pose2d current = Robot.getRobotContainer().getSwerveDriveTrain().getPose();
    if (current == null)
      return (candLeft != null) ? candLeft : candRight;

    double dL = (candLeft != null) ? candLeft.getTranslation().getDistance(current.getTranslation()) : Double.MAX_VALUE;
    double dR = (candRight != null) ? candRight.getTranslation().getDistance(current.getTranslation())
        : Double.MAX_VALUE;

    return (dL <= dR) ? candLeft : candRight;
  }

  private void pushDebug() {
    if (targetPose == null)
      return;
    SmartDashboard.putNumber("CoralStation/TargetPoseX", targetPose.getX());
    SmartDashboard.putNumber("CoralStation/TargetPoseY", targetPose.getY());
    SmartDashboard.putNumber("CoralStation/TargetPoseDeg", targetPose.getRotation().getDegrees());
    SmartDashboard.putString("CoralStation/TargetPoseStr", targetPose.toString());
    SmartDashboard.putString("CoralStation/Alliance",
        DriverStation.getAlliance().map(Enum::name).orElse("Unknown"));
    SmartDashboard.putString("CoralStation/PosIndex", pos.name());
    SmartDashboard.putString("CoralStation/Side",
        left == null ? "AUTO" : (left.booleanValue() ? "LEFT" : "RIGHT"));
  }
}
