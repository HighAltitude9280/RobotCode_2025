// src/main/java/frc/robot/commands/swerve/autonomous/offSeason/AlignWithTargetPose.java
package frc.robot.commands.swerve.autonomous.offSeason;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstantsPose;
import frc.robot.HighAltitudeConstantsPose.REEF_POSITION;
import frc.robot.HighAltitudeConstantsPose.REEF_SIDE;
import frc.robot.Robot;
import frc.robot.resources.math.PoseUtil;

/**
 * Aligns the robot to a target reef branch pose using field-relative PID. Supports three modes: 1)
 * Explicit position (pos!=null) and branch side (left). 2) From side (side!=null) + front/back ring
 * via Robot.isFrontMode(). 3) Auto-detect via AprilTag ID (pos==null && side==null) using alliance
 * tag arrays.
 *
 * If vision isn't ready at init, the command waits (with safety timeout), trying to resolve pos and
 * the final targetPose every execute() cycle.
 */
public class AlignWithTargetPose extends Command {

  // Selection inputs (may be null in auto-detect mode)
  private REEF_POSITION pos;
  private final REEF_SIDE side;
  private Boolean left;

  // Limits
  private final double maxLinearVelocity;
  private final double maxAngularVelocity;

  // Target & state
  private Pose2d targetPose;
  private boolean isFinished = false;
  private double startTsSec;

  // Optional approach backoff (offset along target heading)
  private final boolean applyBackoff;

  // ---------------------- Constructors ----------------------

  /** Full constructor with explicit position/side/branch. */
  public AlignWithTargetPose(REEF_POSITION position, REEF_SIDE side, Boolean left,
      double maxLinearVelocity, double maxAngularVelocity) {
    this(position, side, left, maxLinearVelocity, maxAngularVelocity, false);
  }

  /** Full constructor with explicit position/side/branch and backoff toggle. */
  public AlignWithTargetPose(REEF_POSITION position, REEF_SIDE side, Boolean left,
      double maxLinearVelocity, double maxAngularVelocity, boolean applyBackoff) {
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());
    this.pos = position;
    this.side = side;
    this.left = left;
    this.maxLinearVelocity = maxLinearVelocity;
    this.maxAngularVelocity = maxAngularVelocity;
    this.applyBackoff = applyBackoff;
  }

  /**
   * NEW: Auto-detect constructor. Reads AprilTag → maps to REEF_POSITION and branch (left/right),
   * then aligns. Pass left=null to use Robot.isLeftMode().
   */
  public AlignWithTargetPose(Boolean left, double maxLinearVelocity, double maxAngularVelocity,
      Boolean applyBackoff) {
    this(null, /* position */
        null, /* side */
        left, maxLinearVelocity, maxAngularVelocity, applyBackoff != null && applyBackoff);
  }

  // ---------------------- Command lifecycle ----------------------

  @Override
  public void initialize() {
    isFinished = false;
    startTsSec = Timer.getFPGATimestamp();

    // Decide branch side from parameter or robot mode
    left = (left != null) ? left : Robot.isLeftMode();

    // If only side is given, derive pos from front/back mode
    if (pos == null && side != null) {
      pos = side.getPosition(Robot.isFrontMode());
    }

    // Context logs
    DriverStation.Alliance alliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    SmartDashboard.putString("Align/Alliance", alliance.toString());
    SmartDashboard.putBoolean("Align/LeftBranch", left);
    SmartDashboard.putString("Align/ReefSide", side != null ? side.name() : "null");
    SmartDashboard.putString("Align/InitPos", pos != null ? pos.name() : "null");
    SmartDashboard.putNumber("Align/StartTs", startTsSec);

    // Try determine target once (do NOT abort if null; keep waiting)
    determineTarget();
    if (pos == null || targetPose == null) {
      SmartDashboard.putString("Align/Status", "Waiting for target (init)...");
    } else {
      SmartDashboard.putString("Align/Status", "Target locked (init)");
      SmartDashboard.putNumber("Align/Target/X", targetPose.getX());
      SmartDashboard.putNumber("Align/Target/Y", targetPose.getY());
      SmartDashboard.putNumber("Align/Target/RotDeg", targetPose.getRotation().getDegrees());
    }
  }

  @Override
  public void execute() {
    if (isFinished)
      return;

    // Safety timeout
    double elapsed = Timer.getFPGATimestamp() - startTsSec;
    SmartDashboard.putNumber("Align/ElapsedSec", elapsed);
    if (elapsed > HighAltitudeConstants.COMMAND_TIMEOUT_SEC) {
      SmartDashboard.putString("Align/Status", "Timeout -> finish");
      isFinished = true;
      return;
    }

    // Ensure we have a valid target; if not, keep waiting
    if (pos == null || targetPose == null) {
      determineTarget();
      if (pos == null || targetPose == null) {
        SmartDashboard.putString("Align/Status", "Waiting for target...");
        return; // skip drive this cycle
      }
    }

    // Optional backoff on final pose (approach offset along heading)
    Pose2d poseToUse = targetPose;
    if (applyBackoff && poseToUse != null) {
      poseToUse = PoseUtil.backoff(poseToUse, HighAltitudeConstants.CORAL_BACKOFF_M);
      SmartDashboard.putBoolean("Align/BackoffApplied", true);
    } else {
      SmartDashboard.putBoolean("Align/BackoffApplied", false);
    }

    // Log odometry, target and error
    Pose2d cur = Robot.getRobotContainer().getSwerveDriveTrain().getPose();
    SmartDashboard.putNumber("Align/Odo/X", cur.getX());
    SmartDashboard.putNumber("Align/Odo/Y", cur.getY());
    SmartDashboard.putNumber("Align/Odo/RotDeg", cur.getRotation().getDegrees());
    SmartDashboard.putNumber("Align/Use/X", poseToUse.getX());
    SmartDashboard.putNumber("Align/Use/Y", poseToUse.getY());
    SmartDashboard.putNumber("Align/Use/RotDeg", poseToUse.getRotation().getDegrees());
    double dx = poseToUse.getX() - cur.getX();
    double dy = poseToUse.getY() - cur.getY();
    double dist = Math.hypot(dx, dy);
    double dYaw = poseToUse.getRotation().minus(cur.getRotation()).getDegrees();
    SmartDashboard.putNumber("Align/Error/DistM", dist);
    SmartDashboard.putNumber("Align/Error/AngleDeg", dYaw);

    // Drive alignment
    boolean reached = Robot.getRobotContainer().getSwerveDriveTrain().AlignWithTargetPose(poseToUse,
        maxLinearVelocity, maxAngularVelocity);

    SmartDashboard.putBoolean("Align/Reached", reached);
    SmartDashboard.putString("Align/Status", reached ? "Reached" : "Tracking");
    isFinished = reached;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getSwerveDriveTrain().stopModules();
    SmartDashboard.putString("Align/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Align/isFinished", isFinished);
    return isFinished;
  }

  // ---------------------- Helpers ----------------------

  /** Resolve pos/targetPose from explicit args or AprilTag mapping. */
  private void determineTarget() {
    DriverStation.Alliance alliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    // Select mapping tables by alliance
    int[] reefIDs =
        (alliance == DriverStation.Alliance.Red) ? HighAltitudeConstantsPose.RED_APRILTAG_IDS
            : HighAltitudeConstantsPose.BLUE_APRILTAG_IDS;
    Pose2d[] branches = (alliance == DriverStation.Alliance.Red)
        ? HighAltitudeConstantsPose.PATHFINDING_RED_BRANCHES
        : HighAltitudeConstantsPose.PATHFINDING_BLUE_BRANCHES;

    SmartDashboard.putString("Align/Det/Alliance", alliance.toString());
    SmartDashboard.putString("Align/Det/BranchTable",
        (alliance == DriverStation.Alliance.Red) ? "RED_BRANCHES" : "BLUE_BRANCHES");
    SmartDashboard.putString("Align/Det/PosIn", pos != null ? pos.name() : "null");
    SmartDashboard.putBoolean("Align/Det/Left", left);

    // If position is not preset, try to detect via AprilTag
    if (pos == null) {
      int id = Robot.getRobotContainer().getVision().getTargetID();
      SmartDashboard.putNumber("Align/Det/TargetID", id);

      if (id > 0) {
        for (int i = 0; i < reefIDs.length; i++) {
          if (id == reefIDs[i]) {
            pos = HighAltitudeConstantsPose.REEF_POSITIONS[i];
            break;
          }
        }
      }
    }

    // Build target pose if we have a valid position
    if (pos != null) {
      int branchIndex = pos.getBranchID(left);
      SmartDashboard.putNumber("Align/Det/BranchIndex", branchIndex);

      if (branchIndex >= 0 && branchIndex < branches.length) {
        targetPose = branches[branchIndex];
        SmartDashboard.putNumber("Align/Det/TargetX", targetPose.getX());
        SmartDashboard.putNumber("Align/Det/TargetY", targetPose.getY());
        SmartDashboard.putNumber("Align/Det/TargetRot", targetPose.getRotation().getDegrees());
        SmartDashboard.putString("Align/Det/State", "Pose set");
      } else {
        targetPose = null;
        SmartDashboard.putString("Align/Det/State", "Branch OOB");
      }
    } else {
      targetPose = null;
      SmartDashboard.putString("Align/Det/State", "pos null (no tag yet)");
    }
  }
}
