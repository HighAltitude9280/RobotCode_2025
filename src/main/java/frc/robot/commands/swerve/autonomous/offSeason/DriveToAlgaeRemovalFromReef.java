// src/main/java/frc/robot/commands/swerve/autonomous/reef/DriveToAlgaeRemovalFromReef.java
package frc.robot.commands.swerve.autonomous.offSeason;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants; // timeouts, lock cycles, REEF_HEIGHT
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.HighAltitudeConstantsPose; // tags + per-tag poses por alianza
import frc.robot.Robot;
import frc.robot.resources.math.PoseUtil;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

/**
 * Vision-driven approach to the reef for algae removal. - Locks a detected AprilTag (anti
 * ping-pong). - Maps tag→(level L2/L3) depending on alliance. - Picks a base pose from
 * HighAltitudeConstantsPose and applies a forward/backoff offset. - Drives using
 * SwerveDriveTrain.AlignWithTargetPose(Pose2d, vMax, wMax).
 */
public class DriveToAlgaeRemovalFromReef extends Command {

  private final double vMax;
  private final double wMax;
  private final double offsetMeters; // +: approach, -: backoff

  private SwerveDriveTrain swerve;
  private Pose2d targetPose; // resolved base pose + offset
  private REEF_HEIGHT detectedLevel = null; // L2 or L3 according to tag
  private boolean finished = false;
  private boolean abort = false;

  // Tag locking
  private double startTs;
  private int stableDetectCount = 0;
  private Integer lockedTagId = null;
  private Integer lastSeenId = null;

  public DriveToAlgaeRemovalFromReef(double vMax, double wMax, double offsetMeters) {
    this.vMax = vMax;
    this.wMax = wMax;
    this.offsetMeters = offsetMeters;
    addRequirements(Robot.getRobotContainer().getSwerveDriveTrain());
  }

  @Override
  public void initialize() {
    swerve = Robot.getRobotContainer().getSwerveDriveTrain();
    finished = false;
    abort = false;
    startTs = Timer.getFPGATimestamp();
    stableDetectCount = 0;
    lockedTagId = null;
    lastSeenId = null;
    targetPose = null;
    detectedLevel = null;
  }

  @Override
  public void execute() {
    if (abort)
      return;

    // Hard timeout guard
    if (Timer.getFPGATimestamp() - startTs > HighAltitudeConstants.COMMAND_TIMEOUT_SEC) {
      abort = true;
      finished = true;
      swerve.stopModules();
      return;
    }

    // 1) Lock a stable AprilTag id
    if (lockedTagId == null) {
      int seen = Robot.getRobotContainer().getVision().getTargetID();
      if (seen <= 0) {
        stableDetectCount = 0;
        lastSeenId = null;
        return;
      }

      if (lastSeenId == null || seen != lastSeenId) {
        lastSeenId = seen;
        stableDetectCount = 1;
      } else {
        stableDetectCount++;
      }

      if (stableDetectCount < HighAltitudeConstants.TAG_DETECTION_LOCK_CYCLES)
        return;

      lockedTagId = seen;

      // 2) Resolve base pose and level from the locked tag
      resolvePoseAndLevelFromTag(lockedTagId);
      if (targetPose == null || detectedLevel == null) {
        abort = true;
        finished = true;
        swerve.stopModules();
        return;
      }
    }

    // 3) Drive to the target pose
    finished = swerve.AlignWithTargetPose(targetPose, vMax, wMax);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return finished || abort;
  }

  // ---------------- Helpers ads

  private void resolvePoseAndLevelFromTag(int tagId) {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    if (alliance == DriverStation.Alliance.Blue) {
      int idx;

      idx = indexOf(HighAltitudeConstantsPose.BLUE_ALGAE_L3_TAGS, tagId);
      if (idx >= 0) {
        Pose2d base = HighAltitudeConstantsPose.BLUE_ALGAE_L3_POSES[idx];
        targetPose = PoseUtil.offsetLocal(base, offsetMeters, 0.0);
        detectedLevel = REEF_HEIGHT.L3;
        return;
      }

      idx = indexOf(HighAltitudeConstantsPose.BLUE_ALGAE_L2_TAGS, tagId);
      if (idx >= 0) {
        Pose2d base = HighAltitudeConstantsPose.BLUE_ALGAE_L2_POSES[idx];
        targetPose = PoseUtil.offsetLocal(base, offsetMeters, 0.0);
        detectedLevel = REEF_HEIGHT.L2;
        return;
      }
    } else { // RED
      int idx;

      idx = indexOf(HighAltitudeConstantsPose.RED_ALGAE_L3_TAGS, tagId);
      if (idx >= 0) {
        Pose2d base = HighAltitudeConstantsPose.RED_ALGAE_L3_POSES[idx];
        targetPose = PoseUtil.offsetLocal(base, offsetMeters, 0.0);
        detectedLevel = REEF_HEIGHT.L3;
        return;
      }

      idx = indexOf(HighAltitudeConstantsPose.RED_ALGAE_L2_TAGS, tagId);
      if (idx >= 0) {
        Pose2d base = HighAltitudeConstantsPose.RED_ALGAE_L2_POSES[idx];
        targetPose = PoseUtil.offsetLocal(base, offsetMeters, 0.0);
        detectedLevel = REEF_HEIGHT.L2;
        return;
      }
    }

    // no match = abort
    targetPose = null;
    detectedLevel = null;
  }

  private static int indexOf(int[] arr, int val) {
    for (int i = 0; i < arr.length; i++)
      if (arr[i] == val)
        return i;
    return -1;
  }

  /** Returns the detected level (L2/L3) after tag lock; null until then. */
  public REEF_HEIGHT getDetectedLevel() {
    return detectedLevel;
  }

  /** Returns the locked tag id once acquired; null until then. */
  public Integer getLockedTagId() {
    return lockedTagId;
  }

  /** Returns the target pose (base+offset) once resolved; null until then. */
  public Pose2d getTargetPose() {
    return targetPose;
  }
}
