// src/main/java/frc/robot/commands/swerve/autonomous/offSeason/DriveToAlgaeRemovalFromReef.java
package frc.robot.commands.swerve.autonomous.offSeason;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants; // timeouts, lock cycles, LATCHED_* y VISION_* tunables
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.HighAltitudeConstantsPose;
import frc.robot.Robot;
import frc.robot.resources.math.PoseUtil;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.subsystems.vision.LastPoseCache;

public class DriveToAlgaeRemovalFromReef extends Command {

  private final double vMax;
  private final double wMax;
  private final double offsetMeters; // + approach, - backoff

  private SwerveDriveTrain swerve;
  private Pose2d targetPose; // base + offset
  private REEF_HEIGHT detectedLevel = null; // L2 o L3 según tag
  private boolean finished = false;
  private boolean abort = false;

  // Vision locking
  private double startTs;
  private int stableDetectCount = 0;
  private Integer lockedTagId = null;
  private Integer lastSeenId = null;

  // Cache de pose para tolerar pérdidas breves de visión
  private final LastPoseCache cache = new LastPoseCache();

  // Logs mínimos
  private static final boolean DEBUG = true; // pon en false para silenciar

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

    dbg("Algae/Status", "Init");
  }

  @Override
  public void execute() {
    if (abort) {
      dbg("Algae/Status", "Aborted");
      return;
    }

    // Timeout interno
    double elapsed = Timer.getFPGATimestamp() - startTs;
    if (elapsed > HighAltitudeConstants.COMMAND_TIMEOUT_SEC) {
      abortAndStop("Timeout");
      return;
    }
    dbg("Algae/ElapsedSec", elapsed);

    // 1) Intento de lock con ID confiable (Vision reliable + TTL)
    if (lockedTagId == null) {
      int seen = Robot.getRobotContainer().getVision().getAlignmentTargetIdReliable();
      dbg("Algae/ID", seen);

      if (seen <= 0) {
        useLatchedIfSafeOrKeepWaiting();
        return;
      }

      if (lastSeenId == null || seen != lastSeenId) {
        lastSeenId = seen;
        stableDetectCount = 1;
      } else {
        stableDetectCount++;
      }

      if (stableDetectCount < HighAltitudeConstants.TAG_DETECTION_LOCK_CYCLES) {
        dbg("Algae/Locking", stableDetectCount);
        return;
      }

      lockedTagId = seen;
      dbg("Algae/LockedId", lockedTagId);

      // 2) Resolver pose base y nivel a partir del tag bloqueado
      resolvePoseAndLevelFromTag(lockedTagId);
      if (targetPose == null || detectedLevel == null) {
        abortAndStop("Tag not mapped");
        return;
      }
      cache.update(targetPose);
      dbg("Algae/Level", detectedLevel.name());
      dbg("Algae/TargetX", targetPose.getX());
      dbg("Algae/TargetY", targetPose.getY());
      dbg("Algae/TargetRotDeg", targetPose.getRotation().getDegrees());
    } else if (targetPose == null) {
      // Teníamos lock pero no pose (raro) → re-resolver o latched
      resolvePoseAndLevelFromTag(lockedTagId);
      if (targetPose == null) {
        useLatchedIfSafeOrKeepWaiting();
        if (targetPose == null)
          return;
      }
    }

    // 3) Drive al target
    boolean reached = swerve.AlignWithTargetPose(targetPose, vMax, wMax);
    dbg("Algae/Reached", reached);
    finished = reached;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
    dbg("Algae/Status", interrupted ? "Interrupted" : "Finished");
  }

  @Override
  public boolean isFinished() {
    return finished || abort;
  }

  // ---------------- Helpers ----------------

  private void resolvePoseAndLevelFromTag(int tagId) {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    if (alliance == DriverStation.Alliance.Blue) {
      int idx = indexOf(HighAltitudeConstantsPose.BLUE_ALGAE_L3_TAGS, tagId);
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
      int idx = indexOf(HighAltitudeConstantsPose.RED_ALGAE_L3_TAGS, tagId);
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

    // Sin mapeo
    targetPose = null;
    detectedLevel = null;
  }

  private void useLatchedIfSafeOrKeepWaiting() {
    var opt = cache.getIfFresh(HighAltitudeConstants.LATCHED_POSE_MAX_AGE_SEC);
    if (opt.isEmpty())
      return;

    double age = cache.ageSec();
    var speeds = swerve.getChassisSpeeds();
    double v = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double drift = v * age;
    double omegaDeg = Math.toDegrees(Math.abs(speeds.omegaRadiansPerSecond) * age);

    boolean ok = drift <= HighAltitudeConstants.LATCHED_POSE_MAX_TRANSLATION_DRIFT_M
        && omegaDeg <= HighAltitudeConstants.LATCHED_POSE_MAX_HEADING_DRIFT_DEG;

    dbg("Algae/LatchedAgeMs", age * 1000.0);
    dbg("Algae/LatchedAccept", ok);

    if (ok) {
      targetPose = opt.get();
      dbg("Algae/Status", "Using latched");
    }
  }

  private static int indexOf(int[] arr, int val) {
    for (int i = 0; i < arr.length; i++)
      if (arr[i] == val)
        return i;
    return -1;
  }

  private void abortAndStop(String reason) {
    abort = true;
    finished = true;
    swerve.stopModules();
    dbg("Algae/Status", "Abort: " + reason);
  }

  // ---------------- GETTERS (explícitos + alias) ----------------

  /** Nivel detectado (L2/L3) tras lock; null hasta entonces. */
  public REEF_HEIGHT getAlgaeRemovalDetectedLevel() {
    return detectedLevel;
  }

  /** Tag id bloqueado; null hasta tener lock. */
  public Integer getAlgaeRemovalLockedTagId() {
    return lockedTagId;
  }

  /** Pose objetivo (base+offset) resuelta; null hasta tener lock. */
  public Pose2d getAlgaeRemovalTargetPose() {
    return targetPose;
  }


  public int getAlgaeRemovalPoseIdx() {
    if (detectedLevel == null)
      return -1; // sentinel: aún no hay lock
    return (detectedLevel == REEF_HEIGHT.L3)
        ? frc.robot.HighAltitudeConstants.PoseIdx.ALGAE_REMOVE_L3
        : frc.robot.HighAltitudeConstants.PoseIdx.ALGAE_REMOVE_L2;
  }

  // ---- Alias legacy (por compatibilidad). Si quieres, márcalos @Deprecated. ----
  public REEF_HEIGHT getDetectedLevel() {
    return getAlgaeRemovalDetectedLevel();
  }

  public Integer getLockedTagId() {
    return getAlgaeRemovalLockedTagId();
  }

  public Pose2d getTargetPose() {
    return getAlgaeRemovalTargetPose();
  }

  public int getPoseIdx() {
    return getAlgaeRemovalPoseIdx();
  }


  // ---- helpers de logging minimal ----
  private static void dbg(String k, String v) {
    if (DEBUG)
      SmartDashboard.putString(k, v);
  }

  private static void dbg(String k, boolean v) {
    if (DEBUG)
      SmartDashboard.putBoolean(k, v);
  }

  private static void dbg(String k, double v) {
    if (DEBUG)
      SmartDashboard.putNumber(k, v);
  }
}
