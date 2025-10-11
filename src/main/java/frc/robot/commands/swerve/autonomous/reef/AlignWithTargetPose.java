package frc.robot.commands.swerve.autonomous.reef;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.REEF_POSITION;
import frc.robot.HighAltitudeConstants.REEF_SIDE;
import frc.robot.Robot;
import frc.robot.subsystems.vision.LastPoseCache;

public class AlignWithTargetPose extends Command {

  private REEF_POSITION pos;
  private final REEF_SIDE side;
  private Boolean left;
  private final double maxLinearVelocity, maxAngularVelocity;

  private Pose2d targetPose;
  private boolean isFinished = false;
  private boolean abort = false;

  private final LastPoseCache cache = new LastPoseCache();
  private double startTs;
  private int stableDetectCount = 0;

  private Integer lockedTagId = null;  // anti ping-pong

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
    abort = false;
    isFinished = false;
    startTs = Timer.getFPGATimestamp();
    stableDetectCount = 0;
    lockedTagId = null;

    // Branch por default
    left = (left != null) ? left : Robot.isLeftMode();
    if (pos == null && side != null) pos = side.getPosition(Robot.isFrontMode());

    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    SmartDashboard.putString("Align/alliance", alliance.toString());
    SmartDashboard.putBoolean("Align/leftBranch", left);
    SmartDashboard.putString("Align/reefSide", side != null ? side.name() : "null");

    // 1) Intentar target fresca (preset o detección estable) y cachearla
    boolean hasFresh = determineTargetFreshAndCache();
    if (!hasFresh) {
      // 2) Intentar latched si es segura (TTL + deriva)
      targetPose = pickLatchedIfSafe();
    }

    if (targetPose == null) {
      DriverStation.reportWarning("[Align] No target nor safe cache at init; abort.", false);
      stopAndFinish();
      return;
    }
    SmartDashboard.putString("Align/status", hasFresh ? "Target locked" : "Using latched");
  }

  @Override
  public void execute() {
    if (abort) return;

    // Timeout duro
    if (Timer.getFPGATimestamp() - startTs > HighAltitudeConstants.COMMAND_TIMEOUT_SEC) {
      DriverStation.reportWarning("[Align] Timeout; stopping.", false);
      stopAndFinish();
      return;
    }

    // Intentar refrescar target fresca de nuevo (rápido cuando hay visión)
    boolean hasFresh = determineTargetFreshAndCache();
    if (!hasFresh) {
      targetPose = pickLatchedIfSafe();
      if (targetPose == null) {
        DriverStation.reportWarning("[Align] Target lost/expired; stopping.", false);
        stopAndFinish();
        return;
      }
      SmartDashboard.putString("Align/status", "Using latched");
    } else {
      SmartDashboard.putString("Align/status", "Target locked");
    }

    // Con target válida, alinear
    SmartDashboard.putNumber("Align/TargetAngleDeg", targetPose.getRotation().getDegrees());
    isFinished = Robot.getRobotContainer().getSwerveDriveTrain()
        .AlignWithTargetPose(targetPose, maxLinearVelocity, maxAngularVelocity);

    // Nota: si tu AlignWithTargetPose del drivetrain ya usa perfiles/FF,
    // esto quedará reactivo y suave; si no, considera un profiled PID para giro.
  }

  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getSwerveDriveTrain().stopModules();
    SmartDashboard.putString("Align/status", interrupted ? "Interrupted" : "Finished");
  }

  @Override
  public boolean isFinished() {
    return isFinished || abort;
  }

  // ---------- Helpers de fiabilidad/velocidad ----------

  /** Intenta fijar 'pos' y 'targetPose' frescos y actualizar caché. */
  private boolean determineTargetFreshAndCache() {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    int[] reefIDs = (alliance == DriverStation.Alliance.Red)
        ? HighAltitudeConstants.RED_APRILTAG_IDS
        : HighAltitudeConstants.BLUE_APRILTAG_IDS;
    var branches = (alliance == DriverStation.Alliance.Red)
        ? HighAltitudeConstants.PATHFINDING_RED_BRANCHES
        : HighAltitudeConstants.PATHFINDING_BLUE_BRANCHES;

    // Si ya tengo pos (preset o locked), solo construyo pose:
    if (pos == null) {
      int id = Robot.getRobotContainer().getVision().getTargetID();
      if (id <= 0) { stableDetectCount = 0; return false; }

      // Anti ping-pong: mantener id estable DETECT_STABLE_CYCLES
      if (lockedTagId != null && id != lockedTagId) {
        stableDetectCount = 0; // cambió temporalmente, no aceptes aún
        return false;
      }
      if (lockedTagId == null) {
        if (++stableDetectCount < HighAltitudeConstants.TAG_DETECTION_LOCK_CYCLES) return false;
        lockedTagId = id; // queda locked
      }

      // Mapear id→pos
      for (int i = 0; i < reefIDs.length; i++) {
        if (lockedTagId == reefIDs[i]) { pos = HighAltitudeConstants.REEF_POSITIONS[i]; break; }
      }
      if (pos == null) return false;
    }

    int branchIndex = pos.getBranchID(left);
    if (branchIndex < 0 || branchIndex >= branches.length) {
      DriverStation.reportWarning("[Align] branchIndex OOB: " + branchIndex, false);
      targetPose = null;
      return false;
    }

    targetPose = branches[branchIndex];
    cache.update(targetPose); // solo cuando la fuente es fresca/determinista
    SmartDashboard.putBoolean("Align/UsingCachedPose", false);
    return true;
  }

  /** Devuelve pose latched solo si cumple TTL y deriva cinemática. */
  //TODO: checar que funcione por getChassisSpeeds, que use la velocidad respecto al field, en vez de respecto al chassis
  private Pose2d pickLatchedIfSafe() {
    var opt = cache.getIfFresh(HighAltitudeConstants.LATCHED_POSE_MAX_AGE_SEC);
    if (opt.isEmpty()) return null;

    double age = cache.ageSec();
    var speeds = Robot.getRobotContainer().getSwerveDriveTrain().getChassisSpeeds();
    double v = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double drift = v * age;
    double omegaDriftDeg = Math.toDegrees(Math.abs(speeds.omegaRadiansPerSecond) * age);

    SmartDashboard.putNumber("Align/LatchedAgeMs", age * 1000.0);
    if (drift <= HighAltitudeConstants.LATCHED_POSE_MAX_TRANSLATION_DRIFT_M && omegaDriftDeg <= HighAltitudeConstants.LATCHED_POSE_MAX_HEADING_DRIFT_DEG) {
      SmartDashboard.putBoolean("Align/UsingCachedPose", true);
      return opt.get();
    }
    return null;
  }

  private void stopAndFinish() {
    Robot.getRobotContainer().getSwerveDriveTrain().stopModules();
    abort = true;
    isFinished = true;
  }
}
