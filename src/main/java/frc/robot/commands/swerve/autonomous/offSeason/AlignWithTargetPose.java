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

public class AlignWithTargetPose extends Command {

  // Entradas (pueden ser null en auto)
  private REEF_POSITION pos;
  private final REEF_SIDE side;
  private Boolean left;

  // Límites
  private final double maxLinearVelocity;
  private final double maxAngularVelocity;

  // Estado/target
  private Pose2d targetPose;
  private boolean isFinished = false;
  private double startTsSec;

  // Opcional: backoff
  private final boolean applyBackoff;

  // ---- Anti ping-pong + retarget ----
  private Integer lockedTagId = null; // tag en uso
  private Integer candidateTagId = null; // posible nuevo tag
  private int candidateStable = 0; // frames consecutivos del candidato
  private double lastFreshIdTs = -1; // último ts con visión de id/lock
  private static final double ID_TTL_SEC = 0.35; // aguantar lock sin visión (s)

  public AlignWithTargetPose(REEF_POSITION position, REEF_SIDE side, Boolean left,
      double maxLinearVelocity, double maxAngularVelocity) {
    this(position, side, left, maxLinearVelocity, maxAngularVelocity, false);
  }

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

  // Auto (pos/side null), left puede ser null para tomar Robot.isLeftMode()
  public AlignWithTargetPose(Boolean left, double maxLinearVelocity, double maxAngularVelocity,
      Boolean applyBackoff) {
    this(null, null, left, maxLinearVelocity, maxAngularVelocity,
        applyBackoff != null && applyBackoff);
  }

  @Override
  public void initialize() {
    isFinished = false;
    startTsSec = Timer.getFPGATimestamp();

    // Branch side por parámetro o modo de robot
    left = (left != null) ? left : Robot.isLeftMode();

    // Si dan solo side, infiere pos del front/back
    if (pos == null && side != null) {
      pos = side.getPosition(Robot.isFrontMode());
    }

    // reset retarget state
    lockedTagId = null;
    candidateTagId = null;
    candidateStable = 0;
    lastFreshIdTs = -1;

    // primer cálculo (puede no resolver aún)
    determineTarget(true);
  }

  @Override
  public void execute() {
    if (isFinished)
      return;

    double elapsed = Timer.getFPGATimestamp() - startTsSec;
    SmartDashboard.putNumber("Align/ElapsedSec", elapsed);
    if (elapsed > HighAltitudeConstants.COMMAND_TIMEOUT_SEC) {
      SmartDashboard.putString("Align/Status", "Timeout");
      isFinished = true;
      return;
    }

    // SIEMPRE reevalúa (permite retarget)
    determineTarget(false);

    if (targetPose == null) {
      SmartDashboard.putString("Align/Status", "Waiting target");
      return;
    }

    Pose2d poseToUse = targetPose;
    if (applyBackoff) {
      poseToUse = PoseUtil.backoff(poseToUse, HighAltitudeConstants.CORAL_BACKOFF_M);
    }

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

  // ---------------- Helpers ----------------

  /**
   * Resuelve/actualiza pos & target. Si recibo otro tag estable, **retargetea**. Si pierdo visión
   * brevemente, mantengo lock por ID_TTL_SEC.
   */
  private void determineTarget(boolean isInit) {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    int[] reefIDs =
        (alliance == DriverStation.Alliance.Red) ? HighAltitudeConstantsPose.RED_APRILTAG_IDS
            : HighAltitudeConstantsPose.BLUE_APRILTAG_IDS;
    Pose2d[] branches = (alliance == DriverStation.Alliance.Red)
        ? HighAltitudeConstantsPose.PATHFINDING_RED_BRANCHES
        : HighAltitudeConstantsPose.PATHFINDING_BLUE_BRANCHES;

    // Si ya viene pos explícita (o por side), solo arma la pose y sal
    if (pos != null) {
      int idx = pos.getBranchID(left);
      if (idx >= 0 && idx < branches.length) {
        targetPose = branches[idx];
        SmartDashboard.putNumber("Align/Target/X", targetPose.getX());
        SmartDashboard.putNumber("Align/Target/Y", targetPose.getY());
        SmartDashboard.putNumber("Align/Target/RotDeg", targetPose.getRotation().getDegrees());
      } else {
        targetPose = null;
      }
      return;
    }

    // ---- Auto a partir del tag confiable ----
    final double now = Timer.getFPGATimestamp();
    int id = Robot.getRobotContainer().getVision().getAlignmentTargetIdReliable();
    SmartDashboard.putNumber("Align/ID", id);

    if (id > 0) {
      lastFreshIdTs = now;

      if (lockedTagId == null) {
        // aún sin lock: espera N frames estables para fijarlo
        updateCandidate(id);
        if (candidateStable >= HighAltitudeConstants.TAG_DETECTION_LOCK_CYCLES) {
          lockedTagId = candidateTagId;
          SmartDashboard.putNumber("Align/LockedId", lockedTagId);
          mapIdToPose(lockedTagId, reefIDs, branches);
        }
      } else if (id == lockedTagId) {
        // seguimos viendo el mismo tag: nada que hacer
        candidateTagId = null;
        candidateStable = 0;
      } else {
        // posible retarget: requiere N frames estables del nuevo id
        updateCandidate(id);
        if (candidateStable >= HighAltitudeConstants.TAG_DETECTION_LOCK_CYCLES) {
          lockedTagId = candidateTagId;
          SmartDashboard.putNumber("Align/LockedId", lockedTagId);
          mapIdToPose(lockedTagId, reefIDs, branches);
        }
      }
    } else {
      // sin visión: mantén lock por un TTL corto; después, suelta
      if (lockedTagId != null && (now - lastFreshIdTs) <= ID_TTL_SEC) {
        // conserva targetPose actual
      } else {
        lockedTagId = null;
        candidateTagId = null;
        candidateStable = 0;
        targetPose = null; // forzar espera
      }
    }

    if (isInit) {
      SmartDashboard.putString("Align/Status",
          (targetPose != null) ? "Target locked (init)" : "Waiting (init)");
    }
  }

  private void updateCandidate(int newId) {
    if (candidateTagId == null || candidateTagId != newId) {
      candidateTagId = newId;
      candidateStable = 1;
    } else {
      candidateStable++;
    }
    SmartDashboard.putNumber("Align/CandidateId", candidateTagId);
    SmartDashboard.putNumber("Align/CandidateStable", candidateStable);
  }

  private void mapIdToPose(int id, int[] reefIDs, Pose2d[] branches) {
    // id -> REEF_POSITION -> branch index -> pose
    for (int i = 0; i < reefIDs.length; i++) {
      if (id == reefIDs[i]) {
        pos = HighAltitudeConstantsPose.REEF_POSITIONS[i];
        break;
      }
    }
    if (pos == null) {
      targetPose = null;
      return;
    }

    int idx = pos.getBranchID(left);
    if (idx >= 0 && idx < branches.length) {
      targetPose = branches[idx];
      SmartDashboard.putNumber("Align/Target/X", targetPose.getX());
      SmartDashboard.putNumber("Align/Target/Y", targetPose.getY());
      SmartDashboard.putNumber("Align/Target/RotDeg", targetPose.getRotation().getDegrees());
    } else {
      targetPose = null;
    }
  }
}
