// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

public class DriveToTargetBranchPose extends Command {

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

  // ---- LÓGICA DE CACHÉ ELIMINADA ----
  // Se eliminaron: lockedTagId, candidateTagId, candidateStable, lastFreshIdTs, ID_TTL_SEC


  /** Constructor para alineación manual (pos/side explícitos) */
  public DriveToTargetBranchPose(REEF_POSITION position, REEF_SIDE side, Boolean left,
      double maxLinearVelocity, double maxAngularVelocity) {
    this(position, side, left, maxLinearVelocity, maxAngularVelocity, false);
  }

  /** Constructor para alineación manual (pos/side explícitos) con backoff */
  public DriveToTargetBranchPose(REEF_POSITION position, REEF_SIDE side, Boolean left,
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
   * === ESTE ES EL CONSTRUCTOR QUE PIDES === Constructor para Auto (pos/side son null). Solo usa
   * 'left' y el tag de visión (getBestTargetID) para determinar el target.
   */
  public DriveToTargetBranchPose(Boolean left, double maxLinearVelocity, double maxAngularVelocity,
      Boolean applyBackoff) {
    this(null, null, left, maxLinearVelocity, maxAngularVelocity,
        applyBackoff != null && applyBackoff);
  }

  @Override
  public void initialize() {
    isFinished = false;
    startTsSec = Timer.getFPGATimestamp();

    // Resuelve 'left'
    left = (left != null) ? left : Robot.isLeftMode();

    // Lógica para constructores manuales (si se usa)
    if (pos == null && side != null) {
      pos = side.getPosition(Robot.isFrontMode());
    }

    // Lógica de caché eliminada de initialize()

    // primer cálculo (puede no resolver aún si 'pos' es null y no hay visión)
    determineTarget(true);
  }

  @Override
  public void execute() {
    if (isFinished)
      return;

    // Timeout
    double elapsed = Timer.getFPGATimestamp() - startTsSec;
    SmartDashboard.putNumber("AlignTUPU/ElapsedSec", elapsed);
    if (elapsed > HighAltitudeConstants.COMMAND_TIMEOUT_SEC) {
      SmartDashboard.putString("AlignTUPU/Status", "Timeout");
      isFinished = true;
      return;
    }

    // Reevalúa el target en CADA frame (ya no hay caché)
    determineTarget(false);

    // Si no hay targetPose (porque no se especificó o no hay visión), espera.
    if (targetPose == null) {
      SmartDashboard.putString("AlignTUPU/Status", "Waiting target");
      // Detiene el robot si no hay target. Cumple "solo se alinee si ve uno bueno"
      Robot.getRobotContainer().getSwerveDriveTrain().stopModules();
      return;
    }

    // Aplica backoff si es necesario
    Pose2d poseToUse = targetPose;
    if (applyBackoff) {
      poseToUse = PoseUtil.backoff(poseToUse, HighAltitudeConstants.CORAL_BACKOFF_M);
    }

    // Ejecuta la alineación
    boolean reached = Robot.getRobotContainer().getSwerveDriveTrain().AlignWithTargetPose(poseToUse,
        maxLinearVelocity, maxAngularVelocity);

    SmartDashboard.putBoolean("AlignTUPU/Reached", reached);
    SmartDashboard.putString("AlignTUPU/Status", reached ? "Reached" : "Tracking");
    isFinished = reached;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.getRobotContainer().getSwerveDriveTrain().stopModules();
    SmartDashboard.putString("AlignTUPU/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("AlignTUPU/isFinished", isFinished);
    return isFinished;
  }

  // ---------------- Helpers ----------------

  /**
   * Resuelve/actualiza pos & target. VERSIÓN SIMPLIFICADA: Si pos es null, usa getBestTargetID()
   * directamente. Si no hay target, targetPose se vuelve null.
   */
  private void determineTarget(boolean isInit) {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    int[] reefIDs =
        (alliance == DriverStation.Alliance.Red) ? HighAltitudeConstantsPose.RED_APRILTAG_IDS
            : HighAltitudeConstantsPose.BLUE_APRILTAG_IDS;
    Pose2d[] branches = (alliance == DriverStation.Alliance.Red)
        ? HighAltitudeConstantsPose.PATHFINDING_RED_BRANCHES
        : HighAltitudeConstantsPose.PATHFINDING_BLUE_BRANCHES;

    // --- Caso 1: Alineación Manual (pos/side dados en constructor) ---
    if (pos != null) {
      int idx = pos.getBranchID(left);
      if (idx >= 0 && idx < branches.length) {
        targetPose = branches[idx];
        SmartDashboard.putNumber("AlignTUPU/Target/X", targetPose.getX());
        SmartDashboard.putNumber("AlignTUPU/Target/Y", targetPose.getY());
        SmartDashboard.putNumber("AlignTUPU/Target/RotDeg", targetPose.getRotation().getDegrees());
      } else {
        targetPose = null; // ID de branch inválido
      }
      return; // Termina aquí si es manual
    }

    // --- Caso 2: Alineación Automática (pos == null) ---
    // Se usa el nuevo método de Vision sin caché

    // Llama al nuevo método en Vision
    int id = Robot.getRobotContainer().getVision().getBestTargetID();
    SmartDashboard.putNumber("AlignTUPU/ID", id);

    if (id > 0) {
      // Si vemos un ID válido, lo mapeamos a una pose
      mapIdToPose(id, reefIDs, branches);
    } else {
      // Si no vemos un ID válido (getBestTargetID() retornó -1),
      // borramos el target.
      targetPose = null; // Fuerza al robot a esperar en execute()
    }

    // Toda la lógica de lockedTagId, candidateTagId, stable, TTL, etc., se elimina.

    if (isInit) {
      SmartDashboard.putString("AlignTUPU/Status",
          (targetPose != null) ? "Target locked (init)" : "Waiting (init)");
    }
  }

  // updateCandidate() fue eliminado porque ya no es necesario.

  /** Mapea un ID de AprilTag a una Pose2d de 'branches' */
  private void mapIdToPose(int id, int[] reefIDs, Pose2d[] branches) {
    // id -> REEF_POSITION -> branch index -> pose

    REEF_POSITION currentPos = null; // Usa variable local en lugar de 'this.pos'

    for (int i = 0; i < reefIDs.length; i++) {
      if (id == reefIDs[i]) {
        currentPos = HighAltitudeConstantsPose.REEF_POSITIONS[i];
        break;
      }
    }

    // Si el ID no mapea a ninguna posición conocida
    if (currentPos == null) {
      targetPose = null;
      return;
    }

    // Convierte REEF_POSITION a un índice usando 'left'
    int idx = currentPos.getBranchID(left);
    if (idx >= 0 && idx < branches.length) {
      targetPose = branches[idx];
      SmartDashboard.putNumber("AlignTUPU/Target/X", targetPose.getX());
      SmartDashboard.putNumber("AlignTUPU/Target/Y", targetPose.getY());
      SmartDashboard.putNumber("AlignTUPU/Target/RotDeg", targetPose.getRotation().getDegrees());
    } else {
      targetPose = null; // Índice de branch inválido
    }
  }
}
