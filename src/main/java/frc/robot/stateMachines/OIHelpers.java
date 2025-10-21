// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.stateMachines;

import java.util.EnumSet;
import java.util.function.Supplier;
// WPILib Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// Robot Imports
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.PoseIdx;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.Robot.GameMode;
import frc.robot.commands.autonomous.coral.CoralL1_StationIntakeSequence;
import frc.robot.commands.autonomous.coral.CoralLX_StationIntakeSequence;
import frc.robot.commands.autonomous.coral.DriveToTargetBranchAndScore;
import frc.robot.commands.autonomous.coral.ScoreAtPose;
import frc.robot.commands.cancel.ResetLiftEncoders;
import frc.robot.commands.extensor.compound.algae.AlgaeIntakeFloor;
import frc.robot.commands.extensor.compound.both.LiftWristGoToPose;
import frc.robot.commands.extensor.gripper.IntakeAuto;
import frc.robot.commands.extensor.gripper.algae.IntakeAlgaeAuto;
import frc.robot.commands.extensor.gripper.manual.HoldAlgae;
import frc.robot.commands.swerve.autonomous.offSeason.BackoffFromCurrentPose;
import frc.robot.commands.swerve.autonomous.offSeason.DriveToAlgaeRemovalFromReef;
import frc.robot.commands.swerve.autonomous.offSeason.DriveToNet;
import frc.robot.commands.swerve.autonomous.offSeason.DriveToProcessor;

/**
 * OIHelpers es una clase de utilidad (no se puede instanciar) que maneja: 1. El estado de la pose
 * seleccionada por el copiloto ({@code selectedPoseIdx}). 2. Comandos "Lanzadores" (InstantCommand)
 * que se usan en los bindings del OI para evitar problemas de "requisitos" (requirements) al
 * iniciar el robot.
 */
public final class OIHelpers {

  /** Constructor privado para prevenir instanciación. */
  private OIHelpers() {}

  // ==================================================================================
  // 1. GESTIÓN DE ESTADO (INTERNO)
  // ==================================================================================

  // Valor por defecto seguro (L2)
  private static int selectedPoseIdx = frc.robot.HighAltitudeConstants.PoseIdx.L2;

  /** Devuelve el poseIdx seleccionado actualmente (0..12). */
  public static synchronized int getSelectedPoseIdx() {
    return selectedPoseIdx;
  }

  /** Setea el poseIdx seleccionado (clamped a 0..COUNT-1). */
  public static synchronized void setSelectedPoseIdx(int idx) {
    int max = frc.robot.HighAltitudeConstants.PoseIdx.COUNT - 1;
    if (idx < 0)
      idx = 0;
    if (idx > max)
      idx = max;
    selectedPoseIdx = idx;
  }

  // ==================================================================================
  // 2. MÉTODOS DE UTILIDAD (HELPERS)
  // ==================================================================================

  /**
   * Helper para bindings: envuelve un comando para que solo se ejecute si el modo actual es el
   * requerido.
   */
  public static Command onlyInMode(Supplier<GameMode> modeSup, GameMode required, Command inner) {
    return new ConditionalCommand(inner, Commands.none(), () -> modeSup.get() == required);
  }

  /**
   * Helper para bindings: envuelve un comando para que solo se ejecute si el modo actual está en el
   * set de modos permitidos.
   */
  public static Command onlyInModes(Supplier<GameMode> modeSup, EnumSet<GameMode> allowed,
      Command inner) {
    return new ConditionalCommand(inner, Commands.none(), () -> allowed.contains(modeSup.get()));
  }

  /** Atajo: setea el seleccionado mapeando desde REEF_HEIGHT. */
  public static synchronized void setSelectedByHeight(REEF_HEIGHT h) {
    setSelectedPoseIdx(mapHeightToPoseIdx(h));
  }

  /** Mapeo REEF_HEIGHT → uno de tus 13 poseIdx. */
  private static int mapHeightToPoseIdx(REEF_HEIGHT h) {
    switch (h) {
      case BOTTOM:
        return PoseIdx.INTAKE_REAR;
      case L2:
        return PoseIdx.L2;
      case L3:
        return PoseIdx.L3;
      case TOP:
        return PoseIdx.L4;
      default:
        return PoseIdx.L2;
    }
  }

  /** Mapea un PoseIdx (int) a un nombre (String) para el Dashboard. */
  public static String mapPoseIdxToName(int idx) {
    switch (idx) {
      case PoseIdx.INTAKE_REAR:
        return "Intake Rear";
      case PoseIdx.L1_INTAKE:
        return "L1 Intake";
      case PoseIdx.L1_SCORE:
        return "L1 Score";
      case PoseIdx.L2:
        return "L2";
      case PoseIdx.L3:
        return "L3";
      case PoseIdx.L4:
        return "L4";
      case PoseIdx.ALGAE_INTAKE_FLOOR:
        return "Algae Intake Floor";
      case PoseIdx.ALGAE_HOLD:
        return "Algae Hold";
      case PoseIdx.PROCESSOR_SCORE:
        return "Processor Score";
      case PoseIdx.NET_PREPOS:
        return "Net PrePos";
      case PoseIdx.NET_SCORE:
        return "Net Score";
      case PoseIdx.ALGAE_REMOVE_L2:
        return "Algae Remove L2";
      case PoseIdx.ALGAE_REMOVE_L3:
        return "Algae Remove L3";
      default:
        return "Desconocido (" + idx + ")";
    }
  }

  // ==================================================================================
  // 3. COMANDOS DE AJUSTE DE ESTADO (PARA COPILOTO)
  // ==================================================================================

  /** Guarda en estado el nivel (L2/L3/L4/Bottom) elegido por el copiloto. */
  public static class SetSelectedHeightCommand extends InstantCommand {
    private final REEF_HEIGHT height;

    public SetSelectedHeightCommand(REEF_HEIGHT height) {
      this.height = height;
    }

    @Override
    public void initialize() {
      OIHelpers.setSelectedByHeight(height);
      System.out.println(height);
    }
  }

  /**
   * Setea directamente uno de los 13 poseIdx y actualiza el SmartDashboard con el nombre de la
   * pose.
   */
  public static class SetSelectedPoseIdxCommand extends InstantCommand {
    private final int poseIdx;

    public SetSelectedPoseIdxCommand(int poseIdx) {
      this.poseIdx = poseIdx;
    }

    @Override
    public void initialize() {
      OIHelpers.setSelectedPoseIdx(poseIdx);
      String poseName = OIHelpers.mapPoseIdxToName(poseIdx);
      SmartDashboard.putString("LEVEL SELECTED", poseName);
    }
  }

  // ==================================================================================
  // 4. COMANDOS DE LANZAMIENTO DE ACCIONES (PARA PILOTO)
  // ==================================================================================

  // --- A. Lanzadores de "Pose Seleccionada" ---

  /**
   * (LANZADOR) Dispara LiftWristGoToPose con el índice seleccionado actualmente.
   */
  public static class LiftWristGoToSelectedPose extends InstantCommand {
    @Override
    public void initialize() {
      int idx = OIHelpers.getSelectedPoseIdx();
      new LiftWristGoToPose(idx).schedule();
    }
  }

  /**
   * (LANZADOR) Se alinea a la rama (branch) y dispara la secuencia de ScoreAtPose usando el índice
   * seleccionado actualmente.
   */
  public static class DriveToBranchAndScoreSelected extends InstantCommand {
    private final boolean left;

    public DriveToBranchAndScoreSelected(boolean left) {
      this.left = left;
    }

    @Override
    public void initialize() {
      int idx = OIHelpers.getSelectedPoseIdx();
      new DriveToTargetBranchAndScore(left, idx).schedule();
    }
  }

  // --- B. Lanzadores de "Intake" ---

  /** (LANZADOR) Lanza la secuencia de Intake de estación para CORAL_L1. */
  public static class LaunchCoralL1StationIntake extends InstantCommand {
    private final boolean left;

    public LaunchCoralL1StationIntake(boolean left) {
      this.left = left;
    }

    @Override
    public void initialize() {
      new CoralL1_StationIntakeSequence(left).schedule();
    }
  }

  /** (LANZADOR) Lanza la secuencia de Intake de estación para CORAL_LX. */
  public static class LaunchCoralLXStationIntake extends InstantCommand {
    private final boolean left;

    public LaunchCoralLXStationIntake(boolean left) {
      this.left = left;
    }

    @Override
    public void initialize() {
      new CoralLX_StationIntakeSequence(left).schedule();
    }
  }

  /** (LANZADOR) Lanza la secuencia de Intake de Algae desde el suelo. */
  public static class LaunchAlgaeIntakeFloor extends InstantCommand {
    public LaunchAlgaeIntakeFloor() {}

    @Override
    public void initialize() {
      new AlgaeIntakeFloor().schedule();
    }
  }

  /**
   * (LANZADOR) Lanza un LiftWristGoToPose a un ÍNDICE ESPECÍFICO. (Usado para el onFalse de
   * ALGAE_HOLD).
   */
  public static class LaunchLiftWristGoToPose extends InstantCommand {
    private final int poseIdx;

    public LaunchLiftWristGoToPose(int poseIdx) {
      this.poseIdx = poseIdx;
    }

    @Override
    public void initialize() {
      new LiftWristGoToPose(poseIdx).schedule();
    }
  }

  // --- C. Lanzadores de "Score" y Acciones Fijas ---

  /**
   * (LANZADOR) Se alinea al Processor y dispara la secuencia de ScoreAtPose usando la pose fija
   * PROCESSOR_SCORE.
   */
  public static class DriveToProcessorAndScore extends InstantCommand {
    @Override
    public void initialize() {
      new SequentialCommandGroup(new DriveToProcessor(HighAltitudeConstants.VISION_POSE_MAX_SPEED,
          HighAltitudeConstants.VISION_POSE_MAX_TURN), new ScoreAtPose(PoseIdx.PROCESSOR_SCORE))
              .schedule();
    }
  }

  /**
   * (LANZADOR) Secuencia completa para anotar en la Net (CORREGIDA v3): - HoldAlgae corre en
   * paralelo durante las fases 1, 2 y 3. 1. (Paralelo) MIENTRAS (DriveToNet) Y (LiftWrist va a
   * ALGAE_HOLD), con timeout 6s. 2. (Secuencial) LiftWrist va a NET_PREPOS. 3. (Paralelo) MIENTRAS
   * (Backoff) Y (LiftWrist MANTIENE NET_PREPOS), con timeout 6s. 4. (Secuencial) HoldAlgae se
   * interrumpe y Ejecuta ScoreAtPose(NET_SCORE).
   */
  public static class DriveToNetAndScore extends InstantCommand {
    @Override
    public void initialize() {

      // --- FASE 1: Drive to Net + Go to ALGAE_HOLD (Timeout 6s) ---
      // HoldAlgae ahora está AFUERA de este grupo.
      Command phase1_DriveAndPrep = new ParallelRaceGroup(
          // Comandos principales (uno termina la fase):
          new DriveToNet(HighAltitudeConstants.VISION_POSE_MAX_SPEED,
              HighAltitudeConstants.VISION_POSE_MAX_TURN),
          new WaitCommand(6.0), // Timeout

          // Comando de fondo (corre hasta que termine un principal):
          new LiftWristGoToPose(PoseIdx.ALGAE_HOLD) // Mueve Lift/Wrist
      );

      // --- FASE 2: Go to NET_PREPOS ---
      // HoldAlgae ahora está AFUERA de este paso.
      Command phase2_GoToPreScore = new LiftWristGoToPose(PoseIdx.NET_PREPOS);

      // --- FASE 3: Backoff + Hold NET_PREPOS (Timeout 6s) ---
      // HoldAlgae ahora está AFUERA de este grupo.
      Command phase3_BackoffAndHoldPose = new ParallelRaceGroup(
          // Comandos principales:
          new BackoffFromCurrentPose(-0.1, HighAltitudeConstants.VISION_POSE_MAX_SPEED * 0.2,
              HighAltitudeConstants.VISION_POSE_MAX_TURN),
          new WaitCommand(6.0), // Timeout

          // Comando de fondo:
          new LiftWristGoToPose(PoseIdx.NET_PREPOS) // MANTIENE Lift/Wrist
      );

      // --- FASE 4: Score ---
      Command phase4_ActualScore = new ScoreAtPose(PoseIdx.NET_SCORE);

      // --- SECUENCIA COMPLETA ---
      // Usamos ParallelDeadlineGroup para correr HoldAlgae junto a las fases 1-3.
      new SequentialCommandGroup(
          // Paso 1: Ejecuta HoldAlgae en paralelo con la secuencia 1-2-3.
          // Cuando la secuencia 1-2-3 termine, HoldAlgae será interrumpido.
          new ParallelDeadlineGroup(
              // EL DEADLINE: La secuencia de las 3 fases de movimiento.
              new SequentialCommandGroup(phase1_DriveAndPrep, phase2_GoToPreScore,
                  phase3_BackoffAndHoldPose),
              // EL COMANDO PARALELO: Se ejecuta hasta que el deadline termine.
              new HoldAlgae()), // Fin del ParallelDeadlineGroup

          // Paso 2: Ejecuta la anotación DESPUÉS de que HoldAlgae fue interrumpido.
          phase4_ActualScore

      ).schedule();
    }
  }
  /**
   * (LANZADOR) Lanza la secuencia de alineación con visión para quitar algas del reef.
   */
  public static class DriveToAlgaeRemoval extends InstantCommand {
    public DriveToAlgaeRemoval() {}

    @Override
    public void initialize() {
      Command realCommand =
          new DriveToAlgaeRemovalFromReef(HighAltitudeConstants.VISION_POSE_MAX_SPEED * 0.8,
              HighAltitudeConstants.VISION_POSE_MAX_TURN * 0.8,
              HighAltitudeConstants.ALGAE_RETRACT_OFFSET_M);
      realCommand.schedule();
    }
  }
  // (Asegúrate de tener los imports para IntakeAuto, IntakeAlgaeAuto, y ResetLiftEncoders)

  /** (LANZADOR) Ejecuta el comando IntakeAuto. */
  public static class LaunchIntakeAuto extends InstantCommand {
    public LaunchIntakeAuto() {}

    @Override
    public void initialize() {
      new IntakeAuto().schedule();
    }
  }

  /** (LANZADOR) Ejecuta el comando IntakeAlgaeAuto. */
  public static class LaunchIntakeAlgaeAuto extends InstantCommand {
    public LaunchIntakeAlgaeAuto() {}

    @Override
    public void initialize() {
      new IntakeAlgaeAuto().schedule();
    }
  }

  /** (LANZADOR) Ejecuta el comando ResetLiftEncoders. */
  public static class LaunchResetLiftEncoders extends InstantCommand {
    public LaunchResetLiftEncoders() {}

    @Override
    public void initialize() {
      new ResetLiftEncoders().schedule();
    }
  }
} // Fin de la clase OIHelpers
