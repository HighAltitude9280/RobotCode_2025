// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.stateMachines;

import java.util.EnumSet;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.HighAltitudeConstants.PoseIdx;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.Robot.GameMode;
import frc.robot.commands.extensor.compound.both.LiftWristGoToPose;

public final class OIHelpers {
  private OIHelpers() {}

  public static Command onlyInMode(Supplier<GameMode> modeSup, GameMode required, Command inner) {
    return new ConditionalCommand(inner, Commands.none(), () -> modeSup.get() == required);
  }

  public static Command onlyInModes(Supplier<GameMode> modeSup, EnumSet<GameMode> allowed,
      Command inner) {
    return new ConditionalCommand(inner, Commands.none(), () -> allowed.contains(modeSup.get()));
  }

  // Valor por defecto seguro (ajústalo si prefieres otro target)
  private static int selectedPoseIdx = frc.robot.HighAltitudeConstants.PoseIdx.L2;

  /** Devuelve el poseIdx seleccionado (0..12). */
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

  /** Atajo: setea el seleccionado mapeando desde REEF_HEIGHT. */
  public static synchronized void setSelectedByHeight(REEF_HEIGHT h) {
    setSelectedPoseIdx(mapHeightToPoseIdx(h));
  }

  /** Mapeo REEF_HEIGHT → uno de tus 13 poseIdx (ajústalo si quieres L1_INTAKE). */
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

  // ---------------------------------------------------------------------------
  // Comandos anidados para no crear más archivos
  // ---------------------------------------------------------------------------

  /** Guarda en estado el nivel (L2/L3/L4/Bottom) elegido por el copiloto. */
  public static class SetSelectedHeightCommand
      extends edu.wpi.first.wpilibj2.command.InstantCommand {
    private final REEF_HEIGHT height;

    public SetSelectedHeightCommand(REEF_HEIGHT height) {
      this.height = height;
    }

    @Override
    public void initialize() {
      OIHelpers.setSelectedByHeight(height);
      // TODO opcional: feedback (LEDs/haptics/log)
    }
  }

  /** Setea directamente uno de los 13 poseIdx (útil si quieres botones directos). */
  public static class SetSelectedPoseIdxCommand extends InstantCommand {
    private final int poseIdx;

    public SetSelectedPoseIdxCommand(int poseIdx) {
      this.poseIdx = poseIdx;
    }

    @Override
    public void initialize() {
      OIHelpers.setSelectedPoseIdx(poseIdx);
    }
  }

  /**
   * Dispara LiftWristGoToPose con el índice seleccionado actualmente. No reclama requirements (deja
   * que los default commands mantengan posición).
   */
  public static class LiftWristGoToSelectedPose extends InstantCommand {
    @Override
    public void initialize() {
      int idx = OIHelpers.getSelectedPoseIdx();
      new LiftWristGoToPose(idx).schedule();
    }
  }
}
