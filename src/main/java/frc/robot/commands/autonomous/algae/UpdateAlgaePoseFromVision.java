// src/main/java/frc/robot/commands/autonomous/algae/UpdateAlgaePoseFromVision.java
package frc.robot.commands.autonomous.algae;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants.PoseIdx;
import frc.robot.HighAltitudeConstantsPose;
import frc.robot.Robot;
import frc.robot.stateMachines.OIHelpers;

/**
 * Comando que corre en paralelo, revisando la visión (getBestTargetID) CADA FRAME y actualizando el
 * PoseIdx en OIHelpers. Está diseñado para ser cancelado (ej. por un ParallelRaceGroup) cuando ya
 * no se necesite (ej. cuando el drive-lock termine).
 */
public class UpdateAlgaePoseFromVision extends Command {

  public UpdateAlgaePoseFromVision() {
    // No requiere subsistemas, solo lee de Vision (que corre en periodic)
  }

  @Override
  public void initialize() {
    OIHelpers.setSelectedPoseIdx(PoseIdx.INTAKE_REAR);
  }

  @Override
  public void execute() {
    // 1. Llama a tu método de visión instantáneo
    int id = Robot.getRobotContainer().getVision().getBestTargetID();
    if (id <= 0) {
      return; // No hay tag, no actualiza (mantiene el último válido o NONE)
    }

    // 2. Mapea el ID a un PoseIdx
    Integer idx = resolvePoseIdxFromVisionID(id);

    // 3. Actualiza OIHelpers si es un ID de alga válido
    if (idx != null) {
      // Actualiza constantemente con el último tag válido visto
      OIHelpers.setSelectedPoseIdx(idx);
    }
  }

  @Override
  public boolean isFinished() {
    return false; // Nunca termina por sí solo. Debe ser cancelado por la secuencia.
  }

  @Override
  public void end(boolean interrupted) {
    // Al ser cancelado (interrupted == true), el último valor
    // que escribió en OIHelpers queda "latchado".
  }

  /** Mapea el Tag de visión a PoseIdx.ALGAE_REMOVE_L2 o L3. */
  private Integer resolvePoseIdxFromVisionID(int id) {
    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    if (alliance == DriverStation.Alliance.Blue) {
      if (contains(HighAltitudeConstantsPose.BLUE_ALGAE_L3_TAGS, id))
        return PoseIdx.ALGAE_REMOVE_L3;
      if (contains(HighAltitudeConstantsPose.BLUE_ALGAE_L2_TAGS, id))
        return PoseIdx.ALGAE_REMOVE_L2;
    } else {
      if (contains(HighAltitudeConstantsPose.RED_ALGAE_L3_TAGS, id))
        return PoseIdx.ALGAE_REMOVE_L3;
      if (contains(HighAltitudeConstantsPose.RED_ALGAE_L2_TAGS, id))
        return PoseIdx.ALGAE_REMOVE_L2;
    }
    return null;
  }

  private static boolean contains(int[] arr, int v) {
    for (int a : arr)
      if (a == v)
        return true;
    return false;
  }
}
