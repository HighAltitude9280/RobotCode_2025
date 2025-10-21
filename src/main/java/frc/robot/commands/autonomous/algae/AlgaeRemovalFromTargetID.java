// src/main/java/frc/robot/commands/autonomous/algae/AlgaeRemovalFromTargetID.java
package frc.robot.commands.autonomous.algae;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// Quita InstantCommand si ya no se usa
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.PoseIdx;
// Quita HighAltitudeConstantsPose, DriverStation, Robot si ya no se usan aquí
import frc.robot.Robot; // Robot todavía se necesita para los subsistemas
import frc.robot.commands.extensor.lift.control.LiftDefaultCommand;
import frc.robot.commands.extensor.lift.control.LiftGoToTarget;
import frc.robot.commands.extensor.wrist.control.WristGoToTarget;
import frc.robot.stateMachines.OIHelpers;

/**
 * (Refactorizado) Mueve los mecanismos (Lift/Wrist) a una pose de alga. YA NO SELECCIONA el ID.
 * Simplemente LEE el PoseIdx de OIHelpers. - MOVE_TO_DETECTED_LEVEL: Lee OIHelpers y mueve a L2/L3
 * si está seteado. - MOVE_TO_HOLD: Mueve a ALGAE_HOLD.
 */
public class AlgaeRemovalFromTargetID extends SequentialCommandGroup {

  public enum StateAction {
    // SELECT_ONLY fue removido
    MOVE_TO_DETECTED_LEVEL, MOVE_TO_HOLD
  }

  public AlgaeRemovalFromTargetID(StateAction action) {

    // El 'selectIdx' (InstantCommand) se ELIMINA de aquí.

    // Paso 1: Mover a L2/L3 detectado (si aplica)
    Command moveToDetected = Commands.defer(() -> {
      // Esta lógica lee el estado de OIHelpers (que fue seteado por otro comando)
      int idx = OIHelpers.getSelectedPoseIdx();
      if (idx != PoseIdx.ALGAE_REMOVE_L2 && idx != PoseIdx.ALGAE_REMOVE_L3) {
        return Commands.none(); // No hay un idx válido seteado, no hacer nada
      }
      double lift = HighAltitudeConstants.liftForPose(idx);
      double wrist = HighAltitudeConstants.wristForPose(idx);

      return new SequentialCommandGroup(
          new LiftGoToTarget(HighAltitudeConstants.LIFT_MAX_POWER, lift,
              HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
          new ParallelRaceGroup(
              new LiftDefaultCommand(HighAltitudeConstants.LIFT_MAX_POWER,
                  HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
              new WristGoToTarget(wrist, HighAltitudeConstants.WRIST_DRIVE_SPEED)));
    }, Set.of(Robot.getRobotContainer().getLift(), Robot.getRobotContainer().getWrist()));

    // Paso 2: Mover a HOLD (Esta lógica no cambia)
    Command moveToHold = Commands.defer(() -> {
      int idx = PoseIdx.ALGAE_HOLD;
      double lift = HighAltitudeConstants.liftForPose(idx);
      double wrist = HighAltitudeConstants.wristForPose(idx);

      return new SequentialCommandGroup(
          new ParallelRaceGroup(
              new LiftDefaultCommand(HighAltitudeConstants.LIFT_MAX_POWER,
                  HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
              new WristGoToTarget(wrist, HighAltitudeConstants.WRIST_DRIVE_SPEED)),
          new LiftGoToTarget(HighAltitudeConstants.LIFT_MAX_POWER_GOING_DOWN, lift,
              HighAltitudeConstants.LIFT_ARRIVE_OFFSET));
    }, Set.of(Robot.getRobotContainer().getLift(), Robot.getRobotContainer().getWrist()));

    // Armar según Action
    switch (action) {
      case MOVE_TO_DETECTED_LEVEL:
        // Ya no se incluye selectIdx, solo el movimiento
        addCommands(moveToDetected);
        break;
      case MOVE_TO_HOLD:
        addCommands(moveToHold);
        break;
    }
  }

  // Los métodos resolvePoseIdxFromVision() y contains() se movieron
  // al nuevo comando 'UpdateAlgaePoseFromVision'.
}
