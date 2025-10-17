// src/main/java/frc/robot/commands/autonomous/algae/AlgaeRemovalFromTargetID.java
package frc.robot.commands.autonomous.algae;

import java.util.Set;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.PoseIdx;
import frc.robot.HighAltitudeConstantsPose;
import frc.robot.Robot;
import frc.robot.commands.extensor.lift.control.LiftDefaultCommand;
import frc.robot.commands.extensor.lift.control.LiftGoToTarget;
import frc.robot.commands.extensor.wrist.control.WristGoToTarget;
import frc.robot.stateMachines.OIHelpers;

/**
 * Usa el TargetID (visión reliable) para: - SELECT_ONLY: setear el poseIdx (ALGAE_REMOVE_L2/L3) en
 * OIHelpers. - MOVE_TO_DETECTED_LEVEL: levantar/mover muñeca a L2/L3 detectado. - MOVE_TO_HOLD:
 * mover a ALGAE_HOLD.
 */
public class AlgaeRemovalFromTargetID extends SequentialCommandGroup {

  public enum StateAction {
    SELECT_ONLY, MOVE_TO_DETECTED_LEVEL, MOVE_TO_HOLD
  }

  public AlgaeRemovalFromTargetID(StateAction action) {

    // Paso 1: Resolver poseIdx según Tag (InstantCommand)
    Command selectIdx = new InstantCommand(() -> {
      Integer idx = resolvePoseIdxFromVision();
      if (idx != null)
        OIHelpers.setSelectedPoseIdx(idx);
    });

    // Paso 2: Mover a L2/L3 detectado (si aplica)
    Command moveToDetected = Commands.defer(() -> {
      int idx = OIHelpers.getSelectedPoseIdx();
      if (idx != PoseIdx.ALGAE_REMOVE_L2 && idx != PoseIdx.ALGAE_REMOVE_L3) {
        return Commands.none(); // sin lock o no es L2/L3
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

    // Paso 3: Mover a HOLD
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
      case SELECT_ONLY:
        addCommands(selectIdx);
        break;
      case MOVE_TO_DETECTED_LEVEL:
        addCommands(selectIdx, moveToDetected);
        break;
      case MOVE_TO_HOLD:
        addCommands(moveToHold);
        break;
    }
  }

  /**
   * Mapea el Tag de visión reliable a PoseIdx.ALGAE_REMOVE_L2 o L3.
   * 
   * @return índice o null si no hay tag mapeable.
   */
  private Integer resolvePoseIdxFromVision() {
    int id = Robot.getRobotContainer().getVision().getAlignmentTargetIdReliable();
    if (id <= 0)
      return null;

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
