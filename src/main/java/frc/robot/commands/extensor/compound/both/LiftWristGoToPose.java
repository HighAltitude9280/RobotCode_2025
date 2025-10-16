// src/main/java/frc/robot/commands/extensor/compound/both/LiftWristGoToPose.java
package frc.robot.commands.extensor.compound.both;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.Robot;
import frc.robot.commands.extensor.lift.control.LiftDefaultCommand;
import frc.robot.commands.extensor.lift.control.LiftGoToTarget;
import frc.robot.commands.extensor.wrist.control.WristGoToTarget;
import frc.robot.subsystems.extensor.Lift;
import frc.robot.subsystems.extensor.Wrist;

/**
 * Mueve Lift y Wrist a uno de los 13 targets (índice 0..12) con orden seguro:
 * - Si el objetivo queda ARRIBA de la posición actual: Lift -> (hold) + Wrist
 * - Si el objetivo queda ABAJO: Wrist -> Lift (bajada)
 *
 * Los valores de lift/wrist salen de
 * HighAltitudeConstants.LIFT_POSE/WRIST_POSE.
 */
public class LiftWristGoToPose extends InstantCommand {
  private final Lift lift;
  private final Wrist wrist;
  private final int poseIdx;

  /**
   * @param poseIdx índice 0..12 (ver HighAltitudeConstants.PoseIdx)
   */
  public LiftWristGoToPose(int poseIdx) {
    this.lift = Robot.getRobotContainer().getLift();
    this.wrist = Robot.getRobotContainer().getWrist();
    this.poseIdx = clampIdx(poseIdx);
  }

  /**
   * Compat: si todavía llamas por REEF_HEIGHT, mapeo rápido a tus 13 poses.
   * BOTTOM->L1_SCORE (2), L2->(3), L3->(4), TOP->(5)
   */
  public LiftWristGoToPose(REEF_HEIGHT height) {
    this(mapHeightToPoseIdx(height));
  }

  private static int clampIdx(int i) {
    return Math.max(0, Math.min(i, HighAltitudeConstants.PoseIdx.COUNT - 1));
  }

  // Ajustar este mapeo si BOTTOM->L1_INTAKE (1) en vez de L1_SCORE (2)
  private static int mapHeightToPoseIdx(REEF_HEIGHT h) {
    return switch (h) {
      case BOTTOM -> HighAltitudeConstants.PoseIdx.L1_SCORE; // o L1_INTAKE
      case L2 -> HighAltitudeConstants.PoseIdx.L2;
      case L3 -> HighAltitudeConstants.PoseIdx.L3;
      case TOP -> HighAltitudeConstants.PoseIdx.L4;
    };
  }

  @Override
  public void initialize() {
    final double liftTarget = HighAltitudeConstants.LIFT_POSE[poseIdx];
    final double wristTarget = HighAltitudeConstants.WRIST_POSE[poseIdx];

    final boolean goingUp = liftTarget > lift.getLiftPosMeters();

    if (goingUp) {
      new SequentialCommandGroup(
          new LiftGoToTarget(
              HighAltitudeConstants.LIFT_MAX_POWER,
              liftTarget,
              HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
          new ParallelRaceGroup(
              new LiftDefaultCommand(
                  HighAltitudeConstants.LIFT_MAX_POWER,
                  HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
              new WristGoToTarget(
                  wristTarget,
                  HighAltitudeConstants.WRIST_DRIVE_SPEED)))
          .schedule();

    } else {
      new SequentialCommandGroup(
          new ParallelRaceGroup(
              new LiftDefaultCommand(
                  HighAltitudeConstants.LIFT_MAX_POWER,
                  HighAltitudeConstants.LIFT_ARRIVE_OFFSET),
              new WristGoToTarget(
                  wristTarget,
                  HighAltitudeConstants.WRIST_DRIVE_SPEED)),
          new LiftGoToTarget(
              HighAltitudeConstants.LIFT_MAX_POWER_GOING_DOWN,
              liftTarget,
              HighAltitudeConstants.LIFT_ARRIVE_OFFSET))
          .schedule();
    }
  }
}
