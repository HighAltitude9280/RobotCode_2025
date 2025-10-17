// src/main/java/frc/robot/commands/autonomous/algae/AlgaeDetachFromReefSequence.java
package frc.robot.commands.autonomous.algae;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.commands.extensor.gripper.algae.IntakeUntilCurrentAlgae;
import frc.robot.commands.extensor.lift.control.LiftDefaultCommand;
import frc.robot.commands.swerve.autonomous.offSeason.BackoffFromCurrentPose;
import frc.robot.commands.swerve.autonomous.offSeason.DriveToAlgaeRemovalFromReef;

public class AlgaeDetachFromReefSequence extends SequentialCommandGroup {

  /**
   * @param vMax límite lineal durante drive/alignment
   * @param wMax límite angular durante drive/alignment
   * @param approachM offset (local) para el approach (+avanza, 0 sugerido)
   * @param backoffM backoff local al final (p.ej. 0.4–0.6 m)
   */
  public AlgaeDetachFromReefSequence(double vMax, double wMax, double approachM, double backoffM) {

    var driveLock = new DriveToAlgaeRemovalFromReef(vMax, wMax, approachM)
        .withTimeout(HighAltitudeConstants.DRIVE_TO_POSE_TIMEOUT_S);

    var moveToDetected =
        new AlgaeRemovalFromTargetID(AlgaeRemovalFromTargetID.StateAction.MOVE_TO_DETECTED_LEVEL);

    var intakeUntilCurrent = new ParallelRaceGroup(
        new IntakeUntilCurrentAlgae().withTimeout(HighAltitudeConstants.ALGAE_INTAKE_TIMEOUT_S),
        new LiftDefaultCommand(HighAltitudeConstants.LIFT_MAX_POWER,
            HighAltitudeConstants.LIFT_ARRIVE_OFFSET));

    var backoff = new BackoffFromCurrentPose(backoffM, vMax, wMax)
        .withTimeout(HighAltitudeConstants.DRIVE_TO_POSE_TIMEOUT_S);

    var moveToHold =
        new AlgaeRemovalFromTargetID(AlgaeRemovalFromTargetID.StateAction.MOVE_TO_HOLD);

    addCommands(driveLock, moveToDetected, intakeUntilCurrent, backoff, moveToHold);
  }
}
