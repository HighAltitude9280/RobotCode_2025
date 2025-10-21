// src/main/java/frc/robot/commands/autonomous/algae/AlgaeDetachFromReefSequence.java
package frc.robot.commands.autonomous.algae;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
// Quita imports no usados si IntakeUntilCurrentAlgae ya no está
// import frc.robot.commands.extensor.gripper.algae.IntakeUntilCurrentAlgae;
// import frc.robot.commands.extensor.lift.control.LiftDefaultCommand; // <-- Aún usado por
// AlgaeRemovalFromTargetID? Verifica si es necesario.
import frc.robot.commands.swerve.autonomous.offSeason.BackoffFromCurrentPose;
import frc.robot.commands.swerve.autonomous.offSeason.DriveToAlgaeRemovalFromReef;

public class AlgaeDetachFromReefSequence extends SequentialCommandGroup {

    /**
     * Secuencia para alinearse al reef, mover el brazo a la posición detectada, hacer backoff y
     * mover el brazo a la posición de HOLD. **NOTA:** Esta versión NO incluye el intake del
     * gripper.
     */
    public AlgaeDetachFromReefSequence(double vMax, double wMax, double approachM,
            double backoffM) {

        // --- Comando para manejar y bloquear el tag ---
        var driveLock = new DriveToAlgaeRemovalFromReef(vMax, wMax, approachM)
                .withTimeout(HighAltitudeConstants.DRIVE_TO_POSE_TIMEOUT_S);

        // --- Comando para actualizar el nivel detectado en OIHelpers ---
        var selectAlgaeLevel = new UpdateAlgaePoseFromVision();

        // --- Comando para mover Lift/Wrist al nivel detectado ---
        var moveLiftToDetected = new AlgaeRemovalFromTargetID(
                AlgaeRemovalFromTargetID.StateAction.MOVE_TO_DETECTED_LEVEL);

        // --- Comando para hacer backoff ---
        var backoff = new BackoffFromCurrentPose(backoffM, vMax, wMax)
                .withTimeout(HighAltitudeConstants.DRIVE_TO_POSE_TIMEOUT_S);

        // --- Comando para mover Lift/Wrist a la posición HOLD ---
        var moveToHold =
                new AlgaeRemovalFromTargetID(AlgaeRemovalFromTargetID.StateAction.MOVE_TO_HOLD);

        // === SECUENCIA SIN GRIPPER ===
        addCommands(
                // Paso 1: Moverse Y seleccionar el nivel AL MISMO TIEMPO.
                new ParallelRaceGroup(driveLock, selectAlgaeLevel),

                // Paso 2: Mover el brazo al nivel que se "latchó".
                moveLiftToDetected,

                // Paso 3: Hacer backoff (alejarse).
                backoff,

                // Paso 4: Mover el brazo a la posición HOLD.
                moveToHold);
    }
}
