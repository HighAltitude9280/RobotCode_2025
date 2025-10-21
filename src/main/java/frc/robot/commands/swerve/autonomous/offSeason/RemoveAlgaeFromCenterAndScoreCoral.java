// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.autonomous.offSeason;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.PoseIdx;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.HighAltitudeConstantsPose;
import frc.robot.HighAltitudeConstantsPose.REEF_POSITION;
import frc.robot.commands.autonomous.ScoreCoral;
import frc.robot.commands.extensor.compound.both.LiftWristGoToPose;
import frc.robot.commands.oneDriver.DriveToPose;
// --- Importa todos los comandos y constantes que necesitarás ---
import frc.robot.subsystems.swerve.SwerveDriveTrain;

/**
 * Autónomo que primero se mueve a la posición central para quitar el alga (subiendo el elevador a
 * L3), y luego va a la REEF_POSITION.FC para anotar.
 */
public class RemoveAlgaeFromCenterAndScoreCoral extends SequentialCommandGroup {

  /** Creates a new RemoveAlgaeFromCenterAndScoreCoral. */
  public RemoveAlgaeFromCenterAndScoreCoral() {

    var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    boolean isBlue = alliance == DriverStation.Alliance.Blue;

    // --- 1. Pose para quitar Alga ---
    Pose2d algaeTargetPose = isBlue ? new Pose2d(5.69, 4.043, Rotation2d.fromDegrees(180)) // blue
                                                                                           // pose
        : new Pose2d(11.95, 4.043, Rotation2d.fromDegrees(180)); // red pose

    // --- 2. Pose para anotar en REEF (FC) ---
    Pose2d[] reefPositions = isBlue ? HighAltitudeConstantsPose.PATHFINDING_BLUE_REEF_POS
        : HighAltitudeConstantsPose.PATHFINDING_RED_REEF_POS;

    // Obtenemos la pose específica de FC (Front Center)
    Pose2d reefFCPose = reefPositions[REEF_POSITION.FC.getID()];

    //
    // --- Añade la secuencia de comandos ---
    //
    addCommands(
        // PASO 1: DriveToPose (usando pathfinding) a la pose del alga
        new DriveToPose(algaeTargetPose).withTimeout(4.0),

        // PASO 2: Subir el elevador a L3 para quitar el alga
        new LiftWristGoToPose(PoseIdx.L3)/* .withTimeout(2.5) */, // Poner timeout es buena idea

        SwerveDriveTrain.pathfindToPose(reefFCPose, 3.0),

        // PASO 4: Secuencia final de alineación fina y anotación
        new SequentialCommandGroup(
            new DriveToTargetBranchPose(REEF_POSITION.FC, null, true,
                HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                HighAltitudeConstants.VISION_POSE_MAX_TURN).withTimeout(3.0),

            new ScoreCoral(REEF_HEIGHT.L3)));
  }
}
