// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.coral; // o donde sea que la pongas

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.commands.swerve.autonomous.offSeason.DriveToTargetBranchPose;

public class DriveToTargetBranchAndScore extends SequentialCommandGroup {
  /**
   * * Crea una nueva secuencia para alinearse a la rama (branch) y luego anotar en la pose (índice)
   * especificada.
   */
  public DriveToTargetBranchAndScore(boolean left, int poseIdx) { // <-- CORREGIDO: "int" en lugar
                                                                  // de "PoseIdx"

    addCommands(
        // 1. Se alinea con la rama
        new DriveToTargetBranchPose(left, HighAltitudeConstants.VISION_POSE_MAX_SPEED,
            HighAltitudeConstants.VISION_POSE_MAX_TURN, true).withTimeout(3),

        // 2. Ejecuta el comando de anotación usando el índice
        new ScoreAtPose(poseIdx));
  }
}
