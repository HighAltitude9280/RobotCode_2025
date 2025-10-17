// src/main/java/frc/robot/commands/autonomous/coral/CoralLX_StationIntakeSequence.java
package frc.robot.commands.autonomous.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.commands.extensor.gripper.IntakeAuto;
import frc.robot.commands.swerve.autonomous.offSeason.DriveToCoralStation;

public class CoralLX_StationIntakeSequence extends SequentialCommandGroup {
  public CoralLX_StationIntakeSequence(boolean left) {

    var drive = new DriveToCoralStation(false, null, left,
        HighAltitudeConstants.VISION_POSE_MAX_SPEED, HighAltitudeConstants.VISION_POSE_MAX_TURN);

    var intake = new IntakeAuto();

    addCommands(drive, intake);
  }
}
