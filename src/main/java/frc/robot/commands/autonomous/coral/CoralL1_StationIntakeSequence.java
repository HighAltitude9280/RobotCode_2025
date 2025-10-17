// src/main/java/frc/robot/commands/autonomous/coral/CoralL1_StationIntakeSequence.java
package frc.robot.commands.autonomous.coral;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.commands.extensor.gripper.algae.IntakeAlgaeAuto;
import frc.robot.commands.extensor.gripper.manual.HoldAlgae; // <-- IMPORTANTE
import frc.robot.commands.swerve.autonomous.offSeason.DriveToCoralStation;

public class CoralL1_StationIntakeSequence extends SequentialCommandGroup {
  public CoralL1_StationIntakeSequence(boolean left) {

    var driveL1 = new DriveToCoralStation(/* L1 = */ true, /* left = */ left,
        HighAltitudeConstants.VISION_POSE_MAX_SPEED, HighAltitudeConstants.VISION_POSE_MAX_TURN);

    var intake = new IntakeAlgaeAuto().withTimeout(HighAltitudeConstants.ALGAE_INTAKE_TIMEOUT_S);

    var holdSticky = new ScheduleCommand(new HoldAlgae());

    addCommands(driveL1, intake, holdSticky // <- se agenda y permanece hasta que otro comando del
                                            // gripper lo reemplace
    );
  }
}
