// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.commands.extensor.gripper.IntakeAuto;
import frc.robot.commands.swerve.autonomous.feeder.DriveToCoralStation;
import frc.robot.commands.swerve.autonomous.reef.AlignWithTargetPose;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

public class AutoGenerator extends InstantCommand {
  ArrayList<AutoPortion> autoPath;

  public AutoGenerator(ArrayList<AutoPortion> autoPath) {
    this.autoPath = autoPath;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArrayList<Command> commands = new ArrayList<>();

    boolean blueAlliance = DriverStation.getAlliance()
        .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

    for (var portion : autoPath) {
      // ********* REEF Portion *********
      // 1. Get the final REEF pose from the REEF array (depending on alliance)
      Pose2d[] reefPositions;
      if (blueAlliance) { 
        reefPositions = HighAltitudeConstants.PATHFINDING_BLUE_REEF_POS;
      } else {
        reefPositions = HighAltitudeConstants.PATHFINDING_RED_REEF_POS;
      }
      Pose2d reefFinal = reefPositions[portion.getPos().getID()];

      // 2. Calculate the approach pose for the REEF
      Pose2d approachReef = portion.getApproachPose(reefFinal);

      // 3. Execute PathOnTheFly to the REEF approach pose
      commands.add(SwerveDriveTrain.pathfindToPose(approachReef));

      // 4. Execute fine alignment with AlignWithTargetPose to reach the exact REEF
      // position
      // (ScoreCoral is executed concurrently to score the preloaded game piece)
      commands.add(new SequentialCommandGroup(
          new AlignWithTargetPose(portion.getPos(), null, portion.isLeftBranch(),
              HighAltitudeConstants.VISION_POSE_MAX_SPEED, HighAltitudeConstants.VISION_POSE_MAX_TURN),
          new ScoreCoral(portion.getHeight())));

      // ********* CORAL STATION Portion *********
      // 5. Get the final CORAL STATION pose from the coral arrays (using
      // coralStationPos and leftFeeder)
      Pose2d coralFinal;
      if (blueAlliance) {
        if (portion.isLeftFeeder() != null) {
          coralFinal = portion.isLeftFeeder()
              ? HighAltitudeConstants.PATHFINDING_BLUE_LEFT_CORAL_STATION[portion.getCoralStationPos().getID()]
              : HighAltitudeConstants.PATHFINDING_BLUE_RIGHT_CORAL_STATION[portion.getCoralStationPos().getID()];
        } else {
          coralFinal = HighAltitudeConstants.PATHFINDING_BLUE_LEFT_CORAL_STATION[portion.getCoralStationPos().getID()];
        }
      } else {
        if (portion.isLeftFeeder() != null) {
          coralFinal = portion.isLeftFeeder()
              ? HighAltitudeConstants.PATHFINDING_RED_LEFT_CORAL_STATION[portion.getCoralStationPos().getID()]
              : HighAltitudeConstants.PATHFINDING_RED_RIGHT_CORAL_STATION[portion.getCoralStationPos().getID()];
        } else {
          coralFinal = HighAltitudeConstants.PATHFINDING_RED_LEFT_CORAL_STATION[portion.getCoralStationPos().getID()];
        }
      }

      // 6. Calculate the approach pose for the CORAL STATION
      Pose2d approachCoral = portion.getApproachPose(coralFinal);

      // 7. Execute PathOnTheFly to the CORAL STATION approach pose
      commands.add(SwerveDriveTrain.pathfindToPose(approachCoral));

      // 8. Execute fine alignment using DriveToCoralStation to reach the exact CORAL
      // STATION position
      commands.add(new DriveToCoralStation(portion.getCoralStationPos(), portion.isLeftFeeder(),
          HighAltitudeConstants.VISION_POSE_MAX_SPEED, HighAltitudeConstants.VISION_POSE_MAX_TURN));

      // 9. (Optional) Execute intake to pick up game piece from the feeder
      commands.add(new IntakeAuto());
    }

    // Schedule the sequential command group with all assembled commands.
    new SequentialCommandGroup(commands.toArray(new Command[0])).schedule();
  }
}
