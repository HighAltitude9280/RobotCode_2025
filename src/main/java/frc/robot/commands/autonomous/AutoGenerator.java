// AutoGenerator.java
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
      // ********** REEF Portion **********
      // 1. Get final REEF pose from the appropriate array (depending on alliance)
      Pose2d[] reefPositions = blueAlliance ? HighAltitudeConstants.PATHFINDING_BLUE_REEF_POS
          : HighAltitudeConstants.PATHFINDING_RED_REEF_POS;
      Pose2d reefFinal = reefPositions[portion.getPos().getID()];
      // 2. Calculate approach pose for REEF (intakeMode = false)
      Pose2d approachReef = portion.getApproachPose(reefFinal, false);
      // 3. Execute coarse pathfinding to REEF approach pose
      commands.add(SwerveDriveTrain.pathfindToPose(approachReef));
      // 4. Execute fine alignment for REEF using AlignWithTargetPose (and ScoreCoral)
      commands.add(new SequentialCommandGroup(
          new AlignWithTargetPose(portion.getPos(), null, portion.isLeftBranch(),
              HighAltitudeConstants.VISION_POSE_MAX_SPEED, HighAltitudeConstants.VISION_POSE_MAX_TURN),
          new ScoreCoral(portion.getHeight())));

      // ********** CORAL STATION Portion **********
      // 5. Get final Coral Station pose from the corresponding array using
      // coralStationPos
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
      // 6. Calculate approach pose for Coral Station (intake mode = true)
      Pose2d approachCoral = portion.getApproachPose(coralFinal, true);
      // 7. Execute coarse pathfinding to the Coral Station approach pose
      commands.add(SwerveDriveTrain.pathfindToPose(approachCoral));
      // 8. Execute fine alignment for Coral Station using DriveToCoralStation
      commands.add(new DriveToCoralStation(portion.getCoralStationPos(), portion.isLeftFeeder(),
          HighAltitudeConstants.VISION_POSE_MAX_SPEED, HighAltitudeConstants.VISION_POSE_MAX_TURN));
      // 9. Execute intake using IntakeAuto (to pick up the game piece)
      commands.add(new IntakeAuto());
    }

    // Schedule the sequential command group with all assembled commands.
    new SequentialCommandGroup(commands.toArray(new Command[0])).schedule();
  }
}
