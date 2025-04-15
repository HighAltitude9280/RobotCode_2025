// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.CORAL_STATION_POSITION;
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
      // If coralStationPos is not null, handle as a Coral Station portion
      if (portion.getCoralStationPos() != null) {
        Pose2d coralFinal;
        if (blueAlliance) {
          if (portion.isLeftFeeder() != null) {
            coralFinal = portion.isLeftFeeder()
                ? HighAltitudeConstants.PATHFINDING_BLUE_LEFT_CORAL_STATION[portion.getCoralStationPos().getID()]
                : HighAltitudeConstants.PATHFINDING_BLUE_RIGHT_CORAL_STATION[portion.getCoralStationPos().getID()];
          } else {
            coralFinal = HighAltitudeConstants.PATHFINDING_BLUE_LEFT_CORAL_STATION[portion.getCoralStationPos()
                .getID()];
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
        // Calculate the approach pose for the Coral Station portion
        Pose2d approachCoral = portion.getApproachPose(coralFinal);
        // First stage: drive to the approach pose (coarse movement)
        commands.add(SwerveDriveTrain.pathfindToPose(approachCoral));
        // Second stage: fine alignment using DriveToCoralStation for Coral Station
        // portions
        commands.add(new DriveToCoralStation(portion.getCoralStationPos(), portion.isLeftFeeder(),
            HighAltitudeConstants.VISION_POSE_MAX_SPEED, HighAltitudeConstants.VISION_POSE_MAX_TURN));
      } else {
        // Process as a REEF portion using the REEF logic
        Pose2d[] positions;
        Pose2d feeder;
        if (blueAlliance) {
          positions = HighAltitudeConstants.PATHFINDING_BLUE_REEF_POS;
          if (portion.isLeftFeeder() == null) {
            feeder = null;
          } else {
            feeder = portion.isLeftFeeder()
                ? HighAltitudeConstants.PATHFINDING_LEFT_BLUE_FEEDER
                : HighAltitudeConstants.PATHFINDING_RIGHT_BLUE_FEEDER;
          }
        } else {
          positions = HighAltitudeConstants.PATHFINDING_RED_REEF_POS;
          if (portion.isLeftFeeder() == null) {
            feeder = null;
          } else {
            feeder = portion.isLeftFeeder()
                ? HighAltitudeConstants.PATHFINDING_LEFT_RED_FEEDER
                : HighAltitudeConstants.PATHFINDING_RIGHT_RED_FEEDER;
          }
        }
        // Get the final REEF pose using the portion's REEF_POSITION
        Pose2d reef = positions[portion.getPos().getID()];
        // Calculate the approach pose for the REEF portion
        Pose2d approachReef = portion.getApproachPose(reef);
        // First stage: drive to the approach pose (coarse movement)
        commands.add(SwerveDriveTrain.pathfindToPose(approachReef));
        // Second stage: fine alignment using AlignWithTargetPose for REEF portions
        commands.add(new ParallelDeadlineGroup(
            new AlignWithTargetPose(portion.getPos(), null, portion.isLeftBranch(),
                HighAltitudeConstants.VISION_POSE_MAX_SPEED, HighAltitudeConstants.VISION_POSE_MAX_TURN),
            new ScoreCoral(portion.getHeight())));
        // Feeder block for REEF portions: if a feeder is defined, use
        // DriveToCoralStation for fine alignment
        if (feeder != null) {
          // Coarse movement: drive to feeder pose
          commands.add(SwerveDriveTrain.pathfindToPose(feeder));
          // Fine alignment at feeder using DriveToCoralStation (using
          // CORAL_STATION_POSITION.MIDDLE as default)
          commands.add(new DriveToCoralStation(CORAL_STATION_POSITION.MIDDLE, portion.isLeftFeeder(),
              HighAltitudeConstants.VISION_POSE_MAX_SPEED, HighAltitudeConstants.VISION_POSE_MAX_TURN));
          // Execute intake using IntakeAuto command
          commands.add(new IntakeAuto());
        }
      }
    }

    // Schedule the sequential command group with all assembled commands.
    new SequentialCommandGroup(commands.toArray(new Command[0])).schedule();
  }
}
