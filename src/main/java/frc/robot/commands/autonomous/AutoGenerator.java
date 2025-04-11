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
import frc.robot.commands.compound.LiftWristGoToTargetHeight;
import frc.robot.commands.compound.ScoreGamePieceLiftDown;
import frc.robot.commands.extensor.gripper.IntakeUntilCurrentCoral;
import frc.robot.commands.swerve.autonomous.AlignVisionMoveMeters;
import frc.robot.commands.swerve.autonomous.SwerveMoveMeters;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoGenerator extends InstantCommand 
{
  ArrayList<AutoPortion> autoPath;
  public AutoGenerator(ArrayList<AutoPortion> autoPath) {
    this.autoPath = autoPath;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    ArrayList<Command> commands = new ArrayList<>();

    
    boolean blueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

    for(var portion : autoPath)
    {
      Pose2d[] positions;
      Pose2d feeder;

      if(blueAlliance)
      {
        positions = HighAltitudeConstants.PATHFINDING_BLUE_REEF_POS;
        if(portion.isLeftFeeder() == null) feeder = null;

        else feeder = portion.isLeftFeeder() ? HighAltitudeConstants.PATHFINDING_LEFT_BLUE_FEEDER : 
          HighAltitudeConstants.PATHFINDING_RIGHT_BLUE_FEEDER;        
      }
      else
      {
        positions = HighAltitudeConstants.PATHFINDING_RED_REEF_POS;

        if(portion.isLeftFeeder() == null) feeder = null;
        
        else feeder = portion.isLeftFeeder() ? HighAltitudeConstants.PATHFINDING_LEFT_RED_FEEDER : 
          HighAltitudeConstants.PATHFINDING_RIGHT_RED_FEEDER;        
      }
      
      Pose2d reef = positions[portion.getPos().getID()];
      commands.add(SwerveDriveTrain.pathfindToPose(reef));

      commands.add(new ParallelDeadlineGroup(new AlignVisionMoveMeters(portion.getPos(),null, portion.isLeftBranch()), 
                      new LiftWristGoToTargetHeight(portion.getHeight())));
      commands.add(new ScoreGamePieceLiftDown());
      if(feeder != null)
      {
        commands.add(SwerveDriveTrain.pathfindToPose(feeder));
        commands.add(new SwerveMoveMeters(-0.5, feeder.getRotation().getDegrees(), 0.1).withTimeout(0.2));
        commands.add(new IntakeUntilCurrentCoral());
      }
    }

    new SequentialCommandGroup(commands.toArray(new Command[0])).schedule();
  }
}
