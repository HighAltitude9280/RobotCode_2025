// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.autonomous.reef;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.HighAltitudeConstantsPose;
import frc.robot.HighAltitudeConstantsPose.REEF_POSITION;
import frc.robot.HighAltitudeConstantsPose.REEF_SIDE;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

public class PathplanToReef extends InstantCommand {
  REEF_POSITION pos = null;

  REEF_SIDE side = null;

  public PathplanToReef(REEF_POSITION pos) {
    this.pos = pos;
  }

  public PathplanToReef(REEF_SIDE side) {
    this.side = side;
  }

  public PathplanToReef() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (pos == null) {
      if (side == null)
        side = Robot.getReefMode();

      pos = side.getPosition(Robot.isFrontMode());

    }

    Pose2d targetPose;
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
      targetPose = HighAltitudeConstantsPose.PATHFINDING_RED_REEF_POS[pos.getID()];
    else
      targetPose = HighAltitudeConstantsPose.PATHFINDING_BLUE_REEF_POS[pos.getID()];

    SwerveDriveTrain.pathfindToPose(targetPose).schedule();
  }
}
