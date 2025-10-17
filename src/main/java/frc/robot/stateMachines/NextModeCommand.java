// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.stateMachines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;

public class NextModeCommand extends InstantCommand {
  @Override
  public void initialize() {
    var rc = Robot.getRobotContainer();
    var cur = rc.getGameMode();
    GameMode next = switch (cur) {
      case CORAL_L1 -> GameMode.CORAL_LX;
      case CORAL_LX -> GameMode.ALGAE;
      case ALGAE -> GameMode.CORAL_L1;
      case MANUAL -> GameMode.CORAL_LX;
    };
    rc.setGameMode(next);
    System.out.println("Acabo de cambiar a comando: " + next);

  }
}
