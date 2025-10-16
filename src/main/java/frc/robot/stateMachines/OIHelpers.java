// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.stateMachines;

import java.util.EnumSet;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Robot.GameMode;

public final class OIHelpers {
  private OIHelpers() {
  }

  public static Command onlyInMode(Supplier<GameMode> modeSup, GameMode required, Command inner) {
    return new ConditionalCommand(inner, Commands.none(), () -> modeSup.get() == required);
  }

  public static Command onlyInModes(Supplier<GameMode> modeSup, EnumSet<GameMode> allowed, Command inner) {
    return new ConditionalCommand(inner, Commands.none(), () -> allowed.contains(modeSup.get()));
  }
}