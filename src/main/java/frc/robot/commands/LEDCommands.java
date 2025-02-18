package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSubsystem;

public class LEDCommands extends Command {
  private final CANdleSubsystem candleSubsystem;
  private final int r, g, b;

  public LEDCommands(CANdleSubsystem subsystem, int red, int green, int blue) {
    candleSubsystem = subsystem;
    r = red;
    g = green;
    b = blue;
    addRequirements(candleSubsystem);
  }

  @Override
  public void initialize() {
    candleSubsystem.setLEDColor(r, g, b);
  }

  @Override
  public boolean isFinished() {
    return true; // Solo cambia el color y termina
  }
}