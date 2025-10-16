// src/main/java/frc/robot/stateMachines/ChangeModeCommand.java
package frc.robot.stateMachines;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;

/**
 * Cambia el modo del robot. Opcionalmente cancela lo que esté corriendo
 * (recomendado si tienes bindings whileTrue activos en el modo anterior).
 */
public class ChangeModeCommand extends InstantCommand {
  private final GameMode newMode;
  private final boolean cancelRunning;

  /** Por defecto cancela comandos en curso. */
  public ChangeModeCommand(GameMode newMode) {
    this(newMode, true);
  }

  public ChangeModeCommand(GameMode newMode, boolean cancelRunning) {
    this.newMode = newMode;
    this.cancelRunning = cancelRunning;
  }

  @Override
  public void initialize() {
    if (cancelRunning) {
      CommandScheduler.getInstance().cancelAll();
    }
    Robot.getRobotContainer().setGameMode(newMode); // Actualiza Dashboard dentro del setter
  }
}
