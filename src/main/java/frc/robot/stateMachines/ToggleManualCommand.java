// src/main/java/frc/robot/stateMachines/ToggleManualCommand.java
package frc.robot.stateMachines;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;

/**
 * Toggle de MODO MANUAL:
 * - Si NO estás en MANUAL, guarda el modo actual y entra a MANUAL.
 * - Si YA estás en MANUAL, regresa al último modo no-manual guardado.
 */
public class ToggleManualCommand extends InstantCommand {
  private final boolean cancelRunning;

  /** Por defecto cancela lo que esté corriendo al hacer el toggle. */
  public ToggleManualCommand() {
    this(true);
  }

  public ToggleManualCommand(boolean cancelRunning) {
    this.cancelRunning = cancelRunning;
  }

  // Último modo no-manual recordado (scope global al comando)
  private static GameMode lastNonManualMode = GameMode.CORAL_LX; // fallback sensato

  @Override
  public void initialize() {
    var rc = Robot.getRobotContainer();
    var cur = rc.getGameMode();

    GameMode next;
    if (cur == GameMode.MANUAL) {
      // Salir de Manual → regresar al último modo no-manual
      next = lastNonManualMode;
    } else {
      // Entrar a Manual → recordar el modo actual para regresar después
      lastNonManualMode = cur;
      next = GameMode.MANUAL;
    }

    if (cancelRunning) {
      CommandScheduler.getInstance().cancelAll();
    }
    rc.setGameMode(next);
  }
}
