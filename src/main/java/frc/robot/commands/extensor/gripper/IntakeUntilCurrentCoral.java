// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extensor.gripper;

import java.security.Timestamp;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HighAltitudeConstants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.manipulator.Gripper;

public class IntakeUntilCurrentCoral extends Command {
  private final Gripper gripper;
  double nextOffTime = -1;
  double nextOnTime = -1;

  /** Creates a new IntakeUntilCurrentCoral. */
  public IntakeUntilCurrentCoral() {
    gripper = Robot.getRobotContainer().getGripper();
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gripper.gripperInCurrent();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Asegurar que el gripper sigue encendido hasta que la corriente supere el
    // umbral
    if (!gripper.isCurrentThresholdExceeded()) {
      if (MathSharedStore.getTimestamp() > nextOnTime) {
        OI.getInstance().getPilot().getJoystick().setRumble(RumbleType.kBothRumble, 0);
        nextOffTime = MathSharedStore.getTimestamp() + 0.2;
      }

      if (MathSharedStore.getTimestamp() > nextOffTime) {
        OI.getInstance().getPilot().getJoystick().setRumble(RumbleType.kBothRumble, 1);
        nextOnTime = MathSharedStore.getTimestamp() + 5;
      }
    } else {
      gripper.gripperInCurrent();

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripper.stopGripper();
    OI.getInstance().getPilot().getJoystick().setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gripper.isCurrentThresholdExceeded();
  }
}
