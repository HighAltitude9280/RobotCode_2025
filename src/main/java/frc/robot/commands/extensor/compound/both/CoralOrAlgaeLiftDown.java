// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extensor.compound.both;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.extensor.compound.algae.KeepAlgaeSafe;
import frc.robot.commands.extensor.compound.coral.ScoreGamePieceLiftDown;
import frc.robot.subsystems.extensor.Lift;
import frc.robot.subsystems.extensor.Wrist;
import frc.robot.subsystems.manipulator.Gripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralOrAlgaeLiftDown extends InstantCommand {
  Lift lift;
  Wrist wrist;
  Gripper gripper;

  public CoralOrAlgaeLiftDown() {
    lift = Robot.getRobotContainer().getLift();
    wrist = Robot.getRobotContainer().getWrist();
    gripper = Robot.getRobotContainer().getGripper();
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(lift, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (Robot.isCoralMode())
      new ScoreGamePieceLiftDown().schedule();
    else {
      new KeepAlgaeSafe().schedule();
    }
  }
}
