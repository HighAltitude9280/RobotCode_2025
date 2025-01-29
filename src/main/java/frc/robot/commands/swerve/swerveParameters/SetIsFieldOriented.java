// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.swerveParameters;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIsFieldOriented extends InstantCommand {
  boolean shouldBeFieldOriented;
  SwerveDriveTrain swerveDriveTrain;

  public SetIsFieldOriented(boolean shouldBeFieldOriented) {
    this.shouldBeFieldOriented = shouldBeFieldOriented;

    swerveDriveTrain = Robot.getRobotContainer().getSwerveDriveTrain();
    addRequirements(swerveDriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDriveTrain.setIsFieldOriented(shouldBeFieldOriented);
  }
}
