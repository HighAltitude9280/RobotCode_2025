// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extensor.compound.algae;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HighAltitudeConstants.PoseIdx;
import frc.robot.commands.extensor.compound.both.LiftWristGoToPose;
import frc.robot.commands.extensor.gripper.algae.IntakeAlgaeAuto;
import frc.robot.commands.extensor.gripper.manual.IntakeAlgae;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeIntakeFloor extends SequentialCommandGroup {
  /** Creates a new AlgaeIntake. */
  public AlgaeIntakeFloor() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelDeadlineGroup(new WaitCommand(1), new IntakeAlgae(),
        new LiftWristGoToPose(PoseIdx.ALGAE_INTAKE_FLOOR)), new WaitCommand(0.5), new IntakeAlgaeAuto());
  }
}
