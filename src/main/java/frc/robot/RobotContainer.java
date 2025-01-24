// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.resources.components.Navx;

/** Add your docs here. */
public class RobotContainer {

    private Navx navx;

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        navx = new Navx();

    }

    public Navx getNavx() {
        return navx;
    }

    public void ConfigureButtonBindings() {
        OI.getInstance().ConfigureButtonBindings();
        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case Joakin:
                break;

            default:

                break;
        }

        switch (HighAltitudeConstants.CURRENT_COPILOT) {
            default:
                break;
        }

        // swerveDriveTrain.setDefaultCommand(new DefaultSwerveDriveNew());
        // climber.setDefaultCommand(new MaintainClimberPosition());
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    public void putAutoChooser() {
        SmartDashboard.putData("Autonomous", m_chooser);
    }

    public void generateAutos() {

        m_chooser.setDefaultOption("Nothing", new WaitCommand(0));

    }
}
