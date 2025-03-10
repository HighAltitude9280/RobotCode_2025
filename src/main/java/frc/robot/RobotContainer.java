// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.AutoLeave;
import frc.robot.commands.autonomous.LeaveAndL4;
import frc.robot.commands.extensor.lift.control.LiftDefaultCommand;
import frc.robot.commands.extensor.wrist.control.WristDefaultCommand;
import frc.robot.commands.leds.SetLEDColor;
import frc.robot.commands.swerve.DefaultSwerveDriveNew;
import frc.robot.resources.components.Navx;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.extensor.Climber;
import frc.robot.subsystems.extensor.Lift;
import frc.robot.subsystems.extensor.Wrist;
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.subsystems.vision.Vision;

/** Add your docs here. */
public class RobotContainer {

    private Navx navx;
    private SwerveDriveTrain swerveDriveTrain;
    private Vision vision;
    private Lift lift;
    private Gripper gripper;
    private Wrist wrist;
    private CANdleSubsystem candleSubsystem;
    private Climber climber;

    private boolean precisionModeOn = false;
    private boolean overrideEncoders = false;

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        vision = new Vision();
        navx = new Navx();
        swerveDriveTrain = new SwerveDriveTrain();
        lift = new Lift();
        gripper = new Gripper();
        wrist = new Wrist();
        candleSubsystem = new CANdleSubsystem();
        climber = new Climber();
    }

    public Navx getNavx() {
        return navx;
    }

    public Vision getVision() {
        return vision;
    }

    public SwerveDriveTrain getSwerveDriveTrain() {
        return swerveDriveTrain;
    }

    public Lift getLift() {
        return lift;
    }

    public Gripper getGripper() {
        return gripper;
    }

    public Wrist getWrist() {
        return wrist;
    }

    public CANdleSubsystem getCaNdleSubsystem() {
        return candleSubsystem;
    }

    public Climber getClimber() {
        return climber;
    }

    public void setOverrideEncoders(boolean override) {
        overrideEncoders = override;
    }

    public boolean getOverrideEncoders() {
        return overrideEncoders;
    }

    public void setPrecisionMode(boolean precisionMode) {
        precisionModeOn = precisionMode;
    }

    public boolean getPrecisionMode() {
        return precisionModeOn;
    }

    public void ConfigureButtonBindings() {
        OI.getInstance().ConfigureButtonBindings();
        swerveDriveTrain.setDefaultCommand(new DefaultSwerveDriveNew());

        // TODO: Crear un comando manual por si deja de funcionar el PID (que overridee
        // el default command).

        wrist.setDefaultCommand(new WristDefaultCommand(HighAltitudeConstants.WRIST_DRIVE_SPEED));
        lift.setDefaultCommand(new LiftDefaultCommand(HighAltitudeConstants.LIFT_MAX_POWER,
                HighAltitudeConstants.LIFT_ARRIVE_OFFSET));

        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case Joakin:
                break;

            default:

                break;
        }

        switch (HighAltitudeConstants.CURRENT_COPILOT) {
            case Joakin:
                break;

            default:
                break;
        }

        // swerveDriveTrain.setDefaultCommand(new DefaultSwerveDriveNew());
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    public void putAutoChooser() {
        SmartDashboard.putData("Autonomous", m_chooser);
    }

    public void generateAutos() {

        m_chooser.setDefaultOption("Nothing", new WaitCommand(0));
        m_chooser.addOption("AutoLeave", new AutoLeave(1.5, 0.7));
        m_chooser.addOption("Leave and L4", new LeaveAndL4());

    }
}
