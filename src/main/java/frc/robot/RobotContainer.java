// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.HighAltitudeConstants.REEF_POSITION;
import frc.robot.commands.autonomous.AutoGenerator;
import frc.robot.commands.autonomous.AutoLeave;
import frc.robot.commands.autonomous.AutoPortion;
import frc.robot.commands.autonomous.ScoreCoral;
import frc.robot.commands.autonomous.center.Center2L4Left;
import frc.robot.commands.autonomous.center.Center2L4Right;
import frc.robot.commands.autonomous.center.LeaveAndL4;
import frc.robot.commands.compound.ScoreGamePieceLiftDown;
import frc.robot.commands.extensor.gripper.manual.ScoreGamePiece;
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
    private boolean generalBrakingMode = true;

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

    public void setBrakeModeAllMotors(boolean brake) {
        wrist.setBrakeModeAllMotors(brake);
        lift.setBrakeModeAllMotors(brake);
        swerveDriveTrain.setBrakeModeAllMotors(brake);
    }

    public void toggleBrakeModeAllMotors() {
        setBrakeModeAllMotors(!generalBrakingMode);
        generalBrakingMode = !generalBrakingMode;
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
        m_chooser.addOption("2L4 Center Right", new PathPlannerAuto(new Center2L4Right()));
        m_chooser.addOption("2L4 Center Left", new PathPlannerAuto(new Center2L4Left()));

        m_chooser.addOption("PID TEST", new PathPlannerAuto("Translation PID"));

        m_chooser.addOption("L4Right", new PathPlannerAuto("L4Right"));
        m_chooser.addOption("L4Left", new PathPlannerAuto("L4Left"));

        m_chooser.addOption("L4Center", new PathPlannerAuto("L4Center"));

        NamedCommands.registerCommand("ScoreCoralL4", new ScoreCoral(REEF_HEIGHT.TOP));
        NamedCommands.registerCommand("AutoLeave", new AutoLeave(2, 0.8).withTimeout(3.4));

        m_chooser.addOption("Orbit Right", new AutoGenerator(new ArrayList<>(Arrays.asList(
                new AutoPortion(REEF_POSITION.FR, true, REEF_HEIGHT.TOP, true),
                new AutoPortion(REEF_POSITION.BR, true, REEF_HEIGHT.TOP, true),
                new AutoPortion(REEF_POSITION.BR, false, REEF_HEIGHT.TOP, true),
                new AutoPortion(REEF_POSITION.BC, false, REEF_HEIGHT.TOP, null)))));
        m_chooser.addOption("Orbit Left", new AutoGenerator(new ArrayList<>(Arrays.asList(
                new AutoPortion(REEF_POSITION.FL, false, REEF_HEIGHT.TOP, true),
                new AutoPortion(REEF_POSITION.BL, true, REEF_HEIGHT.TOP, true),
                new AutoPortion(REEF_POSITION.BL, false, REEF_HEIGHT.TOP, true),
                new AutoPortion(REEF_POSITION.BC, false, REEF_HEIGHT.TOP, null)))));
    }
}
