// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.commands.SetLEDColor;
import frc.robot.commands.compound.LiftWristGoToReefHeight;
import frc.robot.commands.compound.LiftWristIntakeAlgae;
import frc.robot.commands.gripper.manual.ScoreGamePiece;
import frc.robot.commands.gripper.IntakeUntilCoral;
import frc.robot.commands.gripper.manual.IntakeAlgae;
import frc.robot.commands.lift.control.LiftSetMetersTarget;
import frc.robot.commands.modes.SetCoralMode;
import frc.robot.commands.modes.WhileHeldPrecisionMode;
import frc.robot.commands.swerve.swerveParameters.ResetOdometryZeros;
import frc.robot.commands.swerve.swerveParameters.SetIsFieldOriented;
import frc.robot.commands.swerve.test.TestDirectionPIDSwerve;
import frc.robot.commands.swerve.test.TestSwerve;
import frc.robot.commands.wrist.control.WristMantainTarget;
import frc.robot.resources.joysticks.HighAltitudeJoystick;
import frc.robot.resources.joysticks.HighAltitudeJoystick.AxisType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.ButtonType;
import frc.robot.resources.joysticks.HighAltitudeJoystick.JoystickType;

/** Add your docs here. */
public class OI {
    public static OI instance;

    private HighAltitudeJoystick pilot;
    private HighAltitudeJoystick copilot;

    public void ConfigureButtonBindings() {
        ////////////////////////// PILOT //////////////////////////

        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case Joakin:

                pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);

                pilot.setAxisDeadzone(AxisType.LEFT_X, 0.06);
                pilot.setAxisDeadzone(AxisType.LEFT_Y, 0.06);
                pilot.setAxisDeadzone(AxisType.RIGHT_X, 0.06);

                pilot.onTrue(ButtonType.BACK, new SetIsFieldOriented(true));
                pilot.onTrue(ButtonType.START, new SetIsFieldOriented(false));
                pilot.onTrueCombo(new ResetOdometryZeros(), ButtonType.START, ButtonType.BACK);

                pilot.whileTrue(ButtonType.LB, new ScoreGamePiece());
                pilot.whileTrue(ButtonType.RB, new IntakeAlgae());

                pilot.whileTrue(ButtonType.POV_E, new WhileHeldPrecisionMode());

            case JoakinButChambing:

                pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);

                pilot.setAxisDeadzone(AxisType.LEFT_X, 0.2);
                pilot.setAxisDeadzone(AxisType.LEFT_Y, 0.2);
                pilot.setAxisDeadzone(AxisType.RIGHT_X, 0.2);

                pilot.onTrue(ButtonType.BACK, new SetIsFieldOriented(true));
                pilot.onTrue(ButtonType.START, new SetIsFieldOriented(false));
                pilot.onTrue(ButtonType.X, new ResetOdometryZeros());

                pilot.whileTrue(ButtonType.Y, new TestDirectionPIDSwerve());
                // pilot.whileTrue(ButtonType.POV_N, new TestDrivePIDFFSwerve(0.5));
                // pilot.whileTrue(ButtonType.POV_S, new TestDrivePIDFFSwerve(-0.5));
                pilot.whileTrue(ButtonType.B, new TestSwerve());
            default:
                break;

        }
        switch (HighAltitudeConstants.CURRENT_COPILOT) {

            case Carlos:

                copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);

                copilot.whileTrue(ButtonType.LB, new ScoreGamePiece()); // Score Game Piece
                copilot.whileTrue(ButtonType.RB, new IntakeAlgae()); // Intake Algae / Reverse Coral

                copilot.onTrue(ButtonType.A, new LiftWristGoToReefHeight(REEF_HEIGHT.BOTTOM)); // L1 / Processor
                copilot.onTrue(ButtonType.B, new LiftWristGoToReefHeight(REEF_HEIGHT.L2)); // L2 / Algae Removal 1
                copilot.onTrue(ButtonType.X, new LiftWristGoToReefHeight(REEF_HEIGHT.L3)); // L2 / Algae Removal 2
                copilot.onTrue(ButtonType.Y, new LiftWristGoToReefHeight(REEF_HEIGHT.TOP)); // L4 / NET

                copilot.onTrue(ButtonType.LT, new LiftWristGoToReefHeight(REEF_HEIGHT.BOTTOM)); // Intake Position
                copilot.whileTrue(ButtonType.LT, new IntakeUntilCoral()); // Intake until Coral

                copilot.onTrue(ButtonType.RT, new LiftWristIntakeAlgae()); // L1 / Processor
                copilot.whileTrue(ButtonType.RT, new IntakeAlgae());

                copilot.onTrue(ButtonType.BACK, new SetCoralMode(false)); // Algae Mode
                copilot.onTrue(ButtonType.START, new SetCoralMode(true)); // Coral Mode

                copilot.whileTrue(ButtonType.POV_N, new SetLEDColor()
                );
                break;
            default:

                copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);

                copilot.whileTrue(ButtonType.LB, new ScoreGamePiece()); // saca alga y mete embudo al REEF
                copilot.whileTrue(ButtonType.RB, new IntakeAlgae()); // agarra alga, regresa coral al embudo en zero pos

                // copilot.whileTrue(ButtonType.Y, new LiftFeedForward(0.0, 0.0));

                copilot.onTrue(ButtonType.A, new WristMantainTarget(29.5, HighAltitudeConstants.WRIST_DRIVE_SPEED)); // Intake
                                                                                                                     // Wrist
                copilot.whileTrue(ButtonType.A, new LiftSetMetersTarget(0.001)); // Cero

                copilot.onTrue(ButtonType.X, new WristMantainTarget(60, HighAltitudeConstants.WRIST_DRIVE_SPEED)); // 60
                                                                                                                   // para
                                                                                                                   // L4
                                                                                                                   // dunk
                copilot.onTrue(ButtonType.B, new WristMantainTarget(160, HighAltitudeConstants.WRIST_DRIVE_SPEED)); // 160
                                                                                                                    // para
                                                                                                                    // Alga

                copilot.whileTrue(ButtonType.POV_W, new LiftSetMetersTarget(0.77)); // L4
                copilot.whileTrue(ButtonType.POV_S, new LiftSetMetersTarget(0.10)); // L2
                copilot.whileTrue(ButtonType.POV_N, new LiftSetMetersTarget(0.35)); // L3
                copilot.whileTrue(ButtonType.POV_E, new LiftSetMetersTarget(0.5)); // Alga Arriba disque
                break;
        }

    }

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    public double getDefaultSwerveDriveSpeed() {

        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case DefaultUser:
                return pilot.getAxis(AxisType.LEFT_Y);

            case Joakin:
                return pilot.getAxis(AxisType.LEFT_Y);

            default:
                return pilot.getAxis(AxisType.LEFT_Y);

        }
    }

    public double getDefaultSwerveDriveStrafe() {

        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case DefaultUser:
                return pilot.getAxis(AxisType.LEFT_X);

            case Joakin:
                return pilot.getAxis(AxisType.LEFT_X);

            default:
                return pilot.getAxis(AxisType.LEFT_X);
        }
    }

    public double getDefaultSwerveDriveTurn() {

        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case DefaultUser:
                return pilot.getAxis(AxisType.RIGHT_X);

            case Joakin:
                return pilot.getAxis(AxisType.RIGHT_X);

            default:
                return pilot.getAxis(AxisType.RIGHT_X);
        }
    }

    public HighAltitudeJoystick getPilot() {
        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case DefaultUser:
                return pilot;

            case Joakin:
                return pilot;

            default:
                return pilot;
        }
    }

    public HighAltitudeJoystick getCopilot() {
        switch (HighAltitudeConstants.CURRENT_COPILOT) {

            case DefaultUser:
                return copilot;

            case Joakin:
                return copilot;

            default:
                return copilot;
        }
    }
}
