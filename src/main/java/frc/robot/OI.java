// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.HighAltitudeConstants.REEF_POSITION;
import frc.robot.commands.autonomous.AutoLeave;
import frc.robot.commands.cancel.PathCancelCommand;
import frc.robot.commands.cancel.SubsystemsCancelCommand;
import frc.robot.commands.compound.LiftWristGoToTargetHeight;
import frc.robot.commands.compound.CoralOrAlgaeLiftDown;
import frc.robot.commands.compound.KeepAlgaeSafe;
import frc.robot.commands.compound.notbeingused.CoralModeLiftWrist;
import frc.robot.commands.extensor.gripper.IntakeUntilCoral;
import frc.robot.commands.extensor.gripper.IntakeUntilCurrentCoral;
import frc.robot.commands.extensor.gripper.manual.IntakeAlgae;
import frc.robot.commands.extensor.gripper.manual.ScoreGamePiece;
import frc.robot.commands.extensor.lift.manual.LiftDownControl;
import frc.robot.commands.extensor.lift.manual.LiftUpControl;
import frc.robot.commands.extensor.wrist.manual.WristDownControl;
import frc.robot.commands.extensor.wrist.manual.WristUpControl;
import frc.robot.commands.leds.SetFlameModeHighAltitude;
import frc.robot.commands.leds.SetLEDOff;
import frc.robot.commands.modes.SetCoralMode;
import frc.robot.commands.modes.SetLeftMode;
import frc.robot.commands.modes.WhileHeldPrecisionMode;
import frc.robot.commands.swerve.DefaultSwerveDriveNew;
import frc.robot.commands.swerve.autonomous.AlignVisionMoveMeters;
import frc.robot.commands.swerve.autonomous.SwerveMoveMeters;
import frc.robot.commands.swerve.autonomous.reef.AlignWithTargetVision;
import frc.robot.commands.swerve.swerveParameters.ResetOdometryZeros;
import frc.robot.commands.swerve.swerveParameters.SetIsFieldOriented;
import frc.robot.commands.swerve.test.TestDirectionPIDSwerve;
import frc.robot.commands.swerve.test.TestDrivePIDFFSwerve;
import frc.robot.commands.swerve.test.TestSwerve;
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

                pilot.setAxisDeadzone(AxisType.LEFT_X, 0.1);
                pilot.setAxisDeadzone(AxisType.LEFT_Y, 0.1);
                pilot.setAxisDeadzone(AxisType.RIGHT_X, 0.1);

                pilot.onTrue(ButtonType.BACK, new SetIsFieldOriented(true));
                pilot.onTrue(ButtonType.START, new SetIsFieldOriented(false));
                pilot.onTrueCombo(new ResetOdometryZeros(), ButtonType.START, ButtonType.BACK);

                // pilot.whileTrueCombo(new AlignVisionMoveMeters(), ButtonType.RT,
                // ButtonType.LT);

                pilot.whileTrue(ButtonType.B, new SetLeftMode(false));
                pilot.whileTrue(ButtonType.X, new SetLeftMode(true));

                pilot.onTrueCombo(new DefaultSwerveDriveNew(), ButtonType.A, ButtonType.B);

                pilot.whileTrue(ButtonType.POV_E, new WhileHeldPrecisionMode());

                pilot.whileTrueCombo(new AlignVisionMoveMeters(null, null, null), ButtonType.LB, ButtonType.RB);

                pilot.whileTrue(ButtonType.A,
                        new AutoLeave(HighAltitudeConstants.SWERVE_METERS_DISTANCE_ALIGN_TO_REEF, 0.7));

                break;
            case JoakinButChambing:

                pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);

                pilot.setAxisDeadzone(AxisType.LEFT_X, 0.1);
                pilot.setAxisDeadzone(AxisType.LEFT_Y, 0.1);
                pilot.setAxisDeadzone(AxisType.RIGHT_X, 0.1);

                pilot.whileTrue(ButtonType.POV_E, new WhileHeldPrecisionMode());

                pilot.onTrue(ButtonType.BACK, new SetIsFieldOriented(true));
                pilot.onTrue(ButtonType.START, new SetIsFieldOriented(false));
                pilot.onTrue(ButtonType.X, new ResetOdometryZeros());

                // pilot.whileTrue(ButtonType.Y, new TestDirectionPIDSwerve());
                pilot.whileTrue(ButtonType.POV_N, new TestDrivePIDFFSwerve(4));
                pilot.whileTrue(ButtonType.POV_S, new TestDrivePIDFFSwerve(-4));

                // pilot.whileTrueCombo(new PathCancelCommand(), ButtonType.RB, ButtonType.LB);

                // pilot.whileTrue(ButtonType.B, new TestSwerve());
                break;
            default:
                break;

        }
        switch (HighAltitudeConstants.CURRENT_COPILOT) {

            case Carlos:

                copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);

                copilot.whileTrue(ButtonType.LB, new ScoreGamePiece(HighAltitudeConstants.GRIPPER_IN_SPEED)); // Score
                                                                                                              // Game
                                                                                                              // Piece
                copilot.onTrue(ButtonType.RT, new IntakeAlgae()); // Intake Algae / Reverse Coral

                copilot.whileTrue(ButtonType.RB, new IntakeAlgae());

                copilot.onTrue(ButtonType.B, new LiftWristGoToTargetHeight(REEF_HEIGHT.BOTTOM));
                copilot.onFalse(ButtonType.B, new CoralOrAlgaeLiftDown());

                copilot.onTrue(ButtonType.A, new LiftWristGoToTargetHeight(REEF_HEIGHT.L2));
                copilot.onFalse(ButtonType.A, new CoralOrAlgaeLiftDown());

                copilot.onTrue(ButtonType.X, new LiftWristGoToTargetHeight(REEF_HEIGHT.L3));
                copilot.onFalse(ButtonType.X, new CoralOrAlgaeLiftDown());

                copilot.onTrue(ButtonType.Y, new LiftWristGoToTargetHeight(REEF_HEIGHT.TOP));
                copilot.onFalse(ButtonType.Y, new CoralOrAlgaeLiftDown());

                copilot.onTrue(ButtonType.BACK, new SetCoralMode(false)); // Algae Mode
                copilot.onTrue(ButtonType.START, new SetCoralMode(true)); // Coral Mode

                copilot.onTrue(ButtonType.LT, new LiftWristGoToTargetHeight(REEF_HEIGHT.BOTTOM)); // Intake Position
                copilot.onTrue(ButtonType.LT, new IntakeUntilCurrentCoral());

                copilot.whileTrue(ButtonType.POV_N, new LiftUpControl());
                copilot.whileTrue(ButtonType.POV_S, new LiftDownControl());

                copilot.whileTrue(ButtonType.POV_E, new WristUpControl());
                copilot.whileTrue(ButtonType.POV_W, new WristDownControl());

                /*
                 * copilot.onTrue(ButtonType.A, new
                 * LiftWristGoToReefHeight(REEF_HEIGHT.BOTTOM)); // L1 / Processor
                 * copilot.onTrue(ButtonType.B, new LiftWristGoToReefHeight(REEF_HEIGHT.L2)); //
                 * L2 / Algae Removal 1
                 * copilot.onTrue(ButtonType.X, new LiftWristGoToReefHeight(REEF_HEIGHT.L3)); //
                 * L2 / Algae Removal 2
                 * copilot.onTrue(ButtonType.Y, new LiftWristGoToReefHeight(REEF_HEIGHT.TOP));//
                 * L4 / NET
                 * copilot.whileTrue(ButtonType.POV_S, new SetFlameModeHighAltitude());
                 * copilot.whileTrue(ButtonType.POV_N, new SetLEDOff());
                 */

                // copilot.whileTrue(ButtonType.POV_E, new WristDown());
                // copilot.whileTrue(ButtonType.POV_W, new WristUp());

                /*
                 * copilot.onTrue(ButtonType.RT, new LiftWristIntakeAlgae()); // L1 / Processor
                 * copilot.whileTrue(ButtonType.RT, new IntakeAlgae());
                 */

                break;

            case ItaiAndGomezButChambingButCompetionButIsLeonButIsREEFSCAPE:

                copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);

                copilot.onTrue(ButtonType.BACK, new CoralModeLiftWrist(false)); // Algae Mode
                copilot.onTrue(ButtonType.START, new CoralModeLiftWrist(true)); // Coral Mode

                copilot.whileTrue(ButtonType.LB, new ScoreGamePiece(HighAltitudeConstants.GRIPPER_IN_SPEED)); // Score
                                                                                                              // Game
                                                                                                              // Piece
                copilot.whileTrue(ButtonType.RB, new IntakeAlgae()); // Intake Algae / Reverse Coral

                break;
            default:
                /*
                 * copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);
                 * 
                 * copilot.whileTrue(ButtonType.LB, new ScoreGamePiece()); // saca alga y mete
                 * embudo al REEF
                 * copilot.whileTrue(ButtonType.RB, new IntakeAlgae()); // agarra alga, regresa
                 * coral al embudo en zero pos
                 * 
                 * // copilot.whileTrue(ButtonType.Y, new LiftFeedForward(0.0, 0.0));
                 * 
                 * copilot.onTrue(ButtonType.A, new WristMantainTarget(29.5,
                 * HighAltitudeConstants.WRIST_DRIVE_SPEED)); // Intake
                 * // Wrist
                 * copilot.whileTrue(ButtonType.A, new LiftSetMetersTarget(0.001)); // Cero
                 * 
                 * copilot.onTrue(ButtonType.X, new WristMantainTarget(60,
                 * HighAltitudeConstants.WRIST_DRIVE_SPEED)); // 60
                 * // para
                 * // L4
                 * // dunk
                 * copilot.onTrue(ButtonType.B, new WristMantainTarget(160,
                 * HighAltitudeConstants.WRIST_DRIVE_SPEED)); // 160
                 * // para
                 * // Alga
                 * 
                 * copilot.whileTrue(ButtonType.POV_W, new LiftSetMetersTarget(0.77)); // L4
                 * copilot.whileTrue(ButtonType.POV_S, new LiftSetMetersTarget(0.10)); // L2
                 * copilot.whileTrue(ButtonType.POV_N, new LiftSetMetersTarget(0.35)); // L3
                 * copilot.whileTrue(ButtonType.POV_E, new LiftSetMetersTarget(0.5)); // Alga
                 * Arriba disque
                 */
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
