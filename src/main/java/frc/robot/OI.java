// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.HighAltitudeConstants.REEF_SIDE;
import frc.robot.commands.cancel.ResetLiftEncoders;
import frc.robot.commands.compound.LiftWristGoToTargetHeight;
import frc.robot.commands.compound.CoralOrAlgaeLiftDown;
import frc.robot.commands.compound.notbeingused.CoralModeLiftWrist;
import frc.robot.commands.extensor.climber.ClimberExtend;
import frc.robot.commands.extensor.climber.ClimberFold;
import frc.robot.commands.extensor.gripper.IntakeAuto;
import frc.robot.commands.extensor.gripper.IntakeUntilCurrentCoral;
import frc.robot.commands.extensor.gripper.manual.IntakeAlgae;
import frc.robot.commands.extensor.gripper.manual.ScoreGamePiece;
import frc.robot.commands.extensor.lift.manual.LiftDown;
import frc.robot.commands.extensor.lift.manual.LiftDownControl;
import frc.robot.commands.extensor.lift.manual.LiftUp;
import frc.robot.commands.extensor.lift.manual.LiftUpControl;
import frc.robot.commands.extensor.wrist.control.WristDefaultCommand;
import frc.robot.commands.extensor.wrist.control.WristGoToTarget;
import frc.robot.commands.extensor.wrist.manual.WristDownControl;
import frc.robot.commands.extensor.wrist.manual.WristUpControl;
import frc.robot.commands.modes.SetCoralMode;
import frc.robot.commands.modes.SetFrontMode;
import frc.robot.commands.modes.SetLeftMode;
import frc.robot.commands.modes.SetReefSideMode;
import frc.robot.commands.modes.WhileHeldPrecisionMode;
import frc.robot.commands.swerve.autonomous.feeder.DriveToCoralStation;
import frc.robot.commands.swerve.autonomous.reef.AlignWithTargetPose;
import frc.robot.commands.swerve.autonomous.reef.PathplanToReefThenVisionPose;
import frc.robot.commands.swerve.swerveParameters.ResetOdometryZeros;
import frc.robot.commands.swerve.swerveParameters.SetIsFieldOriented;
import frc.robot.commands.swerve.test.TestAlignWithPose;
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

                pilot.whileTrue(ButtonType.LS, new ClimberExtend());
                pilot.whileTrue(ButtonType.RS, new ClimberFold());

                pilot.whileTrue(ButtonType.POV_E, new SetLeftMode(false));
                pilot.whileTrue(ButtonType.POV_E,
                        new PathplanToReefThenVisionPose(null, null, false, HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                                HighAltitudeConstants.VISION_POSE_MAX_TURN));

                pilot.whileTrue(ButtonType.POV_W, new SetLeftMode(true));
                pilot.whileTrue(ButtonType.POV_W,
                        new PathplanToReefThenVisionPose(null, null, true, HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                                HighAltitudeConstants.VISION_POSE_MAX_TURN));

                pilot.whileTrue(ButtonType.POV_N, new SetFrontMode(true));
                pilot.whileTrue(ButtonType.POV_S, new SetFrontMode(false));

                pilot.whileTrue(ButtonType.X, new SetReefSideMode(REEF_SIDE.LEFT));
                pilot.whileTrue(ButtonType.A, new SetReefSideMode(REEF_SIDE.CENTER));
                pilot.whileTrue(ButtonType.B, new SetReefSideMode(REEF_SIDE.RIGHT));

                pilot.whileTrue(ButtonType.Y, new WhileHeldPrecisionMode());

                pilot.whileTrue(ButtonType.LB,
                        new AlignWithTargetPose(null, null, true, HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                                HighAltitudeConstants.VISION_POSE_MAX_TURN));

                pilot.whileTrue(ButtonType.RB,
                        new AlignWithTargetPose(null, null, false, HighAltitudeConstants.VISION_POSE_MAX_SPEED,
                                HighAltitudeConstants.VISION_POSE_MAX_TURN));

                pilot.onTrue(ButtonType.LT, new CoralOrAlgaeLiftDown());
                pilot.whileTrue(ButtonType.RT, new DriveToCoralStation(null, null,
                        HighAltitudeConstants.VISION_POSE_MAX_SPEED, HighAltitudeConstants.VISION_POSE_MAX_TURN));

                break;
            case JoakinButChambing:

                pilot = new HighAltitudeJoystick(0, JoystickType.XBOX);

                pilot.setAxisDeadzone(AxisType.LEFT_X, 0.1);
                pilot.setAxisDeadzone(AxisType.LEFT_Y, 0.1);
                pilot.setAxisDeadzone(AxisType.RIGHT_X, 0.1);

                pilot.whileTrue(ButtonType.Y, new WhileHeldPrecisionMode());

                pilot.onTrue(ButtonType.BACK, new SetIsFieldOriented(true));
                pilot.onTrue(ButtonType.START, new SetIsFieldOriented(false));
                pilot.onTrueCombo(new ResetOdometryZeros(), ButtonType.START, ButtonType.BACK);

                // pilot.whileTrue(ButtonType.Y, new TestDirectionPIDSwerve());
                // pilot.whileTrue(ButtonType.LB, new TestDrivePIDFFSwerve(1));
                // pilot.whileTrue(ButtonType.RB, new TestDrivePIDFFSwerve(-1));
                // pilot.whileTrueCombo(new PathCancelCommand(), ButtonType.RB, ButtonType.LB);

                pilot.whileTrue(ButtonType.LB, new IntakeAuto());

                pilot.whileTrue(ButtonType.RB, new TestAlignWithPose());
                /*
                 * pilot.whileTrue(ButtonType.POV_E,
                 * Robot.getRobotContainer().getSwerveDriveTrain().driveSysIdQuasistatic(
                 * Direction.kForward));
                 * pilot.whileTrue(ButtonType.POV_W,
                 * Robot.getRobotContainer().getSwerveDriveTrain().driveSysIdQuasistatic(
                 * Direction.kReverse));
                 * 
                 * pilot.whileTrue(ButtonType.POV_N,
                 * Robot.getRobotContainer().getSwerveDriveTrain().driveSysIdDynamic(Direction.
                 * kForward));
                 * pilot.whileTrue(ButtonType.POV_S,
                 * Robot.getRobotContainer().getSwerveDriveTrain().driveSysIdDynamic(Direction.
                 * kReverse));
                 */
                /*
                 * pilot.whileTrue(ButtonType.POV_N,
                 * Robot.getRobotContainer().getLift().sysIdQuasistatic(Direction.kForward));
                 * pilot.whileTrue(ButtonType.POV_S,
                 * Robot.getRobotContainer().getLift().sysIdQuasistatic(Direction.kReverse));
                 * 
                 * pilot.whileTrue(ButtonType.POV_E,
                 * Robot.getRobotContainer().getLift().sysIdDynamic(Direction.kForward));
                 * pilot.whileTrue(ButtonType.POV_W,
                 * Robot.getRobotContainer().getLift().sysIdDynamic(Direction.kReverse));
                 */

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

                copilot.onTrue(ButtonType.A, new LiftWristGoToTargetHeight(REEF_HEIGHT.L2));

                copilot.onTrue(ButtonType.X, new LiftWristGoToTargetHeight(REEF_HEIGHT.L3));

                copilot.onTrue(ButtonType.Y, new LiftWristGoToTargetHeight(REEF_HEIGHT.TOP));

                copilot.onTrue(ButtonType.BACK, new SetCoralMode(false)); // Algae Mode
                copilot.onTrue(ButtonType.START, new SetCoralMode(true)); // Coral Mode

                copilot.onTrue(ButtonType.LT, new LiftWristGoToTargetHeight(REEF_HEIGHT.BOTTOM)); // Intake Position
                copilot.onTrue(ButtonType.LT, new IntakeUntilCurrentCoral());

                copilot.whileTrue(ButtonType.POV_N, new LiftUpControl());
                copilot.whileTrue(ButtonType.POV_S, new LiftDownControl());

                copilot.whileTrue(ButtonType.POV_E, new WristUpControl());
                copilot.whileTrue(ButtonType.POV_W, new WristDownControl());

                copilot.whileTrue(ButtonType.LS, new ScoreGamePiece(-0.1));
                copilot.whileTrue(ButtonType.RS, new ResetLiftEncoders());
                break;

            case ItaiAndGomezButChambingButCompetionButIsLeonButIsREEFSCAPE:

                copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);

                copilot.onTrue(ButtonType.BACK, new SetCoralMode(false)); // Algae Mode
                copilot.onTrue(ButtonType.START, new SetCoralMode(true)); // Coral Mode

                copilot.whileTrue(ButtonType.LB, new ScoreGamePiece(HighAltitudeConstants.GRIPPER_IN_SPEED)); // Score
                                                                                                              // Game
                                                                                                              // Piece
                copilot.whileTrue(ButtonType.RB, new IntakeAlgae()); // Intake Algae / Reverse Coral

                copilot.whileTrue(ButtonType.POV_N, new LiftUp());
                copilot.whileTrue(ButtonType.POV_S, new LiftDown());

                copilot.whileTrue(ButtonType.POV_E, new WristGoToTarget(0, 0.1));
                copilot.whileTrue(ButtonType.POV_W, new WristGoToTarget(45, 0.1));

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
                return -pilot.getAxis(AxisType.LEFT_Y);

            case Joakin:
                return -pilot.getAxis(AxisType.LEFT_Y);

            default:
                return -pilot.getAxis(AxisType.LEFT_Y);

        }
    }

    public double getDefaultSwerveDriveStrafe() {

        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case DefaultUser:
                return -pilot.getAxis(AxisType.LEFT_X);

            case Joakin:
                return -pilot.getAxis(AxisType.LEFT_X);

            default:
                return -pilot.getAxis(AxisType.LEFT_X);
        }
    }

    public double getDefaultSwerveDriveTurn() {

        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case DefaultUser:
                return -pilot.getAxis(AxisType.RIGHT_X);

            case Joakin:
                return -pilot.getAxis(AxisType.RIGHT_X);

            default:
                return -pilot.getAxis(AxisType.RIGHT_X);
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
