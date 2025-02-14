// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.gripper.manual.GripperIn;
import frc.robot.commands.gripper.manual.GripperOut;
import frc.robot.commands.lift.manual.LiftDown;
import frc.robot.commands.lift.manual.LiftUp;
import frc.robot.commands.swerve.swerveParameters.ResetOdometryZeros;
import frc.robot.commands.swerve.swerveParameters.SetIsFieldOriented;
import frc.robot.commands.swerve.test.TestDirectionPIDSwerve;
import frc.robot.commands.swerve.test.TestDrivePIDFFSwerve;
import frc.robot.commands.swerve.test.TestSwerve;
import frc.robot.commands.wrist.WristMantainTo;
import frc.robot.commands.wrist.manual.WristDown;
import frc.robot.commands.wrist.manual.WristUp;
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

                pilot.whileTrue(ButtonType.POV_N, new LiftUp());
                pilot.whileTrue(ButtonType.POV_S, new LiftDown());

                pilot.whileTrue(ButtonType.LB, new GripperIn());
                pilot.whileTrue(ButtonType.RB, new GripperOut());

                pilot.whileTrue(ButtonType.POV_E, new WristDown());
                pilot.whileTrue(ButtonType.POV_W, new WristUp());

                pilot.onTrue(ButtonType.A, new WristMantainTo(90, HighAltitudeConstants.WRIST_DRIVE_SPEED));

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

            case Joakin:

                copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);
                copilot.whileTrue(ButtonType.POV_N, new LiftUp());
                copilot.whileTrue(ButtonType.POV_S, new LiftDown());

                copilot.whileTrue(ButtonType.LB, new GripperIn());
                copilot.whileTrue(ButtonType.RB, new GripperOut());

                copilot.whileTrue(ButtonType.POV_E, new WristDown());
                copilot.whileTrue(ButtonType.POV_W, new WristUp());
                break;
            default:
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
                return pilot.getAxis(AxisType.LEFT_Y) * 0.88;

            default:
                return pilot.getAxis(AxisType.LEFT_Y);

        }
    }

    public double getDefaultSwerveDriveStrafe() {

        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case DefaultUser:
                return pilot.getAxis(AxisType.LEFT_X);

            case Joakin:
                return pilot.getAxis(AxisType.LEFT_X) * 0.88;

            default:
                return pilot.getAxis(AxisType.LEFT_X);
        }
    }

    public double getDefaultSwerveDriveTurn() {

        switch (HighAltitudeConstants.CURRENT_PILOT) {

            case DefaultUser:
                return pilot.getAxis(AxisType.RIGHT_X);

            case Joakin:
                return pilot.getAxis(AxisType.RIGHT_X) * 0.8;

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
