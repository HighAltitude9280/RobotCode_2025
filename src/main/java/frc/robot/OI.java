// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

                pilot.setAxisDeadzone(AxisType.LEFT_X, 0.2);
                pilot.setAxisDeadzone(AxisType.LEFT_Y, 0.2);
                pilot.setAxisDeadzone(AxisType.RIGHT_X, 0.2);

                pilot.onTrue(ButtonType.BACK, new SetIsFieldOriented(true));
                pilot.onTrue(ButtonType.START, new SetIsFieldOriented(false));
                pilot.onTrue(ButtonType.X, new ResetOdometryZeros());

                pilot.whileTrue(ButtonType.Y, new TestDirectionPIDSwerve());
                pilot.whileTrue(ButtonType.POV_N, new TestDrivePIDFFSwerve(0.5));
                pilot.whileTrue(ButtonType.POV_S, new TestDrivePIDFFSwerve(-0.5));  
                pilot.whileTrue(ButtonType.B, new TestSwerve());


            default:
                break;

        }
        switch (HighAltitudeConstants.CURRENT_COPILOT) {

            case DefaultUser:

                copilot = new HighAltitudeJoystick(1, JoystickType.XBOX);

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
