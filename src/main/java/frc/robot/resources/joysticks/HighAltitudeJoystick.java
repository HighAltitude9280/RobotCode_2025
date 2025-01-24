// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources.joysticks;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.resources.math.Math;

/** Add your docs here. */
public class HighAltitudeJoystick {

    private Joystick joystick;
    private JoystickType joystickType;
    private Haptics haptics;

    public enum JoystickType {
        PS4,
        XBOX,
        UNKNOWN
    }

    public enum ButtonType {
        A,
        B,
        X,
        Y,
        LB,
        RB,
        BACK,
        START,
        LS,
        RS,
        PS,
        TOUCHPAD,
        POV_N,
        POV_NE,
        POV_E,
        POV_SE,
        POV_S,
        POV_SW,
        POV_W,
        POV_NW,
        POV_NULL,
        LT,
        RT,
        JOYSTICK_L_X,
        JOYSTICK_L_Y,
        JOYSTICK_R_X,
        JOYSTICK_R_Y
    }

    public enum AxisType {
        LEFT_X,
        LEFT_Y,
        RIGHT_X,
        RIGHT_Y,
        LEFT_TRIGGER,
        RIGHT_TRIGGER,
        POV_X,
        POV_Y
    }

    private HashMap<Integer, JoystickButton> availableJoystickButtons;
    private HashMap<Integer, POVButton> availablePOVButtons;
    private HashMap<Integer, Trigger> availableAxisButtons;

    private HashMap<ButtonType, Trigger> joystickButtonConfiguration;

    private HashMap<AxisType, Integer> axisConfiguration;
    private HashMap<AxisType, Double> axisDeadzoneConfiguration;
    private HashMap<AxisType, Double> axisMultiplierConfiguration;

    /**
     * 
     * Creates a new {@link HighAltitudeJoystick}, which can be either XBOX or PS4.
     * This Joystick has associated buttons.
     * 
     * If the joystick type is UNKOWN, use
     * {@link #HighAltitudeJoystick(int, int, int)} as constructor preferrably.
     * Otherwise, the default button count will be 14 and the default axis count
     * will be 6. If these values are not true, you risk nullPointerExceptions and
     * DS warning overloads.
     * 
     * Though there's some support for an
     * UNKNOWN {@link JoystickType}, the specific case will have to be handled
     * manually by the programmer using {@link #getJoystickButtonObj(int)} and
     * {@link #getRawAxis(int)}.
     * 
     * 
     * @param port The port of the controller.
     * @param type The {@link JoystickType} of the controller.
     */

    public HighAltitudeJoystick(int port, JoystickType type) {
        this.joystick = new Joystick(port);
        this.joystickType = type;

        initializeHashMaps();

        switch (type) {
            case XBOX:
                configureXboxJoystick();
                break;
            case PS4:
                configurePs4Joystick();
                break;
            case UNKNOWN:
            default:
                configureUnknownJoystick(14, 6);
                break;
        }
        configureDefaultDeadzoneAndMultiplier(0, 1);
        haptics = new Haptics(joystick);
    }

    /**
     * 
     * Creates a new UNKNOWN{@link HighAltitudeJoystick}.
     * This Joystick has associated buttons and axes, which you specify in the
     * constructor's second and third parameter respectively.
     * Use {@link #getJoystickButtonObj(int)} and {@link #getRawAxis(int)} to access
     * their data. Use accurate button and axes counts to prevent DS Warning
     * overloads.
     * 
     * 
     * @param port        The port of the controller.
     * @param type        The {@link JoystickType} of the controller.
     * @param buttonCount The amount of buttons 1...n that this joystick has.
     * @param axisCount   The amount of axes 1...n that this joystick has.
     */

    public HighAltitudeJoystick(int port, int buttonCount, int axisCount) {
        this.joystick = new Joystick(port);
        this.joystickType = JoystickType.UNKNOWN;

        initializeHashMaps();

        configureUnknownJoystick(buttonCount, axisCount);

        configureDefaultDeadzoneAndMultiplier(0, 1);
        haptics = new Haptics(joystick);
    }

    private void initializeHashMaps() {
        availableJoystickButtons = new HashMap<Integer, JoystickButton>();
        availablePOVButtons = new HashMap<Integer, POVButton>();
        availableAxisButtons = new HashMap<Integer, Trigger>();
        axisConfiguration = new HashMap<AxisType, Integer>();
        joystickButtonConfiguration = new HashMap<ButtonType, Trigger>();
    }

    private void configureXboxJoystick() {

        for (int i = 1; i <= 10; i++) {
            availableJoystickButtons.put(i, new JoystickButton(joystick, i));
        }

        availablePOVButtons.put(-1, new POVButton(joystick, -1));
        for (int i = 0; i <= 360; i += 45) {
            availablePOVButtons.put(i, new POVButton(joystick, i));
        }

        // MAPPING STARTS HERE

        axisConfiguration.put(AxisType.LEFT_X, 0);
        axisConfiguration.put(AxisType.LEFT_Y, 1);
        axisConfiguration.put(AxisType.LEFT_TRIGGER, 2);
        axisConfiguration.put(AxisType.RIGHT_TRIGGER, 3);
        axisConfiguration.put(AxisType.RIGHT_X, 4);
        axisConfiguration.put(AxisType.RIGHT_Y, 5);

        joystickButtonConfiguration.put(ButtonType.A, availableJoystickButtons.get(1));
        joystickButtonConfiguration.put(ButtonType.B, availableJoystickButtons.get(2));
        joystickButtonConfiguration.put(ButtonType.X, availableJoystickButtons.get(3));
        joystickButtonConfiguration.put(ButtonType.Y, availableJoystickButtons.get(4));
        joystickButtonConfiguration.put(ButtonType.LB, availableJoystickButtons.get(5));
        joystickButtonConfiguration.put(ButtonType.RB, availableJoystickButtons.get(6));
        joystickButtonConfiguration.put(ButtonType.BACK, availableJoystickButtons.get(7));
        joystickButtonConfiguration.put(ButtonType.START, availableJoystickButtons.get(8));
        joystickButtonConfiguration.put(ButtonType.LS, availableJoystickButtons.get(9));
        joystickButtonConfiguration.put(ButtonType.RS, availableJoystickButtons.get(10));

        joystickButtonConfiguration.put(ButtonType.POV_NULL, availablePOVButtons.get(-1));
        joystickButtonConfiguration.put(ButtonType.POV_N, availablePOVButtons.get(0));
        joystickButtonConfiguration.put(ButtonType.POV_NE, availablePOVButtons.get(45));
        joystickButtonConfiguration.put(ButtonType.POV_E, availablePOVButtons.get(90));
        joystickButtonConfiguration.put(ButtonType.POV_SE, availablePOVButtons.get(135));
        joystickButtonConfiguration.put(ButtonType.POV_S, availablePOVButtons.get(180));
        joystickButtonConfiguration.put(ButtonType.POV_SW, availablePOVButtons.get(225));
        joystickButtonConfiguration.put(ButtonType.POV_W, availablePOVButtons.get(270));
        joystickButtonConfiguration.put(ButtonType.POV_NW, availablePOVButtons.get(315));
        // MAPPING ENDS HERE

        for (AxisType axisType : AxisType.values()) {
            if (axisConfiguration.get(axisType) != null) {
                int port = axisConfiguration.get(axisType);
                BooleanSupplier booleanSupplier = () -> isAxisPressed(axisType);
                availableAxisButtons.put(port, new Trigger(booleanSupplier));
            }
        }

        joystickButtonConfiguration.put(ButtonType.LT,
                availableAxisButtons.get(axisConfiguration.get(AxisType.LEFT_TRIGGER)));
        joystickButtonConfiguration.put(ButtonType.RT,
                availableAxisButtons.get(axisConfiguration.get(AxisType.RIGHT_TRIGGER)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_L_X,
                availableAxisButtons.get(axisConfiguration.get(AxisType.LEFT_X)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_L_Y,
                availableAxisButtons.get(axisConfiguration.get(AxisType.LEFT_Y)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_R_X,
                availableAxisButtons.get(axisConfiguration.get(AxisType.RIGHT_X)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_R_Y,
                availableAxisButtons.get(axisConfiguration.get(AxisType.RIGHT_Y)));

    }

    private void configurePs4Joystick() {
        for (int i = 1; i <= 14; i++) {
            availableJoystickButtons.put(i, new JoystickButton(joystick, i));
        }

        availablePOVButtons.put(-1, new POVButton(joystick, -1));
        for (int i = 0; i <= 360; i += 45) {
            availablePOVButtons.put(i, new POVButton(joystick, i));
        }

        // MAPPING STARTS HERE
        axisConfiguration.put(AxisType.LEFT_X, 0);
        axisConfiguration.put(AxisType.LEFT_Y, 1);
        axisConfiguration.put(AxisType.LEFT_TRIGGER, 3);
        axisConfiguration.put(AxisType.RIGHT_TRIGGER, 4);
        axisConfiguration.put(AxisType.RIGHT_X, 2);
        axisConfiguration.put(AxisType.RIGHT_Y, 5);

        joystickButtonConfiguration.put(ButtonType.A, availableJoystickButtons.get(2));
        joystickButtonConfiguration.put(ButtonType.B, availableJoystickButtons.get(3));
        joystickButtonConfiguration.put(ButtonType.X, availableJoystickButtons.get(1));
        joystickButtonConfiguration.put(ButtonType.Y, availableJoystickButtons.get(4));
        joystickButtonConfiguration.put(ButtonType.LB, availableJoystickButtons.get(5));
        joystickButtonConfiguration.put(ButtonType.RB, availableJoystickButtons.get(6));
        joystickButtonConfiguration.put(ButtonType.BACK, availableJoystickButtons.get(9));
        joystickButtonConfiguration.put(ButtonType.START, availableJoystickButtons.get(10));
        joystickButtonConfiguration.put(ButtonType.LS, availableJoystickButtons.get(11));
        joystickButtonConfiguration.put(ButtonType.RS, availableJoystickButtons.get(12));
        joystickButtonConfiguration.put(ButtonType.PS, availableJoystickButtons.get(13));
        joystickButtonConfiguration.put(ButtonType.TOUCHPAD, availableJoystickButtons.get(14));

        joystickButtonConfiguration.put(ButtonType.POV_NULL, availablePOVButtons.get(-1));
        joystickButtonConfiguration.put(ButtonType.POV_N, availablePOVButtons.get(0));
        joystickButtonConfiguration.put(ButtonType.POV_NE, availablePOVButtons.get(45));
        joystickButtonConfiguration.put(ButtonType.POV_E, availablePOVButtons.get(90));
        joystickButtonConfiguration.put(ButtonType.POV_SE, availablePOVButtons.get(135));
        joystickButtonConfiguration.put(ButtonType.POV_S, availablePOVButtons.get(180));
        joystickButtonConfiguration.put(ButtonType.POV_SW, availablePOVButtons.get(225));
        joystickButtonConfiguration.put(ButtonType.POV_W, availablePOVButtons.get(270));
        joystickButtonConfiguration.put(ButtonType.POV_NW, availablePOVButtons.get(315));
        // MAPPING ENDS HERE

        for (AxisType axisType : AxisType.values()) {
            if (axisConfiguration.get(axisType) != null) {
                int port = axisConfiguration.get(axisType);
                BooleanSupplier booleanSupplier = () -> isAxisPressed(axisType);
                availableAxisButtons.put(port, new Trigger(booleanSupplier));
            }
        }

        joystickButtonConfiguration.put(ButtonType.LT,
                availableAxisButtons.get(axisConfiguration.get(AxisType.LEFT_TRIGGER)));
        joystickButtonConfiguration.put(ButtonType.RT,
                availableAxisButtons.get(axisConfiguration.get(AxisType.RIGHT_TRIGGER)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_L_X,
                availableAxisButtons.get(axisConfiguration.get(AxisType.LEFT_X)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_L_Y,
                availableAxisButtons.get(axisConfiguration.get(AxisType.LEFT_Y)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_R_X,
                availableAxisButtons.get(axisConfiguration.get(AxisType.RIGHT_X)));
        joystickButtonConfiguration.put(ButtonType.JOYSTICK_R_Y,
                availableAxisButtons.get(axisConfiguration.get(AxisType.RIGHT_Y)));
    }

    private void configureUnknownJoystick(int buttonCount, int axisCount) {
        for (int i = 1; i <= buttonCount; i++) {
            availableJoystickButtons.put(i, new JoystickButton(joystick, i));
        }

        for (int i = 0; i <= 360; i += 45) {
            availablePOVButtons.put(i, new POVButton(joystick, i));
        }

        for (int i = 0; i < axisCount; i++) {
            int currentPort = i;
            BooleanSupplier booleanSupplier = () -> isAxisPressed(currentPort);
            availableAxisButtons.put(i, new Trigger(booleanSupplier));
        }

    }

    private void configureDefaultDeadzoneAndMultiplier(double deadzone, double multiplier) {
        axisDeadzoneConfiguration = new HashMap<AxisType, Double>();
        axisMultiplierConfiguration = new HashMap<AxisType, Double>();
        for (AxisType axisType : AxisType.values()) {
            axisDeadzoneConfiguration.put(axisType, deadzone);
            axisMultiplierConfiguration.put(axisType, multiplier);
        }
    }

    /**
     * Will return the RAW value of the given {@link AxisType}. The mapping that
     * relates {@link AxisType} with the corresponding port is
     * {@link axisConfiguration}. PS4 triggers will return a value from -1 to 1
     * instead of returning from 0-1.
     * 
     * @param axisType The desired axis
     * @return Raw axis value
     */
    public double getRawAxis(AxisType axisType) {
        if (axisType.equals(AxisType.POV_X))
            return getPovXAxis();
        if (axisType.equals(AxisType.POV_Y))
            return getPovYAxis();
        try {
            return joystick.getRawAxis(axisConfiguration.get(axisType));
        } catch (NullPointerException e) {
            DriverStation.reportError("Axis " + axisType + " not found. Returning 0.", true);
        }
        return 0;
    }

    /**
     * Will return the RAW value of the given axis port. PS4 triggers will return a
     * value from -1 to 1 instead of returning from 0-1.
     * 
     * @param axisType The port of the desired axis
     * @return Raw axis value
     */
    public double getRawAxis(int axisPort) {
        return joystick.getRawAxis(axisPort);
    }

    /**
     * Will return the PROCESSED value of the chosen axis, applying both the set
     * <b>deadzone</b> and <b>multiplier</b>. PS4 Triggers are given in standard 0
     * to 1 instead of -1 to 1. Use {@link #setAxisDeadzone(AxisType, double)} and
     * {@link #setAxisMultiplier(AxisType, double)} to modify these values.
     * 
     * @param axis The desired axis
     * @return Processed axis value
     */
    public double getAxis(AxisType axis) {
        double input = getRawAxis(axis);
        if (joystickType == JoystickType.PS4
                && (axis.equals(AxisType.LEFT_TRIGGER) || axis.equals(AxisType.RIGHT_TRIGGER))) {
            input = (input + 1) / 2;
        }
        double deadzonedInput = Math.applyDeadzone(input, axisDeadzoneConfiguration.get(axis));
        double multipliedInput = deadzonedInput * axisMultiplierConfiguration.get(axis);
        return multipliedInput;
    }

    /**
     * Will return the value of the joystick's Left and Right Triggers combined. For
     * instance, if the left one is pressed all the way in, -1 will be returned. If
     * the
     * right trigger is pressed all the way in, +1 will returned. If both
     * are pressed, 0 will be returned.
     * 
     * @return Processed axis value
     */
    public double getTriggers() {
        double left = getAxis(AxisType.LEFT_TRIGGER);
        double right = getAxis(AxisType.RIGHT_TRIGGER);

        return right - left;
    }

    /**
     * Treats the POV axis as if it were another axis.
     * 
     * @return the 'raw' x-value of the POV. Use {@link #getAxis()} to obtain a
     *         value with deadzone/multiplier applied.
     */

    public double getPovXAxis() {
        double x = Math.sin(Math.toRadians(joystick.getPOV()));
        return joystick.getPOV() == -1 || Math.abs(x) < 0.1 ? 0 : x;
    }

    /**
     * Treats the POV axis as if it were another axis.
     * 
     * @return the 'raw' y-value of the POV. Use {@link #getAxis()} to obtain a
     *         value with deadzone/multiplier applied.
     */

    public double getPovYAxis() {
        double y = Math.cos(Math.toRadians(joystick.getPOV()));
        return joystick.getPOV() == -1 || Math.abs(y) < 0.1 ? 0 : y;
    }

    public boolean isAxisPressed(AxisType axisType) {
        return Math.abs(getAxis(axisType)) > 0.5;
    }

    public boolean isAxisPressed(int axisPort) {
        return Math.abs(getRawAxis(axisPort)) > 0.5;
    }

    public void setAxisDeadzone(AxisType axis, double deadzone) {
        axisDeadzoneConfiguration.put(axis, Math.abs(deadzone));
    }

    public void setAxisMultiplier(AxisType axis, double multiplier) {
        axisMultiplierConfiguration.put(axis, multiplier);
    }

    public Trigger getButtonObj(ButtonType buttonType) {
        return joystickButtonConfiguration.get(buttonType);
    }

    public JoystickButton getJoystickButtonObj(int port) {
        return availableJoystickButtons.get(port);
    }

    public POVButton getPOVButtonObj(int angle) {
        return availablePOVButtons.get(angle);
    }

    public Trigger getAxisButtonObj(int axis) {
        return availableAxisButtons.get(axis);
    }

    public Haptics getHaptics() {
        return haptics;
    }

    public Joystick getJoystick() {
        return joystick;
    }

    // METHODS FOR ASSOCIATING COMMANDS YES THEY'RE A LOT BUT WE'D RATHER HAVE IT
    // THIS WAY

    /**
     * Starts the given command whenever the button changes from 'unpressed' to
     * 'pressed'.
     * Won't cancel the command.
     * 
     * @param buttonType the button which will trigger the command
     * @param command    command to be assigned to button
     */
    public void onTrue(ButtonType buttonType, Command command) {
        try {
            Trigger chosenButton = joystickButtonConfiguration.get(buttonType);
            chosenButton.onTrue(command);
        } catch (NullPointerException e) {
            reportButtonError(buttonType, command);
        }
    }

    /**
     * Starts the given command whenever the condition changes from 'unpressed' to
     * 'pressed'.
     * Cancels the given command whenever the condition changes from 'pressed' to
     * 'unpressed'.
     *
     * @param povE the button which will trigger the command
     * @param command    command to be assigned to button
     */
    public void whileTrue(ButtonType povE, Command command) {
        try {
            Trigger chosenButton = joystickButtonConfiguration.get(povE);
            chosenButton.whileTrue(command);
        } catch (NullPointerException e) {
            reportButtonError(povE, command);
        }
    }

    /**
     * When the condition changes from 'unpressed' to 'pressed', starts the command
     * if it's not running
     * and cancels the command if it's already running.
     *
     * @param buttonType the button which will trigger the command
     * @param command    command to be assigned to button
     */
    public void toggleOnTrue(ButtonType buttonType, Command command) {
        try {
            Trigger chosenButton = joystickButtonConfiguration.get(buttonType);
            chosenButton.toggleOnTrue(command);
        } catch (NullPointerException e) {
            reportButtonError(buttonType, command);
        }
    }

    /**
     * Starts the given command whenever the condition changes from 'pressed' to
     * 'unpressed'.
     * Won't cancel the command.
     *
     * @param buttonType the button which will trigger the command
     * @param command    command to be assigned to button
     */
    public void onFalse(ButtonType buttonType, Command command) {
        try {
            Trigger chosenButton = joystickButtonConfiguration.get(buttonType);
            chosenButton.onFalse(command);
        } catch (NullPointerException e) {
            reportButtonError(buttonType, command);
        }
    }

    /**
     * Starts the given command whenever the condition changes from 'pressed' to
     * 'unpressed'.
     * Cancels the given command whenever the condition changes from 'unpressed' to
     * 'pressed'.
     *
     * @param buttonType the button which will trigger the command
     * @param command    command to be assigned to button
     */
    public void whileFalse(ButtonType buttonType, Command command) {
        try {
            Trigger chosenButton = joystickButtonConfiguration.get(buttonType);
            chosenButton.whileFalse(command);
        } catch (NullPointerException e) {
            reportButtonError(buttonType, command);
        }
    }

    /**
     * When the condition changes from 'pressed' to 'unpressed', starts the command
     * if it's not running
     * and cancels the command if it's already running.
     *
     * @param buttonType the button which will trigger the command
     * @param command    command to be assigned to button
     */
    public void toggleOnFalse(ButtonType buttonType, Command command) {
        try {
            Trigger chosenButton = joystickButtonConfiguration.get(buttonType);
            chosenButton.toggleOnFalse(command);
        } catch (NullPointerException e) {
            reportButtonError(buttonType, command);
        }
    }

    /**
     * Starts the given command whenever the condition of ALL chosen buttons
     * is 'pressed'. Won't cancel the command.
     *
     * @param command command to be assigned to button
     * @param buttons these are the buttons which will trigger the command
     */
    public void onTrueCombo(Command command, ButtonType... buttons) {
        int n = 0;
        Trigger triggerList;

        // Cycle through the given buttons until one of them ISN'T null.
        while (n < buttons.length) {
            if (joystickButtonConfiguration.get(buttons[n]) != null) {
                break;
            } else
                reportButtonErrorCombo(buttons[n], command);
            n++;
        }
        // If we've reached end of list and all of them were null, exit the method.
        if (n == buttons.length)
            return;

        // Otherwise, triggerlist will become the first button that isn't null
        triggerList = joystickButtonConfiguration.get(buttons[n]);

        // Add all additional buttons that aren't null.
        for (int i = n; i < buttons.length; i++) {
            try {
                Trigger chosenButton = joystickButtonConfiguration.get(buttons[i]);
                triggerList = triggerList.and(chosenButton);
            } catch (NullPointerException e) {
                reportButtonErrorCombo(buttons[i], command);
            }
        }
        // Assign the command
        triggerList.onTrue(command);

    }

    /**
     * Starts the given command whenever the condition of ALL chosen buttons
     * is 'pressed'. Cancells the command when at least one of the buttons is
     * 'unpressed'.
     *
     * @param command command to be assigned to button
     * @param buttons these are the buttons which will trigger the command
     */
    public void whileTrueCombo(Command command, ButtonType... buttons) {
        int n = 0;
        Trigger triggerList;

        // Cycle through the given buttons until one of them ISN'T null.
        while (n < buttons.length) {
            if (joystickButtonConfiguration.get(buttons[n]) != null) {
                break;
            } else
                reportButtonErrorCombo(buttons[n], command);
            n++;
        }
        // If we've reached end of list and all of them were null, exit the method.
        if (n == buttons.length)
            return;

        // Otherwise, triggerlist will become the first button that isn't null
        triggerList = joystickButtonConfiguration.get(buttons[n]);

        // Add all additional buttons that aren't null.
        for (int i = n; i < buttons.length; i++) {
            try {
                Trigger chosenButton = joystickButtonConfiguration.get(buttons[i]);
                triggerList = triggerList.and(chosenButton);
            } catch (NullPointerException e) {
                reportButtonErrorCombo(buttons[i], command);
            }
        }
        // Assign the command
        triggerList.whileTrue(command);
    }

    /**
     * When all buttons are 'pressed', starts the command if it's not running
     * and cancels the command if it's already running.
     *
     * @param command command to be assigned to button
     * @param buttons these are the buttons which will trigger the command
     */
    public void toggleOnTrueCombo(Command command, ButtonType... buttons) {
        int n = 0;
        Trigger triggerList;

        // Cycle through the given buttons until one of them ISN'T null.
        while (n < buttons.length) {
            if (joystickButtonConfiguration.get(buttons[n]) != null) {
                break;
            } else
                reportButtonErrorCombo(buttons[n], command);
            n++;
        }
        // If we've reached end of list and all of them were null, exit the method.
        if (n == buttons.length)
            return;

        // Otherwise, triggerlist will become the first button that isn't null
        triggerList = joystickButtonConfiguration.get(buttons[n]);

        // Add all additional buttons that aren't null.
        for (int i = n; i < buttons.length; i++) {
            try {
                Trigger chosenButton = joystickButtonConfiguration.get(buttons[i]);
                triggerList = triggerList.and(chosenButton);
            } catch (NullPointerException e) {
                reportButtonErrorCombo(buttons[i], command);
            }
        }
        // Assign the command
        triggerList.toggleOnTrue(command);
    }

    private void reportButtonError(ButtonType b, Command c) {
        DriverStation.reportWarning("Button " + b + " not found! The command " + c + " won't be assigned.", true);
    }

    private void reportButtonErrorCombo(ButtonType b, Command c) {
        DriverStation.reportWarning("Button " + b + " not found when assigning combo for " + c, true);
    }
}
