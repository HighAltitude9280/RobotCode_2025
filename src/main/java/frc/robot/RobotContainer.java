// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.resources.components.Navx;

/** Add your docs here. */
public class RobotContainer {

    private Navx navx;

    public RobotContainer() {
        navx = new Navx();

    }

    public Navx getNavx() {
        return navx;
    }
}
