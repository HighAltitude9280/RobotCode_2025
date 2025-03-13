// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.HighAltitudeConstants.REEF_HEIGHT;
import frc.robot.HighAltitudeConstants.REEF_POSITION;

/** Add your docs here. */
public class AutoPortion 
{
    private REEF_POSITION pos;
    public REEF_POSITION getPos() {
        return pos;
    }
    private boolean leftBranch;
    public boolean isLeftBranch() {
        return leftBranch;
    }
    private REEF_HEIGHT height;

    public REEF_HEIGHT getHeight() {
        return height;
    }
    private Boolean leftFeeder;
    public Boolean isLeftFeeder() {
        return leftFeeder;
    }
    public AutoPortion(REEF_POSITION pos, boolean leftBranch,REEF_HEIGHT height, Boolean leftFeeder)
    {
        this.pos = pos;
        this.leftBranch = leftBranch;
        this.leftFeeder = leftFeeder;
    }
}
