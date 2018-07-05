package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;

public class FollowPathArcCommand extends FollowPathCommand
{
    public FollowPathArcCommand()
    {
        super(DrivetrainConstants.ANGLE_PID);
    }
}
