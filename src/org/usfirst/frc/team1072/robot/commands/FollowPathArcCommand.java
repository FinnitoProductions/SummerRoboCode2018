package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.RobotMap;

public class FollowPathArcCommand extends FollowPathCommand
{
    public FollowPathArcCommand()
    {
        super(RobotMap.DT_ANGLE_PID, RobotMap.ENABLE_NOTIFIER);
    }
}
