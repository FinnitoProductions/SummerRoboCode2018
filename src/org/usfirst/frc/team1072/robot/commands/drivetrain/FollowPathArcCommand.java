package org.usfirst.frc.team1072.robot.commands.drivetrain;

import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.commands.auton.FollowPathCommand;

/**
 * Follows a motion profile in an arc-like manner.
 * @author Finn Frankis
 * @version 6/28/18
 */
public class FollowPathArcCommand extends FollowPathCommand
{
    /**
     * Constructs a new motion profile arc.
     */
    public FollowPathArcCommand()
    {
        super(DrivetrainConstants.ANGLE_PID);
    }
}
