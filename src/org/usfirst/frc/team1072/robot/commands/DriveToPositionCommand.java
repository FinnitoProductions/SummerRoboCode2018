package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveToPositionCommand extends Command
{
    public DriveToPositionCommand()
    {
        requires(Robot.dt);
    }

    public void execute(double position)
    {
        Robot.dt.arcadeDrivePosition(position);
    }
    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return true;
    }
    
}
