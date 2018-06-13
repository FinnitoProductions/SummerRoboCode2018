package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ArcadeDriveCommand extends Command
{

    public ArcadeDriveCommand()
    {
        requires(Robot.dt);
    }
    
    public void execute(double speed, double turn)
    {
        Robot.dt.arcadeDrive(speed, turn);
    }
    @Override
    protected boolean isFinished()
    {
        return true;
    }

}
