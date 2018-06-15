package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class MoveElevatorVelocityCommand extends Command
{
    
    public MoveElevatorVelocityCommand()
    {
        requires(Robot.el);
    }
    
    public void execute(double speed)
    {
        Robot.el.moveElevatorVelocity(speed);
    }
    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return true;
    }

}
