package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class MoveElevatorPositionCommand extends Command
{
    public MoveElevatorPositionCommand()
    {
        requires(Robot.el);
    }

    public void execute(double position)
    {
        Robot.el.moveElevatorPosition(position);
    }
    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return false;
    }
}
