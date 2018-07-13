package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * 
 * @author Finn Frankis
 * @version Jul 13, 2018
 */
public class InitializeElevatorCommand extends Command
{
    private boolean isFinished;
    public void initialize()
    {
        System.out.println("INITIALIZING ELEVATOR " + Robot.getCurrentTimeMs());
        Robot.el.talonInitAutonomous();
        isFinished = true;
    }

    /**
    * @return
    */
    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        if (isFinished)
            System.out.println("ELEVATOR INIT FINISHED " + Robot.getCurrentTimeMs());
        return isFinished;
    }
}
