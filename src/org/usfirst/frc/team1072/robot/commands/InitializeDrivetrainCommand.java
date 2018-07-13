package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * 
 * @author Finn Frankis
 * @version Jul 13, 2018
 */
public class InitializeDrivetrainCommand extends Command
{
    private boolean isFinished;
    public void initialize()
    {
        System.out.println("INITIALIZING DT " + Robot.getCurrentTimeMs());
        Robot.dt.talonInit();
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
            System.out.println("DT INIT FINISHED " + Robot.getCurrentTimeMs());
        return isFinished;
    }
}
