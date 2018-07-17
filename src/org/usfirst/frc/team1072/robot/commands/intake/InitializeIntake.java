package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * 
 * @author Finn Frankis
 * @version Jul 13, 2018
 */
public class InitializeIntake extends Command
{
    private boolean isFinished;
    public void initialize()
    {
        System.out.println("INITIALIZING INTAKE " + Robot.getCurrentTimeMs());
        Robot.intake.talonInit();
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
            System.out.println("INTAKE INIT FINISHED " + Robot.getCurrentTimeMs());
        return isFinished;
    }
}
