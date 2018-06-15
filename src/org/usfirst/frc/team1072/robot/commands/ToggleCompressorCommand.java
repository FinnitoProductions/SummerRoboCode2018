package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;

public class ToggleCompressorCommand extends Command
{

    public ToggleCompressorCommand()
    {
        requires(Intake.pn);
    }
    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return true;
    }
    
    public void execute()
    {
        Intake.pn.setCompressor(!Intake.pn.getCompressor().getClosedLoopControl());
    }

}
