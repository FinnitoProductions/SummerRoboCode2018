package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

public class SetSolenoidCommand extends Command
{

    public SetSolenoidCommand()
    {
        requires(Intake.pn);
    }
    
    public void execute(String key, DoubleSolenoid.Value state)
    {
        Intake.pn.getSolenoid(key).set(state);
    }
    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return false;
    }
    
}
