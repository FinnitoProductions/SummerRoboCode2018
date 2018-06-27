package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnRobotToAngleCommand extends Command
{
    private double angle;
    
    public TurnRobotToAngleCommand()
    {
        requires (Robot.dt);
        angle = 0;
    }
    public TurnRobotToAngleCommand(double angle)
    {
        requires (Robot.dt);
        this.angle = angle;
    }
    
    public void initialize()
    {
        Robot.dt.velocityConfigureSensors(FeedbackDevice.None);
    }
    @Override
    public void execute()
    {
        OI oi = new OI();

        double joystickRight = oi.getGamepad().getLeftX() *RobotMap.PIGEON_UNITS_PER_ROTATION / 2;
        if (Math.abs(oi.getGamepad().getLeftX()) < 0.1)
            joystickRight = 0;
        Robot.dt.getRightTalon().selectProfileSlot(RobotMap.DT_ANGLE_PID, RobotMap.PRIMARY_PID);
        Robot.dt.getLeftTalon().selectProfileSlot(RobotMap.DT_ANGLE_PID, RobotMap.PRIMARY_PID);
        Robot.dt.getLeftTalon().setSensorPhase(false);
        Robot.dt.getRightTalon().setSensorPhase(false);
        Robot.dt.getRightTalon().set(ControlMode.Position, joystickRight);
        Robot.dt.getLeftTalon().set(ControlMode.Position, -1 * joystickRight);
        SmartDashboard.putNumber("COMMANDED VALUE TO PIGEON", joystickRight);
        SmartDashboard.putNumber("Pigeon Talon Error", Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID));//joystickRight - Robot.dt.getRightTalon().getSelectedSensorPosition(RobotMap.DT_ANGLE_PID));
        
        

    }
    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return false;
    }
    
    @Override
    public void end()
    {
        Robot.dt.velocityConfigureSensors(FeedbackDevice.CTRE_MagEncoder_Relative);
    }

}
