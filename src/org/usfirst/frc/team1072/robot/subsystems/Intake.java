package org.usfirst.frc.team1072.robot.subsystems;

import org.usfirst.frc.team1072.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Intake extends Subsystem
{
    public static Intake intake = null;
    public static Pneumatics pn = Pneumatics.getInstance();
    
    private TalonSRX leftTalon;
    private TalonSRX rightTalon;
    protected void initDefaultCommand()
    {
        // TODO Auto-generated method stub
        leftTalon = new TalonSRX(RobotMap.INTAKE_TALON_LEFT);
        rightTalon = new TalonSRX(RobotMap.INTAKE_TALON_RIGHT);
    }

    public void intakeOuttakeCube(double speedLeft, double speedRight)
    {
        if (!RobotMap.INT_DOUBLE_CONTROLS)
        {
            speedRight = speedLeft;
        }
        leftTalon.set(ControlMode.PercentOutput, speedLeft);
        rightTalon.set(ControlMode.PercentOutput, speedRight);
    }
    
    
    public TalonSRX getLeftTalon()
    {
        return leftTalon;
    }

    public void setLeftTalon(TalonSRX leftTalon)
    {
        this.leftTalon = leftTalon;
    }

    public TalonSRX getRightTalon()
    {
        return rightTalon;
    }

    public void setRightTalon(TalonSRX rightTalon)
    {
        this.rightTalon = rightTalon;
    }

    public static Intake getInstance()
    {
        if (intake == null)
            intake = new Intake();
        return intake;
    }


}
