package org.usfirst.frc.team1072.robot.subsystems;

import org.usfirst.frc.team1072.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a drive train.
 * @author Finn Frankis
 * @version 6/11/18
 */
public class Drivetrain extends Subsystem
{
    private static Drivetrain instance = null;
    private TalonSRX leftTalon;
    private TalonSRX rightTalon;
    private VictorSPX leftVictor;
    private VictorSPX rightVictor;
    
    public void initDefaultCommand()
    {
        // initialize talons
        leftTalon = new TalonSRX (RobotMap.LEFT_CIM_TALON);
        rightTalon = new TalonSRX (RobotMap.RIGHT_CIM_TALON);
        leftVictor = new VictorSPX (RobotMap.LEFT_CIM_VICTOR);
        rightVictor = new VictorSPX (RobotMap.RIGHT_CIM_VICTOR);
        
    }
    
    public void arcadeDrive(double speed, double turn)
    {
        // victor follows talon
        //leftTalon.set(ControlMode.Velocity, speed - turn);
        rightTalon.set(ControlMode.Velocity, speed + turn);  
        leftTalon.set(ControlMode.Velocity, speed - turn);  
        
         
    }
    
    public void arcadeDrivePosition (double target)
    {
        leftTalon.set(ControlMode.Position, target);
        rightTalon.set(ControlMode.Position, target);
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

    public VictorSPX getLeftVictor()
    {
        return leftVictor;
    }

    public void setLeftVictor(VictorSPX leftVictor)
    {
        this.leftVictor = leftVictor;
    }

    public VictorSPX getRightVictor()
    {
        return rightVictor;
    }

    public void setRightVictor(VictorSPX rightVictor)
    {
        this.rightVictor = rightVictor;
    }

    public static Drivetrain getInstance()
    {
        if (instance == null)
            instance = new Drivetrain();
        return instance;
    }

}
