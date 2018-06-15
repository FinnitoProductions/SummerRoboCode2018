package org.usfirst.frc.team1072.robot.subsystems;

import org.usfirst.frc.team1072.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Elevator extends Subsystem
{
    private static Elevator el;
    
    private VictorSPX topRightVictor;
    private VictorSPX bottomLeftVictor;
    private VictorSPX topLeftVictor;
    private TalonSRX bottomRightTalon; 
    @Override
    protected void initDefaultCommand()
    {
        topRightVictor = new VictorSPX(RobotMap.ELEVATOR_VICTOR_TOPRIGHT);
        topLeftVictor = new VictorSPX (RobotMap.ELEVATOR_VICTOR_TOPLEFT);
        bottomLeftVictor = new VictorSPX(RobotMap.ELEVATOR_VICTOR_BOTTOMLEFT);
        bottomRightTalon = new TalonSRX(RobotMap.ELEVATOR_TALON);
    }
    
    public void moveElevatorVelocity(double speed)
    {
        bottomRightTalon.set(ControlMode.Velocity, speed, DemandType.ArbitraryFeedForward, RobotMap.EL_POS_FGRAV);
    }
    
    public void moveElevatorPosition(double speed)
    {
        bottomRightTalon.set(ControlMode.Position, speed, DemandType.ArbitraryFeedForward, RobotMap.EL_POS_FGRAV);
    }
    
    public VictorSPX getTopRightVictor()
    {
        return topRightVictor;
    }

    public void setTopRightVictor(VictorSPX topRightVictor)
    {
        this.topRightVictor = topRightVictor;
    }

    public VictorSPX getBottomLeftVictor()
    {
        return bottomLeftVictor;
    }

    public void setBottomLeftVictor(VictorSPX bottomLeftVictor)
    {
        this.bottomLeftVictor = bottomLeftVictor;
    }

    public VictorSPX getTopLeftVictor()
    {
        return topLeftVictor;
    }

    public void setTopLeftVictor(VictorSPX topLeftVictor)
    {
        this.topLeftVictor = topLeftVictor;
    }

    public TalonSRX getBottomRightTalon()
    {
        return bottomRightTalon;
    }

    public void setBottomRightTalon(TalonSRX bottomRightTalon)
    {
        this.bottomRightTalon = bottomRightTalon;
    }

    public static Elevator getInstance()
    {
        if (el == null)
            el = new Elevator();
        return el;
    }
    
}
