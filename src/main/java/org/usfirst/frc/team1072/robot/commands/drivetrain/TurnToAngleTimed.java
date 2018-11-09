package org.usfirst.frc.team1072.robot.commands.drivetrain;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain.TurnDirection;
import org.usfirst.frc.team1072.robot.subsystems.Elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.TimedCommand;

public class TurnToAngleTimed extends TimedCommand {
	private TurnDirection direction;
	public TurnToAngleTimed (double time, TurnDirection direction) {
		super (time);
		this.direction = direction;
	}
	
    public void initialize()
    {
        Robot.el.getBottomRightTalon().configSelectedFeedbackSensor
        (FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);

        Robot.dt.selectProfileSlots(Drivetrain.VEL_PID, RobotMap.PRIMARY_PID_INDEX);
        
        
        Robot.dt.getLeftMaster().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.getRightMaster().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
    
        Robot.dt.setTalonSensorPhase(Drivetrain.LEFT_TALON_PHASE, 
                Drivetrain.RIGHT_TALON_PHASE);
        
        Robot.dt.resetTalonCoefficients(Drivetrain.VEL_PID);
    }
    
    /**
     * Executes the command to drive with a given velocity.
     */
    @Override
    public void execute() 
    { 
        OI oi = OI.getInstance();
        
        double leftX = oi.getDriverGamepad().getLeftX();
        double leftY = oi.getDriverGamepad().getLeftY();

        double elevatorPercent = (1.0 * Elevator.getInstance().getBottomRightTalon().getSelectedSensorPosition
        (RobotMap.PRIMARY_PID_INDEX)) / Elevator.SCALE_HIGH_HEIGHT;
        
        
        double elevatorScale = 1;
        if (elevatorPercent > Elevator.THROTTLE_PERCENT) {
            elevatorScale = 1-elevatorPercent*(1-Elevator.MIN_THROTTLE_SPEED);
        }

        double x = 0.8 * Math.pow(Math.abs(leftX), 2) * Math.signum(leftX);
        double y = leftY;
        double k = Math.max(1.0, Math.max(Math.abs(y + x), Math.abs(y - x)));
        double left = elevatorScale * (y + x * Math.abs(x)) / k;
        double right = elevatorScale * (y - x * Math.abs(x)) / k;
        
        Robot.dt.getLeftMaster().set(ControlMode.PercentOutput, ((direction == TurnDirection.LEFT) ? -1 : 1) * left);
        Robot.dt.getRightMaster().set(ControlMode.PercentOutput, ((direction == TurnDirection.LEFT) ? 1 : -1) * right);
    }
}
