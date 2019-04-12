package frc.robot.commands.elevator;

import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;

/**
 * 
 * @author Finn Frankis
 * @author Angela Jia
 * @version 11/8/18
 */
public class ZeroElevator extends Command {
	public static final double STALL_CURRENT = 7;
	public static final double DOWN_SPEED = -0.3 - 0.1;
	public static final int END_HEIGHT = -616;
	public ZeroElevator () {
		requires (Robot.el);
	}
	
	public void initialize() {
		Robot.el.getBottomRightTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
		
		Robot.el.getBottomRightTalon().set(ControlMode.PercentOutput, DOWN_SPEED);
	}
	
	public void execute() {
		Robot.el.getBottomRightTalon().set(ControlMode.PercentOutput, DOWN_SPEED);
	}
	@Override
	protected boolean isFinished() {
		return Robot.el.getBottomRightTalon().getOutputCurrent() > STALL_CURRENT;
	}
	
	protected void end () {
		Robot.el.getBottomRightTalon().setSelectedSensorPosition(END_HEIGHT, RobotMap.PRIMARY_PID_INDEX);
	}
	
	
}
