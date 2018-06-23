//package org.usfirst.frc.team1072.robot.commands;
//
//import edu.wpi.first.wpilibj.command.Command;
//
///**
// * 
// * @author Finn Frankis
// * @version 6/22/18
// */
//public class MotionProfileCombinedCommand extends Command
//{
//    private Command[] commands;
//
//    public MotionProfileCombinedCommand (Command[] commands)
//    {
//        this.commands = commands;
//    }
//    
//    public void initialize()
//    {
//        for (Command c : commands)
//            c.start();
//    }
//    
//
//    @Override
//    protected boolean isFinished()
//    {
//        boolean isFinished = true;
//        for (Command c : commands)
//            isFinished = isFinished && c.isCompleted();
//        return isFinished;
//    }
//    
//    
//}