package org.usfirst.frc.team1072.robot.auto.paths;

import java.util.Map;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderJNI;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;

/**
 *
 * @author Finn Frankis
 *
 */
public abstract class Path {
    private Trajectory leftPath;
    private Trajectory rightPath;
    
    public static final FitMethod FIT_METHOD = FitMethod.HERMITE_QUINTIC;
    public static final int SAMPLE_GENERATION = Config.SAMPLES_HIGH;
    
    public static final double DT_DEFAULT = 0.01;
    public static final double V_DEFAULT = 7.5;
    public static final double ACCEL_DEFAULT = 15;
    public static final double JERK_DEFAULT = 100;
    public static final double WHEELBASE_DEFAULT = 3.0;
    
    public enum SegmentPart {
    	dt, x, y, position, velocity, acceleration, jerk, heading
    }
    
    public Path (Map<SegmentPart, Double>[] leftPath, Map<SegmentPart, Double>[] rightPath) {
    	this.leftPath = generateTrajectory (leftPath);
    	this.rightPath = generateTrajectory (rightPath);
    }
    
    public Path (Waypoint[] waypoints, double dt, double velMax, double accelMax, double jerkMax, double wheelBase) {
    	Trajectory[] generatedPath = PathfinderJNI.modifyTrajectoryTank(Pathfinder.generate(waypoints, new Config(FIT_METHOD, SAMPLE_GENERATION, dt, velMax, accelMax, jerkMax)), wheelBase);
    	leftPath = generatedPath[0];
    	rightPath = generatedPath[1];
    }
    
    public Path (Waypoint[] waypoints, double dt, double velMax, double accelMax, double jerkMax) {
    	this (waypoints, dt, velMax, accelMax, jerkMax, WHEELBASE_DEFAULT);
    }
    
    public Path (Waypoint[] waypoints, double dt) {
    	this (waypoints, dt, V_DEFAULT, ACCEL_DEFAULT, JERK_DEFAULT);
    }
    
    public Path (Waypoint[] waypoints) {
    	this (waypoints, DT_DEFAULT);
    }
    
    public Trajectory getLeftPath() {
    	return leftPath;
    }
    
    public Trajectory getRightPath() {
    	return rightPath;
    }
    
    private Trajectory generateTrajectory (Map<SegmentPart, Double>[] path) {
    	Segment[] segments = new Segment[path.length];
    	int i = 0;
    	for (Map<SegmentPart, Double> segment : path) {
    		segments[i] = new Segment(segment.get(SegmentPart.dt), 
    				segment.get(SegmentPart.x), 
    				segment.get(SegmentPart.y), 
    				segment.get(SegmentPart.position), 
    				segment.get(SegmentPart.velocity), 
    				segment.get(SegmentPart.acceleration),
    				segment.get(SegmentPart.jerk), 
    				segment.get(SegmentPart.heading));
    		i++;
    	}
    	return new Trajectory(segments);
    }
   
}