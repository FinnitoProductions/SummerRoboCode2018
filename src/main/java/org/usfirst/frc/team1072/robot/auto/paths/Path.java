package org.usfirst.frc.team1072.robot.auto.paths;

import java.util.Map;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;

public abstract class Path {
    private Trajectory path;
    
    public static final FitMethod FIT_METHOD = FitMethod.HERMITE_QUINTIC;
    public static final int SAMPLE_GENERATION = Config.SAMPLES_HIGH;
    
    public enum SegmentPart {
    	dt, x, y, position, velocity, acceleration, jerk, heading
    }
    
    public Path (Map<SegmentPart, Double>[] path) {
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
    	this.path = new Trajectory(segments);
    }
    
    public Path (Waypoint[] waypoints, double dt, double velMax, double accelMax, double jerkMax) {
    	this.path = Pathfinder.generate(waypoints, new Config(FIT_METHOD, SAMPLE_GENERATION, dt, velMax, accelMax, jerkMax));
    }
    
    public Trajectory getPath() {
    	return path;
    }
    
    
}