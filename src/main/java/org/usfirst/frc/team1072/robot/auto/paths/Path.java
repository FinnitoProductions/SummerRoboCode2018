package org.usfirst.frc.team1072.robot.auto.paths;

import java.util.Arrays;
import java.util.Map;

import jaci.pathfinder.Trajectory.Segment;

public abstract class Path {
    private Segment[] path;
    
    public enum SegmentPart {
    	dt, x, y, position, velocity, acceleration, jerk, heading
    }
    
    public Path (Map<SegmentPart, Double>[] path) {
    	this.path = new Segment[path.length];
    	int i = 0;
    	for (Map<SegmentPart, Double> segment : path) {
    		this.path[i] = new Segment(segment.get(SegmentPart.dt), 
    				segment.get(SegmentPart.x), 
    				segment.get(SegmentPart.y), 
    				segment.get(SegmentPart.position), 
    				segment.get(SegmentPart.velocity), 
    				segment.get(SegmentPart.acceleration),
    				segment.get(SegmentPart.jerk), 
    				segment.get(SegmentPart.heading));
    		i++;
    	}
    }
    
    public Segment[] getPath() {
    	return path.clone();
    }
}