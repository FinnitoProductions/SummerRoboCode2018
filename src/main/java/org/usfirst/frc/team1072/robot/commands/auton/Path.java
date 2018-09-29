package org.usfirst.frc.team1072.robot.commands.auton;

import jaci.pathfinder.Trajectory.Segment;

public abstract class Path {
    Segment[] path;

    public Path (Double[] path) {

        //new Segment(dt, x, y, position, velocity, acceleration, jerk, heading);
    }
}