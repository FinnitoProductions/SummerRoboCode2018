package frc.robot.auto.paths;

import harkerrobolib.auto.Path;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.FitMethod;

public class BaselinePath extends Path {
    public BaselinePath () {
        super (new Waypoint[] {new Waypoint(1.64, 5.0, 0.0), new Waypoint(4, 5.0, 0)}, FitMethod.HERMITE_CUBIC, 0.01, 8, 5, 60, 1.464);
    }
}