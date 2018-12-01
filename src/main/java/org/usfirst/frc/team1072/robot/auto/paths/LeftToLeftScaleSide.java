package org.usfirst.frc.team1072.robot.auto.paths;

import java.util.Arrays;

import harkerrobolib.auto.Path;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.Waypoint;

/**
 * Represents a standard path starting from the left side to the side of the scale.
 * @author Finn Frankis
 * @version 10/6/18
 */
public class LeftToLeftScaleSide extends Path {
	public LeftToLeftScaleSide () {
		super (
				new Waypoint[] {new Waypoint(1.64, 13.0, 0.0), new Waypoint(10.5, 17.5, 20.0)}, FitMethod.HERMITE_CUBIC, 0.01, 8, 5, 60, 1.464);
	}
}
