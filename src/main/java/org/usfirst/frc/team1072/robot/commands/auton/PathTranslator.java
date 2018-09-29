package org.usfirst.frc.team1072.robot.commands.auton;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

/**
 * @author joel
 *
 */
public class PathTranslator {
	public static void main(String[] args) {
		BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
		Trajectory traj = null;
		while(traj == null) {
			System.out.println("Path filename?");
			try {
				traj = readTrajectory(br.readLine());
			} catch(IOException e) {
				e.printStackTrace();
			}
		}
		String name = null;
		PrintWriter pw = null;
		while(name == null) {
			System.out.println("Path name?");
			try {
				name = br.readLine();
				pw = new PrintWriter(name + "Path.java");
			} catch(FileNotFoundException e) {
				e.printStackTrace();
			} catch(IOException e) {
				e.printStackTrace();
			}
		}
		pw.println("package org.usfirst.frc.team1072.robot.paths;");
		pw.println();
		pw.println("public class " + name + "Path extends Path {");
		pw.println();
		pw.println("\tstatic {");
		pw.println("\t\tPath.paths.put(\"" + name + "\", new " + name + "Path());");
		pw.println("\t}");
		pw.println();
		pw.println("\tpublic " + name + "Path() {");
		pw.println("\t\tsuper(new double[][] {");
		for(int i = 0; i < traj.segments.length; i++) {
			pw.println("\t\t\t{ " + traj.segments[i].velocity + ", " + traj.segments[i].position + " }" + (i != traj.segments.length - 1 ? "," : ""));
		}
		pw.println("\t\t});");
		pw.println("\t}");
		pw.println("}");
		pw.close();
	}
	
	public static Trajectory readTrajectory(String filename) throws FileNotFoundException {
		File f = new File(filename);
		if(f.exists() && f.isFile() && filename.endsWith(".csv")) {
			try {
				return Pathfinder.readFromCSV(f);
			} catch(Exception e) {
				System.out.println("Pathfinder failed to read trajectory: " + filename);
			}
		} else {
			System.out.println("Trajectory: " + filename + ", does not exist or is not a csv file");
		}
		throw new FileNotFoundException("No valid csv file by that name: " + filename);
	}
}
