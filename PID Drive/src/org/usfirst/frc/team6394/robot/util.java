package org.usfirst.frc.team6394.robot;

public abstract class util {
	public static double deadband(double val, double threshold) {
		return Math.abs(val)>=threshold?val:0;
	}
}
