package org.usfirst.frc.team6394.robot;

public abstract class util {
	public static double deadband(double val, double threshold) {
		return Math.abs(val)>=threshold?val:0;
	}
	
	public static double equalsign(double sign, double value) {
		return Math.abs(value)*Math.abs(sign)/sign;
	}
	
	public static boolean isWithin(double value, double center, double range) {
		/*** 
		 * range is in percent
		 */
		
		return (value<=(center+range))&&(value>=(center-range));
	}
}
