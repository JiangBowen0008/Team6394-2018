package org.usfirst.frc.team6394.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class ClosedLoopDrive {
	
	TalonSRX LTalon=new TalonSRX(Constant.LTalonID);
	TalonSRX RTalon=new TalonSRX(Constant.RTalonID);
	VictorSPX RVictor=new VictorSPX(Constant.RVictorID);
	VictorSPX LVictor=new VictorSPX(Constant.LVictorID);
	
	public ClosedLoopDrive() {
		/** Initiate Closed-loop system*/
		
		TalonSRXInit(LTalon);
		TalonSRXInit(RTalon);
		LVictor.follow(LTalon);
		RVictor.follow(RTalon);
	}
	
	private void TalonSRXInit(TalonSRX _talon) {
		//set up TalonSRX and closed loop
		
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
				Constant.kPIDLoopIdx,Constant.kTimeoutMs);
		_talon.setSensorPhase(true);
		
		_talon.configNominalOutputForward(0, Constant.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constant.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constant.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constant.kTimeoutMs);
		
		_talon.config_kF(Constant.kPIDLoopIdx, 0.1097, Constant.kTimeoutMs);
		_talon.config_kP(Constant.kPIDLoopIdx, 0.113333, Constant.kTimeoutMs);
		_talon.config_kI(Constant.kPIDLoopIdx, 0, Constant.kTimeoutMs);
		_talon.config_kD(Constant.kPIDLoopIdx, 0, Constant.kTimeoutMs);
	}
	
	public void velDrive(double forward, double turn) {
		/****** speed mode : clockwise is positive */
		
		/* Convert 500 RPM to units / 100ms.
		 * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
		 * velocity setpoint is in units/100ms
		 */
		LTalon.set(ControlMode.Velocity,
				util.deadband(forward*Math.abs(forward)+turn*Math.abs(turn),0.1)
				* 500.0 * 4096 / 600);
		
		RTalon.set(ControlMode.Velocity,
				util.deadband(forward*Math.abs(forward)-turn*Math.abs(turn),0.1)
				* 500.0 * 4096 / 600);
	}
	
	public void PWMDrive(double forward, double turn) {
		/****** PWM mode : clockwise is positive */
		
		LTalon.set(ControlMode.Velocity,
				util.deadband(forward*Math.abs(forward)+turn*Math.abs(turn),0.1));
		
		RTalon.set(ControlMode.Velocity,
				util.deadband(forward*Math.abs(forward)-turn*Math.abs(turn),0.1));	
	}
	
	public void DisDrive(double dis) {
		
	}
}
