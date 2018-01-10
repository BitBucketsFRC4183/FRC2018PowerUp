package org.usfirst.frc.team4183.utils;

import java.io.PrintWriter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonLogger implements ThreadLogger.Client {
	
	private WPI_TalonSRX motor;
	private ThreadLogger logger;
	
	TalonLogger( WPI_TalonSRX _motor) {
		motor = _motor;		
		logger = new ThreadLogger( this, "talon.txt");
	}

	public void start() {
		logger.start();
	}
	
	public void stop() {
		logger.stop();
	}
	
	@Override
	public void writeLine( PrintWriter writer, long millis) {

		/// TODO: This needs to be completely rethought
//		double sp = motor.getgetSetpoint();       // Setpoint (input)
//		double fb = motor.get();               // Feedback value
//		double err = motor.getError();         // Error value (native units)
//		double ov = motor.getOutputVoltage();  // Drive voltage
//		
//		writer.format("%6d %9.1f %9.1f %9.1f %9.1f\n", millis, sp, fb, err, ov);
	}
}
