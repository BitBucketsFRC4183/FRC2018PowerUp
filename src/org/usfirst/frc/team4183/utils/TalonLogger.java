package org.usfirst.frc.team4183.utils;

import java.io.PrintWriter;

import com.ctre.CANTalon;

public class TalonLogger implements ThreadLogger.Client {
	
	private CANTalon motor;
	private ThreadLogger logger;
	
	TalonLogger( CANTalon _motor) {
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

		double sp = motor.getSetpoint();       // Setpoint (input)
		double fb = motor.get();               // Feedback value
		double err = motor.getError();         // Error value (native units)
		double ov = motor.getOutputVoltage();  // Drive voltage
		
		writer.format("%6d %9.1f %9.1f %9.1f %9.1f\n", millis, sp, fb, err, ov);
	}
}
