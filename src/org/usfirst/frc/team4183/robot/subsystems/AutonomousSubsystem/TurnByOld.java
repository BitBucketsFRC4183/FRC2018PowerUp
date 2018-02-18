package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.ControlLoop;
import org.usfirst.frc.team4183.utils.LogWriterFactory;
import org.usfirst.frc.team4183.utils.RateLimit;
import org.usfirst.frc.team4183.utils.SettledDetector;

import edu.wpi.first.wpilibj.command.Command;



public class TurnByOld extends Command implements ControlLoop.ControlLoopUser {
		
	// Largest drive that will be applied
	private final double MAX_DRIVE = 0.9;
	
	// Smallest drive that will be applied
	// (unless error falls within dead zone, then drive goes to 0)
	// THIS MUST BE LARGE ENOUGH TO ROTATE THE ROBOT from stopped position;
	// if it isn't, you can get stuck in this state.
	// But if this is TOO BIG, you'll get limit cycling, and also get stuck.
	private final double MIN_DRIVE = RobotMap.TURNBY_MIN_DRIVE; 
	
	// Angle at which we reduce drive from MAX to MIN	
	private final double MIN_DRIVE_ANGLE_DEG = 40.0;
	
	// Size of dead zone in degrees - also used to determine when done.
	private final double DEAD_ZONE_DEG = 1.0;

	// Dither signal
	private final double DITHER_AMPL = 0.12;
	private final double DITHER_FREQ = 4.0;
	
	// Settled detector lookback for dead zone
	// I would NOT go lower than 150 because of Java thread jitter
	private final long SETTLED_MSECS = 150;
	
	// Also used to determine when done
	private final double STOPPED_RATE_DPS = 2.0;
		
	// Limits ramp rate of drive signal
	private final double RATE_LIM_PER_SEC = 3.0;
		
	private final double degreesToTurn;
	
	private ControlLoop cloop;
	private RateLimit rateLimit;
	private SettledDetector settledDetector;
		

	private boolean WRITE_LOG_FILE = false;
	private static LogWriterFactory logFactory = new LogWriterFactory("TurnBy");
	private LogWriterFactory.Writer logWriter;
	
	
	public TurnByOld( double degreesToTurn) {		
		requires( Robot.autonomousSubsystem);
		
		this.degreesToTurn = degreesToTurn;		
	}

	@Override
	protected void initialize() {
		// Compute setPoint
		double setPoint = degreesToTurn + Robot.imu.getYawDeg();
		
		// Make helpers
		rateLimit = new RateLimit( RATE_LIM_PER_SEC);
		settledDetector = new SettledDetector( SETTLED_MSECS, DEAD_ZONE_DEG);
		logWriter = logFactory.create( WRITE_LOG_FILE);	
			
		// Fire up the loop
		cloop = new ControlLoop( this, setPoint);
		cloop.enableLogging("TurnBy");
		cloop.start();
	}
	

	
	@Override
	protected boolean isFinished() {
		
		if( settledDetector.isSettled() 
			&&
			Robot.imu.getYawRateDps() < STOPPED_RATE_DPS
		) {
			return true;
		}
		
		return false;
	}
	
	@Override
	protected void end() {
	
		// Don't forget to stop the control loop!
		cloop.stop();
		
		logWriter.close();

		// Set output to zero before leaving
    	Robot.driveSubsystem.doAutoTurn(0.0);
	}
	
	@Override
	protected void interrupted() {
		end();
	}
	
	
	@Override
	public double getFeedback() {
		return Robot.imu.getYawDeg();
	}
	
	@Override
	public void setError( double error) {
		
		logWriter.writeLine( 
				String.format("%f %f", error, Robot.imu.getYawRateDps())); 
			
		settledDetector.set(error);
										
		double x;		
		if( Math.abs(error) < MIN_DRIVE_ANGLE_DEG)
			x = Math.signum(error)*MIN_DRIVE;
		else 
			x = Math.signum(error)*MAX_DRIVE;		
		
		x = rateLimit.f(x);
		
		if( Math.abs(error) < DEAD_ZONE_DEG)
			x = 0.0;				

		if( Math.abs(error) > DEAD_ZONE_DEG)
			x += DITHER_AMPL*ditherSignal();

		// Set the output
    	Robot.driveSubsystem.doAutoTurn(x);		
	}
	
	double ditherSignal() {
		return Math.sin( DITHER_FREQ*(2.0*Math.PI)*System.currentTimeMillis()/1000.0);
	}

}
