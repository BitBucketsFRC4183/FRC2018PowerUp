package org.usfirst.frc.team4183.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Yes this is just a re-invention (very simplified) of PIDController.
// I just don't trust the code in PIDController - overly complicated
// and the synchronization looks VERY ad-hoc and buggy. 
// My faith in WPILib is not running real high right now.
// At least when it doesn't work, I'll have nobody else to blame! --tjw

/**
 * Simple Control Loop implementation
 * Useage:
 * 
 * 1) Construct, providing your implementation of ControlLoopUser
 * 2) Call start()
 * 3) ControlLoop will periodically call ControlLoopUser.getFeedback(), you provide the loop feedback value;
 *    and ControlLoopUser.setError(), you use the error value to drive the plant.
 *    Note these calls are happening in a ControlLoop's separate thread.
 * 4) Call stop() when appropriate (you have to determine when that is).
 *    
 * @author twilson
 *
 */
public class ControlLoop {

	// This maybe should be longer - 40 or 50?
	private final static long DEFAULT_MSECS = 20;  
	private final long msecs;
	private volatile double setPoint;
	private volatile String loggingKey = "";
	
	private final LoopThread loopThread;
	private final ControlLoopUser user;
	
	/**
	 * User of ControlLoop must provide an implementation of this interface
	 * @author twilson
	 */
	public interface ControlLoopUser {
		/**
		 * User must provide the loop feedback value
		 * @return The feedback value
		 */
		public double getFeedback();
		
		/**
		 * Give the User the value of loop error
		 * @param error
		 */
		public void setError( double error);
	}
	
	/**
	 * Constructor
	 * @param user  The user of this class
	 * @param setPoint  The loop set point
	 */
	public ControlLoop( ControlLoopUser user, double setPoint) {
		this( user, setPoint, DEFAULT_MSECS);
	}
	
	/**
	 * Constructor
	 * @param user  The user of this class
	 * @param setPoint  The loop set point
	 * @param msecs  The loop interval
	 */
	public ControlLoop( ControlLoopUser user, double setPoint, long msecs) {
		this.user = user;
		this.setPoint = setPoint;
		this.msecs = msecs;
	
		loopThread = new LoopThread();
		loopThread.setPriority(Thread.NORM_PRIORITY+2);		
	}
	
	
	public void enableLogging( String loggingKey) {
		this.loggingKey = loggingKey;
	}
	
	
	/**
	 * Set the loop setpoint (normally done in constructor)
	 * @param setPoint
	 */
	public void setSetpoint( double setPoint) {
		this.setPoint = setPoint;
	}
		
	
	/**
	 * Start operation
	 */
	public void start() {
		loopThread.start();
	}
	
	/**
	 * Stop operation
	 * Does not return until loop thread has exited.
	 */
	public void stop() {
		
		// Signal control loop to quit
		loopThread.quit();
		
		// Wait for thread to exit
		try {
			loopThread.join();
		} catch (InterruptedException e) {
			// Per this excellent article:
			// http://www.ibm.com/developerworks/library/j-jtp05236/
			Thread.currentThread().interrupt();
		}
		
		user.setError(0.0);
	}
	
	
	// This Thread implements the control loop
	private class LoopThread extends Thread {
						
		private void quit() {
			interrupt();
		}
		
		@Override
		public void run( ) {
									
			// Loop until signaled to quit
			while( !isInterrupted()) {

				double error = setPoint - user.getFeedback();
				
				if( !loggingKey.equals(""))
					SmartDashboard.putNumber(loggingKey, error);
					
				user.setError( error);				
				
				// Delay
				try {
					Thread.sleep(msecs);
				} catch (InterruptedException e) {
					interrupt();
				}				
			}			
		}
	}		
}
