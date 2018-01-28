package org.usfirst.frc.team4183.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavxIMU {
	
	private final AHRS ahrs;
	
	// 60 Hz is the default, we're just giving it the default,
	// but we need to know this number to compute yawRate.
	private final int AHRS_UPDATE_RATE = 60;  
	private final boolean DEBUG_THREAD = false;
	
	NavxIMU() {
		
		System.out.println( "Starting NavX AHRS");				
		ahrs = new AHRS(SPI.Port.kMXP, (byte)AHRS_UPDATE_RATE);

		// Wait a bit in background, the print connected & firmware info
		new Thread() {
			public void run() {
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {}
				System.out.format("NavX isConnected=%b firmware=%s\n", 
						ahrs.isConnected(), ahrs.getFirmwareVersion());
				SmartDashboard.putString("firmware version", ahrs.getFirmwareVersion());
			}
		}.start();
		
		
		// Start thread to print out something at a reasonable rate (testing)
		if( DEBUG_THREAD) {
			new Thread() { 
				public void run() {
					while(true) {
						try {
							Thread.sleep(500);
						} catch (InterruptedException e) {}	
						System.out.format("isConnected=%b isCalibrating=%b Yaw=%.2f Rate=%.2f\n", 
								isConnected(), isCalibrating(), getYawDeg(), getYawRateDps());						
					}
				}
			}.start();
		}				
	}
	
	
	// Return yaw angle according to right-hand-rule with z-axis up;
	// that is, +yaw is CCW looking down on robot.	
	public synchronized double getYawDeg() {
		
		if( !isConnected()) {
			System.err.println( "Error, Yaw requested but NavX not connected");
			return 0.0;
		}
		
		if( isCalibrating()) {
			System.err.println( "Warning, Yaw requested but NavX is calibrating");
		}
		
		// Need the - sign to get the Navx to agree with the yaw definition
		return -ahrs.getAngle();
	}

	public synchronized double getYawRateDps() {
		
		if( !isConnected()) {
			System.err.println( "Error, Rate requested but NavX not connected");
			return 0.0;
		}
		
		if( isCalibrating()) {
			System.err.println( "Warning, Rate requested but NavX is calibrating");
		}
		
		// The NavX getRate() does NOT return degrees per second (as claimed);
		// it actually returns just the difference between two successive yaw values in degrees
		// (so it actually returns degrees per delta-T).
		// They forgot to multiply by their own internal update rate
		// (60 HZ by default). So we do it here.
		// Need the - sign to get the Navx to agree with the yaw definition.		
		return -AHRS_UPDATE_RATE*ahrs.getRate();
	}
	
	public synchronized double getFwdAccel_G() {
		
		if( !isConnected()) {
			System.err.println( "Error, Accel requested but NavX not connected");
			return 0.0;
		}
		
		if( isCalibrating()) {
			System.err.println( "Warning, Accel requested but NavX is calibrating");
		}
		
		return ahrs.getRawAccelX();
	}
	
	public synchronized boolean isConnected() {
		return ahrs.isConnected();
	}
	
	public synchronized boolean isCalibrating() {
		return ahrs.isCalibrating();
	}

	public void diagnostics()
	{
		  /* Display 6-axis Processed Angle Data                                      */
        SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
        SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
        SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
        SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
        SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
        
        /* Display tilt-corrected, Magnetometer-based heading (requires             */
        /* magnetometer calibration to be useful)                                   */
        
        SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
        
        /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
        SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
        /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
        
        SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
        SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
        
        SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
        SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
        SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
        SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

        /* Display estimates of velocity/displacement.  Note that these values are  */
        /* not expected to be accurate enough for estimating robot position on a    */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially      */
        /* double (displacement) integration.                                       */
        
        SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
        SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
        SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
        SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
        
        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
        /* NOTE:  These values are not normally necessary, but are made available   */
        /* for advanced users.  Before using this data, please consider whether     */
        /* the processed data (see above) will suit your needs.                     */
        
        SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
        SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
        SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
        SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
        SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
        SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
        SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
        SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
        SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
        SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
        
        /* Omnimount Yaw Axis Information                                           */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
        AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
        SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
        SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
        
        /* Sensor Board Information                                                 */
        SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
        
        /* Quaternion Data                                                          */
        /* Quaternions are fascinating, and are the most compact representation of  */
        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
        /* from the Quaternions.  If interested in motion processing, knowledge of  */
        /* Quaternions is highly recommended.                                       */
        SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
        SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
        SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
        SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
        
        /* Connectivity Debugging Support                                           */
        SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
        SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());
	}
}
