package org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem;

import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.RobotTrajectory;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import jaci.pathfinder.Trajectory;

/*
 * MotionProfileDriver is a 2-controller solution to a left/right tank-style driver
 * 
 * The construction takes two controller or equivalent groups and will feed a left/right profile
 * as commanded
 * 
 * This classis based on the CTRE-Phoenix MotionProfileExample
 * 
 */

public class MotionProfileDriver 
{
	private RobotTrajectory _currentTrajectory;
	
	private final static int BASE_CONTROL_PERIOD_MS = 0;	// Added to each unique point (like scaling the path)
	
	/**
	 * The status of the motion profile executer and buffer inside the Talon.
	 * Instead of creating a new one every time we call getMotionProfileStatus,
	 * keep one copy.
	 */
	private MotionProfileStatus _statusL = new MotionProfileStatus();
	private MotionProfileStatus _statusR = new MotionProfileStatus();
	
	/** additional cache for holding the active trajectory point */
	double _posL=0,_velL=0,_headingL=0;
	double _posR=0, _velR=0, _headingR=0;

	/**
	 * reference to the talon we plan on manipulating. We will not changeMode()
	 * or call set(), just get motion profile status and make decisions based on
	 * motion profile.
	 */
	private TalonSRX _talonL;
	private TalonSRX _talonR;
	
	/**
	 * State machine to make sure we let enough of the motion profile stream to
	 * talon before we fire it.
	 */
	private int _state=0;
	/**
	 * Any time you have a state machine that waits for external events, its a
	 * good idea to add a timeout. Set to -1 to disable. Set to nonzero to count
	 * down to '0' which will print an error message. Counting loops is not a
	 * very accurate method of tracking timeout, but this is just conservative
	 * timeout. Getting time-stamps would certainly work too, this is just
	 * simple (no need to worry about timer overflows).
	 */
	private int _loopTimeout = -1;
	/**
	 * If start() gets called, this flag is set and in the control() we will
	 * service it.
	 */
	private boolean _bStart = false;

	/**
	 * Since the CANTalon.set() routine is mode specific, deduce what we want
	 * the set value to be and let the calling module apply it whenever we
	 * decide to switch to MP mode.
	 */
	private SetValueMotionProfile _setValue = SetValueMotionProfile.Disable;
	/**
	 * How many trajectory points do we wait for before firing the motion
	 * profile.
	 */
	private static final int kMinPointsInTalon = 5;
	/**
	 * Just a state timeout to make sure we don't get stuck anywhere. Each loop
	 * is about 20ms.
	 */
	private static final int kNumLoopsTimeout = 10;
	
	/**
	 * Lets create a periodic task to funnel our trajectory points into our talon.
	 * It doesn't need to be very accurate, just needs to keep pace with the motion
	 * profiler executer.  Now if you're trajectory points are slow, there is no need
	 * to do this, just call _talonL.processMotionProfileBuffer() in your teleop loop.
	 * Generally speaking you want to call it at least twice as fast as the duration
	 * of your trajectory points.  So if they are firing every 20ms, you should call 
	 * every 10ms.
	 */
	class PeriodicRunnable implements java.lang.Runnable 
	{
	    public void run() 
	    {  
	    	// NOTE: Technically there can be temporal shear between
	    	// these two lines such that the advancing of the profile
	    	// could be non-deterministic between left and right.
	    	// Only solution is to use true real-time environments
	    	// since if this happens repeatedly there will be a slight
	    	// drive to one side if all turns are in the same direction.
	    	_talonL.processMotionProfileBuffer();  
	    	_talonR.processMotionProfileBuffer();  
	    }
	}
	Notifier _notifer = new Notifier(new PeriodicRunnable());
	

	/**
	 * C'tor
	 * 
	 * @param talon
	 *            reference to Talon object to fetch motion profile status from.
	 *        profileFramePeriod_ms
	 *            The monotonic period of ALL profiles to be used, we don't
	 *            need to be infintely flexible at this time; just be sure the
	 *            number of points in a trajectory is small-ish to avoid overflowing
	 *            the 128 point buffer in time (e.q., trajectories with 180 points should work)
	 */
	public MotionProfileDriver(TalonSRX _talonL, TalonSRX _talonR, double profilePeriod_ms) 
	{
		_currentTrajectory = new RobotTrajectory();
		
		this._talonL = _talonL;
		this._talonR = _talonR;
		
		 /* since our MP is ??ms per point, set the control frame rate and the
		 * Notifier to half that
		 */
		_talonL.changeMotionControlFramePeriod((int)(profilePeriod_ms/2.0));
		_talonR.changeMotionControlFramePeriod((int)(profilePeriod_ms/2.0));
		_notifer.startPeriodic((int)(profilePeriod_ms / 2000.0));
	}

	/**
	 * Called to clear Motion profile buffer and reset state info during
	 * disabled and when Talon is not in MP control mode.
	 */
	public void reset() 
	{
		/*
		 * Let's clear the buffer just in case user decided to disable in the
		 * middle of an MP, and now we have the second half of a profile just
		 * sitting in memory.
		 */
		_talonL.clearMotionProfileTrajectories();
		_talonR.clearMotionProfileTrajectories();
		/* When we do re-enter motionProfile control mode, stay disabled. */
		_setValue = SetValueMotionProfile.Disable;
		/* When we do start running our state machine start at the beginning. */
		_state = 0;
		
		_loopTimeout = -1;
		/*
		 * If application wanted to start an MP before, ignore and wait for next
		 * button press
		 */
		_bStart = false;

	}

	/**
	 * Called every loop.
	 */
	public void control() 
	{
		/* Get the motion profile status every loop */
		_talonL.getMotionProfileStatus(_statusL);
		_talonR.getMotionProfileStatus(_statusR);

		/*
		 * track time, this is rudimentary but that's okay, we just want to make
		 * sure things never get stuck.
		 */
		if (_loopTimeout < 0) {
			/* do nothing, timeout is disabled */
		} else {
			/* our timeout is nonzero */
			if (_loopTimeout == 0) {
				/*
				 * something is wrong. Talon is not present, unplugged, breaker
				 * tripped
				 */
				MotionProfileInstrumentation.OnNoProgress();
			} else {
				--_loopTimeout;
			}
		}

		/* first check if we are in MP mode */
		if (_talonL.getControlMode() != ControlMode.MotionProfile ||
			_talonR.getControlMode() != ControlMode.MotionProfile) {
			/*
			 * we are not in MP mode. We are probably driving the robot around
			 * using gamepads or some other mode.
			 */
			_state =0;
			_loopTimeout = -1;
		} else {
			/*
			 * we are in MP control mode. That means: starting Mps, checking Mp
			 * progress, and possibly interrupting MPs if thats what you want to
			 * do.
			 */
			switch (_state) {
				case 0: /* wait for application to tell us to start an MP */
					if (_bStart) {
						_bStart = false;
	
						_setValue = SetValueMotionProfile.Disable;
						
						// Only start if we have a trajectory
						if (startFilling())
						{
							/*
							 * MP is being sent to CAN bus, wait a small amount of time
							 */
							_state = 1;
							_loopTimeout = kNumLoopsTimeout;
						}
						else
						{
							System.out.printf("*************** FILL FAILED ***********\n");
						}
					}
					break;
				case 1: /*
						 * wait for MP to stream to Talon, really just the first few
						 * points
						 */
					/* do we have a minimum numberof points in Talon */
					if (_statusL.btmBufferCnt > kMinPointsInTalon && _statusR.btmBufferCnt > kMinPointsInTalon) {
						/* start (once) the motion profile */
						_setValue = SetValueMotionProfile.Enable;
						/* MP will start once the control frame gets scheduled */
						_state = 2;
						_loopTimeout = kNumLoopsTimeout;
					}
					break;
				case 2: /* check the status of the MP */
					/*
					 * if talon is reporting things are good, keep adding to our
					 * timeout. Really this is so that you can unplug your talon in
					 * the middle of an MP and react to it.
					 */
					if (_statusL.isUnderrun == false&& _statusR.isUnderrun==false) {
						_loopTimeout = kNumLoopsTimeout;
					}
					/*
					 * If we are executing an MP and the MP finished, start loading
					 * another. We will go into hold state so robot servo's
					 * position.
					 */
					if (_statusL.activePointValid && _statusL.isLast && _statusR.activePointValid && _statusR.isLast) {
						/*
						 * because we set the last point's isLast to true, we will
						 * get here when the MP is done
						 */
						_setValue = SetValueMotionProfile.Hold;
						_state = 0;
						_loopTimeout = -1;
					}
					break;
			}

			/* Get the motion profile status every loop */
			_talonL.getMotionProfileStatus(_statusL);
			_talonR.getMotionProfileStatus(_statusR);
			
			_headingL = _talonL.getActiveTrajectoryHeading();
			_posL = _talonL.getActiveTrajectoryPosition();
			_velL = _talonL.getActiveTrajectoryVelocity();
			
			_headingR = _talonR.getActiveTrajectoryHeading();
			_posR = _talonR.getActiveTrajectoryPosition();
			_velR = _talonR.getActiveTrajectoryVelocity();

			/* printfs and/or logging */
			/// TODO: Disable when not needed
			MotionProfileInstrumentation.process(_statusL, _posL, _velL, _headingL);
			MotionProfileInstrumentation.process(_statusR, _posR, _velR, _headingR);
		}
	}
	/**
	 * Find enum value if supported.
	 * @param durationMs
	 * @return enum equivalent of durationMs
	 */
	private TrajectoryDuration GetTrajectoryDuration(int durationMs)
	{	 
		/* create return value */
		TrajectoryDuration retval = TrajectoryDuration.Trajectory_Duration_0ms;
		/* convert duration to supported type */
		retval = retval.valueOf(durationMs);
		/* check that it is valid */
		if (retval.value != durationMs) {
			DriverStation.reportError("Trajectory Duration not supported - use configMotionProfileTrajectoryPeriod instead", false);		
		}
		/* pass to caller */
		return retval;
	}
		
	/** Start filling the MPs to all of the involved Talons. */
	private boolean startFilling() 
	{
		boolean fillOk = (_currentTrajectory != null); 
		if (fillOk)
		{
			startFilling(_currentTrajectory.left, _currentTrajectory.right);
		}
		return fillOk;
	}
	private void startFilling(Trajectory profileL, Trajectory profileR) {

		/* create an empty point */
		TrajectoryPoint pointL = new TrajectoryPoint();
		TrajectoryPoint pointR = new TrajectoryPoint();

		/* did we get an underrun condition since last time we checked ? */
		if (_statusL.hasUnderrun||_statusR.hasUnderrun) {
			/* better log it so we know about it */
			MotionProfileInstrumentation.OnUnderrun();
			/*
			 * clear the error. This flag does not auto clear, this way 
			 * we never miss logging it.
			 */
			if(_statusL.hasUnderrun) _talonL.clearMotionProfileHasUnderrun(0);
			if(_statusR.hasUnderrun) _talonR.clearMotionProfileHasUnderrun(0);
		}
		/*
		 * just in case we are interrupting another MP and there is still buffer
		 * points in memory, clear it.
		 */
		_talonL.clearMotionProfileTrajectories();
		_talonR.clearMotionProfileTrajectories();

		/* set the base trajectory period to zero, use the individual trajectory period below */
		_talonL.configMotionProfileTrajectoryPeriod(BASE_CONTROL_PERIOD_MS, RobotMap.CONTROLLER_TIMEOUT_MS);
		_talonR.configMotionProfileTrajectoryPeriod(BASE_CONTROL_PERIOD_MS, RobotMap.CONTROLLER_TIMEOUT_MS);
		
		/* This is fast since it's just into our TOP buffer */
		for (int i = 0; i < profileL.length(); ++i) 
		{
			Trajectory.Segment seg = profileL.get(i);

			/* for each point, fill our structure and pass it to API */
			
			// Motion Profile wants native units for positions and units/100ms for velocity
			// We do this in two steps for clarity
			// First, position converted from meters to rotations, and velocity from m/s to RPM
			pointL.position = RobotMap.meter2inch(seg.position)/ RobotMap.WHEEL_CIRCUMFERENCE_INCHES;	// inches to rotations
       	 	pointL.velocity = RobotMap.meter2inch(seg.velocity) / RobotMap.WHEEL_CIRCUMFERENCE_INCHES * 60.0;
       	 	
       	 	// Second, convert from rotations and RPM to units and units/100ms
			pointL.position = pointL.position * RobotMap.DRIVE_MOTOR_NATIVE_TICKS_PER_REV;         //Convert Revolutions to Units
			pointL.velocity = pointL.velocity * RobotMap.DRIVE_MOTOR_NATIVE_TICKS_PER_REV / 600.0; //Convert RPM to Units/100ms

			pointL.headingDeg = 0; /* future feature - not used in this example*/
			pointL.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
			pointL.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */
			pointL.timeDur = GetTrajectoryDuration((int)(seg.dt*1000));
			pointL.zeroPos = false;
			if (i == 0)
				pointL.zeroPos = true; /* set this to true on the first point */

			pointL.isLastPoint = false;
			if ((i + 1) == profileL.length())
				pointL.isLastPoint = true; /* set this to true on the last point  */

			// Right Side
			seg = profileR.get(i);

			/* for each point, fill our structure and pass it to API */
			// Motion Profile wants native units for positions and units/100ms for velocity
			// We do this in two steps for clarity
			// First, position converted from meters to rotations, and velocity from m/s to RPM
			pointR.position = RobotMap.meter2inch(seg.position)/ RobotMap.WHEEL_CIRCUMFERENCE_INCHES;	// inches to rotations
       	 	pointR.velocity = RobotMap.meter2inch(seg.velocity) / RobotMap.WHEEL_CIRCUMFERENCE_INCHES * 60.0;
       	 	
       	 	// Second, convert from rotations and RPM to units and units/100ms
			pointR.position = pointR.position * RobotMap.DRIVE_MOTOR_NATIVE_TICKS_PER_REV;         //Convert Revolutions to Units
			pointR.velocity = pointR.velocity * RobotMap.DRIVE_MOTOR_NATIVE_TICKS_PER_REV / 600.0; //Convert RPM to Units/100ms

		
			pointR.headingDeg = 0; /* future feature - not used in this example*/
			pointR.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
			pointR.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */
			pointR.timeDur = GetTrajectoryDuration((int)(seg.dt*1000));
			pointR.zeroPos = false;
			if (i == 0)
				pointR.zeroPos = true; /* set this to true on the first point */

			pointR.isLastPoint = false;
			if ((i + 1) == profileR.length())
				pointR.isLastPoint = true; /* set this to true on the last point  */

			_talonL.pushMotionProfileTrajectory(pointL);
			_talonR.pushMotionProfileTrajectory(pointR);
		}
	}
	
	/**
	 * Set a current trajectory prior to starting the engine
	 * If you don't then the engine will simply no do anything
	 */
	public void setCurrentTrajectory(RobotTrajectory aTrajectory)
	{
		reset();
		_currentTrajectory = aTrajectory;
	}
	
	public void startCurrentTrajectory()
	{
		_talonL.set(ControlMode.MotionProfile, _setValue.value);
		_talonR.set(ControlMode.MotionProfile, _setValue.value);
	
		startMotionProfile();
	}
	
	/**
	 * Called by application to signal Talon to start the buffered MP (when it's
	 * able to).
	 */
	private void startMotionProfile() {
		_bStart = true;
	}

	/**
	 * 
	 * @return the output value to pass to Talon's set() routine. 0 for disable
	 *         motion-profile output, 1 for enable motion-profile, 2 for hold
	 *         current motion profile trajectory point.
	 */
	public SetValueMotionProfile getSetValue() {
		return _setValue;
	}	
}
