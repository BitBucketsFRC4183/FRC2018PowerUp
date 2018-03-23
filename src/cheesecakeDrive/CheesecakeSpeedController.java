package cheesecakeDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.SpeedController;

//Special upcast to allow talons and victors to be seen as speed controllers without the other WPI baggage
//This assumes that the motor has been instantiated already and we are just making it look like a speed controller
public class CheesecakeSpeedController<T> implements SpeedController
{
	/** Constructor */
	private T motorController;
	private double _speed;
	public CheesecakeSpeedController(T aMotorController) 
	{
		motorController = aMotorController;
	}

	// ------ set/get routines for WPILIB interfaces ------//
	@Override
	public void set(double speed) 
	{
		_speed = speed;
		((BaseMotorController) motorController).set(ControlMode.PercentOutput, _speed);
	}

	@Override
	public void pidWrite(double output) 
	{
		set(output);
	}

	/**
	 * Common interface for getting the current set speed of a speed controller.
	 *
	 * @return The current set speed. Value is between -1.0 and 1.0.
	 */
	@Override
	public double get() 
	{
		return _speed;
	}

	// ----------------------- Invert routines -------------------//
	@Override
	public void setInverted(boolean isInverted) 
	{
		((BaseMotorController) motorController).setInverted(isInverted);
	}

	@Override
	public boolean getInverted() 
	{
		return ((BaseMotorController) motorController).getInverted();
	}

	// ----------------------- turn-motor-off routines-------------------//
	@Override
	public void disable() 
	{
		((BaseMotorController) motorController).neutralOutput();
	}

	/**
	 * Common interface to stop the motor until Set is called again.
	 */
	@Override
	public void stopMotor() 
	{
		((BaseMotorController) motorController).neutralOutput();
	}
}

