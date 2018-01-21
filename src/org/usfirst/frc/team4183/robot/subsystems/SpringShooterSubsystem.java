package org.usfirst.frc.team4183.robot.subsystems;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SpringShooterSubsystem extends BitBucketsSubsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new Idle());
    }

	@Override
	public boolean diagnostics() {
		// TODO Auto-generated method stub
		return false;
	}
}

