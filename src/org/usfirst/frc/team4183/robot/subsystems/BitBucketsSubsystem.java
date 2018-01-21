package org.usfirst.frc.team4183.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4183.robot.Robot.RunMode;
import org.usfirst.frc.team4183.robot.subsystems.SubsystemUtilities.DiagnosticsState;

/**
 *
 */
public abstract class BitBucketsSubsystem extends Subsystem {
	
	public boolean runDiagnostics;
	public DiagnosticsState lastKnownState;
	public int DIAG_LOOPS_RUN;
	
	public BitBucketsSubsystem() {
		
	}

	public abstract void diagnosticsInit();
	
	public abstract void diagnosticsCheck();
	
	public abstract void diagnosticsFlagSet();
	
	@Override
    protected abstract void initDefaultCommand();
    
    @Override
    public abstract void periodic();
}

