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
	
	protected boolean runDiagnostics = false;
	public DiagnosticsState lastKnownState = DiagnosticsState.UNKNOWN;
	public int DIAG_LOOPS_RUN = 5;
	
	public BitBucketsSubsystem() {
		
	}

	public abstract void diagnosticsInit();
	
	public abstract void diagnosticsExecute();
	
	public abstract void diagnosticsCheck();
	
	public abstract void setDiagnosticsFlag(boolean enable);
	
	public abstract boolean getDiagnosticsFlag();
	
	@Override
    protected abstract void initDefaultCommand();
    
    @Override
    public abstract void periodic();
}

