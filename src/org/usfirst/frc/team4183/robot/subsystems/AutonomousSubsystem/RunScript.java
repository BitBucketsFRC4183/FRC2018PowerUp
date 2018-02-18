package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunScript extends Command {
			
	// These values written by "MeasureGear"
	static double measuredDistance_inch;    // inch
	static double measuredYaw_deg;          // gives Robot pose in target C.S.; +val means Robot sitting CCW (viewed from top)
	
	private int pc = 0;
	private final boolean debug = true;
	private final int position;
	
	// To see the Scripter instruction set documentation, 
	// scroll down to the switch() in executeNextInstruction()
	
	// Dead reckoning numbers are assuming: 
	// positions 1 & 3 start points are 7' left & right of center line respectively,
	// position 2 start point is on center line (directly facing gear peg)
	
	// Script used in Denver - only one Vision measurement
	private String[][] oneVisionScript = {
			{"", 			"BranchOnPosition Left Center Right" },  // Goto label 1,2,3 based on operator position
			{"Left", 		"DriveStraight 82.2" },  // Inch
			{"", 			"TurnBy -60.0" },        // Degrees, + is CCW from top (RHR Z-axis up)
			{"",			"Goto Vis" },
			{"Center",		"DriveStraight 52.0" },
			{"",			"Goto Vis" },
			{"Right",		"DriveStraight 82.2" },
			{"",			"TurnBy 60.0" },
			{"Vis", 		"EnableVisionGear" },   // S.B. ~4' from airship wall, looking straight at it
			{"", 			"MeasureGear" },		// Collect distance & yaw measures, put estimates into measuredDistance, measuredYaw
			{"", 			"YawCorrect" },     		// TurnBy -measuredYaw
			{"", 			"DistanceCorrect 15.0" },	
			{"", 			"DeliverGear" },			// Spit the gear
			{"",        	"BranchOnColorAndPosition BlueBoiler NoBoiler BackUpBlue BackUpRed NoBoiler RedBoiler"},
			{"NoBoiler",    "DriveStraight -12.0"},
			{"",        	"Goto End"},
			{"BackUpBlue",  "DriveStraight -24.0"},
			{"", 			"TurnBy -60.0"},
			{"",       		"DriveStraight 214.0"},
			{"", 			"Goto End"},
			{"BackUpRed",   "DriveStraight -24.0"},
			{"", 			"TurnBy 60.0"},
			{"",            "DriveStraight 214.0"},
			{"",			"Goto End"},
			{"BlueBoiler",  "StartShooter"},
			{"",   			"DriveStraight -70.2"},
			{"",        	"TurnBy -149.3"},
			{"",        	"Goto Shoot"},
			{"RedBoiler",   "StartShooter"},
			{"",    		"DriveStraight -70.2"},
			{"",        	"TurnBy 149.3"},
			{"",        	"Goto Shoot"},
			{"Shoot",   	"Shoot"},
			{"",			"Delay 8000"},  // Have to Delay to allow shoot to happen!!
			{"End", 		"End" }			// MUST finish in End state
	};
	
	// Original script w/ 2 Vision measurements
	private String[][] twoVisionScript = {
			{"", 		"BranchOnPosition Left Center Right" },  // Goto label 1,2,3 based on operator position
			{"Left", 	"DriveStraight 82.2" },  // Inch
			{"", 		"TurnBy -60.0" },        // Degrees, + is CCW from top (RHR Z-axis up)
			{"",		"Goto Vis" },
			{"Center",	"DriveStraight 26.0" },
			{"",		"Goto Vis" },
			{"Right",	"DriveStraight 82.2" },
			{"",		"TurnBy 60.0" },
			{"Vis", 	"EnableVisionGear" },   // S.B. ~4' from airship wall, looking straight at it
			{"", 		"MeasureGear" },		// Collect distance & yaw measures, put estimates into measuredDistance, measuredYaw
			{"", 		"YawCorrect" },     		// TurnBy -measuredYaw
			{"", 		"DistanceCorrect 21.0" },	// Stop short by this much
			{"", 		"MeasureGear" },
			{"", 		"YawCorrect" },
			{"", 		"DistanceCorrect 15.0" },	
			{"", 		"DeliverGear" },			// Spit the gear
			{"",        "BranchOnColorAndPosition BlueBoiler NoBoiler NoBoiler NoBoiler NoBoiler RedBoiler"},
			{"NoBoiler",    "DriveStraight -12.0"},
			{"",        "Goto End"},
			{"BlueBoiler",   "StartShooter"},
			{"",   		"DriveStraight -70.2"},
			{"",        "TurnBy -149.3"},
			{"",        "Goto Shoot"},
			{"RedBoiler",    "StartShooter"},
			{"",    	"DriveStraight -70.2"},
			{"",        "TurnBy 149.3"},
			{"",        "Goto Shoot"},
			{"Shoot",   "Shoot"},
			{"",		"Delay 4000"},  // Have to Delay to allow shoot to happen!!
			{"End", 	"End" }			// MUST finish in End state
	};
	
	
	private String[][] visionScriptState = {
			{"", 		"BranchOnPosition Left Center Right" },  // Goto label 1,2,3 based on operator position
			{"Left", 	"DriveStraight 82.2" },  // Inch
			{"", 		"TurnBy -60.0" },        // Degrees, + is CCW from top (RHR Z-axis up)
			{"",		"Goto Vis" },
			{"Center",	"DriveStraight 36.0" },
			{"",		"Goto Vis" },
			{"Right",	"DriveStraight 82.2" },
			{"",		"TurnBy 60.0" },
			{"Vis", 	"EnableVisionGear" },   // S.B. ~4' from airship wall, looking straight at it
			{"", 		"MeasureGear" },		// Collect distance & yaw measures, put estimates into measuredDistance, measuredYaw
			{"", 		"YawCorrect" },     		// TurnBy -measuredYaw
			{"", 		"DistanceCorrect 21.0" },	// Stop short by this much
			{"", 		"MeasureGear" },
			{"", 		"YawCorrect" },
			{"", 		"DistanceCorrect 15.0" },	
			{"", 		"DeliverGear" },			// Spit the gear
			{"",        "BranchOnColorAndPosition BlueLeft NoBoiler BlueRight RedLeft NoBoiler RedRight"},
			{"NoBoiler",    "DriveStraight -12.0"},
			{"",        "Goto End"},
			{"BlueLeft",   "DriveStraight -33.75"},
			{"",   		"TurnBy 60.0"},
			{"",        "DriveStraight 100.0"},
			{"",		"TurnBy -35.79"},
			{"",		"DriveStraight 287.25"},
			{"",        "Goto End"},
			{"BlueRight", "DriveStraight -33.75"},
			{"", 		"TurnBy -60.0"},
			{"",		"DriveStraight 333.0"},
			{"",		"Goto End"},
			{"RedLeft", "DriveStraight -33.75"},
			{"", 		"TurnBy 60.0"},
			{"",		"DriveStraight 333.0"},
			{"",		"Goto End"},
			{"RedRight",   "DriveStraight -33.75"},
			{"",   		"TurnBy -60.0"},
			{"",        "DriveStraight 100.0"},
			{"",		"TurnBy 35.79"},
			{"",		"DriveStraight 287.25"},
			{"",        "Goto End"},
			{"End", 	"End" }			// MUST finish in End state
	};
	
	// Demo script
	// Start out 3-4' from peg, rotated 90 degrees (peg should be on Robot's right shoulder)
	private String[][] demoScript = {

			{"",		"TurnBy -90.0" },
			{"", 		"EnableVisionGear" },   
			{"", 		"MeasureGear" },		
			{"", 		"YawCorrect" }, 
			{"", 		"DistanceCorrect 21.0" },
			{"", 		"MeasureGear" },
			{"", 		"YawCorrect" },
			{"", 		"DistanceCorrect 15.0" },	
			{"", 		"DeliverGear" },
			{"",    	"DriveStraight -12.0"},
			{"",		"End"}
	};
	
	// Test small moves to make sure MIN_DRIVEs big enough.
	// e.g. TurnBy 5, DriveStraight 3.
	// Test big moves to make sure it behaves & settles.
	// e.g. TurnBy 60, DriveStraight 48.
	static public String[][] tuneScript = {
			{"",        "BranchOnColorAndPosition BlueLeft BlueCntr Noop RedLeft RedCntr Noop"},
			{"BlueLeft",	"TurnBy 5.0" },
			{"",			"TurnBy -5.0"},
			{"",			"End"},
			{"BlueCntr",	"TurnBy 60.0"},
			{"",			"TurnBy -60.0"},
			{"", 			"End"},		
			{"RedLeft",		"DriveStraight 5.0"},
			{"",			"DriveStraight -5.0"},
			{"",			"End"},
			{"RedCntr",		"DriveStraight 60"},
			{"",			"DriveStraight -60"},
			{"",			"End"},
			{"Noop", 		"End" }    // MUST finish with End!
		};
	
	
	
	
	/*****************************************************************
	 * 
	 * Set this variable to the script you actually want to execute!!!
	 * 
	 *****************************************************************/
	private String[][] script = tuneScript; // Initial script (updated at construction)
	
	
	// position 1,2,3 are Left, Center, Right respectively
    public RunScript( int position, String[][] aScript) {
    	// NO requires in this case
    	// This is a meta state that puts the auto system into other states
    	
    	this.position = position;
    	
    	this.script = aScript;
    }

    protected void initialize() {
    	pc = 0;
    }

    protected void execute() {
    	
    	// When Auto subsystem is Idle, we give it something to do!
    	if( "Idle".equals(Robot.autonomousSubsystem.getCurrentCommand().getName()))
    	    executeNextInstruction();   	
    }

    protected boolean isFinished() {
        return false;
    }
 
    
    private void executeNextInstruction() {
    	
    	if( pc >= script.length) {
    		System.err.println( "Scripter.execute: pc is out of bounds (did you forget End in script?)");
    		return;
    	}
    	
      	String instruction = script[pc++][1];
      	
      	if(debug)
      		System.out.format( "Scripter.execute %s\n", instruction);
      		
    	String[] tokens = instruction.split(" +");
    	switch( tokens[0]) {
    	
    	// These are the legal Instruction Opcodes
    	// For each case in switch, a following comment documents Opcode's parameters if any
    	
    	case "Goto":  // label
    		doGoto(tokens[1]);
    		break;

    	case "Delay":  // msecs
    		delay( Long.parseLong(tokens[1]));
    		break;
    		
    	case "BranchOnPosition":  // Left Center Right (refers to robot position relative to airship centerline)
    		branchOnPosition( tokens[1], tokens[2], tokens[3]);
    		break;
    		
    	case "BranchOnColorAndPosition": // blueLeft, blueCntr, blueRight, redleft, ...
    		branchOnColorAndPosition(tokens[1], tokens[2], tokens[3], tokens[4], tokens[5], tokens[6]);
    		break;
    		
    	case "TurnBy": // yaw (degrees, + is CCW from top)
    		turnBy( Double.parseDouble(tokens[1]));
    		break;
    		
    	case "DriveStraight":  // distance (inches)
    		driveStraight( Double.parseDouble(tokens[1]));
    		break;

    	case "YawCorrect":  // (Turns by -measuredYaw)
    		yawCorrect();
    		break;
    		
    	case "DistanceCorrect":  // drives forward measuredDistance - param)
    		distanceCorrect( Double.parseDouble(tokens[1]));
    		break;
    	
    	case "End":  // (Stops all, does not exit - must be last instruction)
    		endState();
    		break;
    		
    	default:
    		throw new IllegalArgumentException( 
    			String.format("Scripter: unknown instruction: %s\n", instruction));
    	}    	
    }
    
	private void doGoto( String label) {
    	if(debug)
    		System.out.format( "Scripter.doGoto %s\n", label);
    	
    	for( int i = 0 ; i < script.length ; i++)
    		if( script[i][0].equals(label)) {
    			pc = i;
    			return;
    		}
    	
    	throw new IllegalArgumentException(
    		String.format("Scripter.doGoto: Label %s not found\n", label));
    }

    private void delay( long msecs) {
    	if(debug)
    		System.out.format("Scripter.delay %d\n", msecs);
    	(new Delay( msecs)).start();
    }
 
    private void branchOnPosition( String lbl1, String lbl2, String lbl3) {  	
    	if(debug)
    		System.out.format( "Scripter.branchOnLocation %s %s %s\n", lbl1, lbl2, lbl3);
    	
    	switch( position) {
    	case 1:
    		doGoto( lbl1);
    		break;
    	case 2:
    		doGoto( lbl2);
    		break;
    	case 3:
    		doGoto( lbl3);
    		break;
    	default:
    		throw new IllegalArgumentException(
    			String.format( "Scripter.branchOnLocation: unknown location %d\n", position));
    	}
    }
    
    private void branchOnColorAndPosition(String lblB1, String lblB2, String lblB3, 
    		String lblR1, String lblR2, String lblR3) {
    	if(debug)
    		System.out.format("Scripter.branchOnColorAndPosition %s %s %s %s %s %s\n", 
    				lblB1, lblB2, lblB3, lblR1, lblR2, lblR3);

    	if(debug)
    		System.out.format("Scripter.branchOnColorAndPosition Position = %d Color %s\n", 
    				position, Robot.visionSubsystem.isBlueAlliance()?"Blue":"Red");
   	
    	if(Robot.visionSubsystem.isBlueAlliance() && position == 1) {
    		doGoto(lblB1);
    	}
    	else if(Robot.visionSubsystem.isBlueAlliance() && position == 2){
    		doGoto(lblB2);
    	}
    	else if(Robot.visionSubsystem.isBlueAlliance() && position == 3){
    		doGoto(lblB3);
    	}
    	else if(Robot.visionSubsystem.isRedAlliance() && position == 1){
    		doGoto(lblR1);
    	}
    	else if(Robot.visionSubsystem.isRedAlliance() && position == 2){
    		doGoto(lblR2);
    	}
    	else if(Robot.visionSubsystem.isRedAlliance() && position == 3){
    		doGoto(lblR3);
    	}
    	else throw new IllegalArgumentException("Scripter.branchOnColorAndPosition");
    }
     
    private void turnBy( double yaw) {
    	if(debug)
    		System.out.format("Scripter.turnBy %f\n", yaw);
    	(new TurnByOld( yaw)).start();
    }
    
    private void driveStraight( double dist) {
    	if(debug)
    		System.out.format( "Scripter.driveStraight %f\n", dist);
    	(new DriveStraight( dist)).start();
    }
    
    private void enableVisionGear() {
    	if(debug)
    		System.out.println("Scripter.enableVisionGear");
    	Robot.visionSubsystem.setGearMode();
    }
    
    private void yawCorrect() {
    	if(debug)
    		System.out.format("Scripter.yawCorrect %f\n", measuredYaw_deg);
    	(new TurnByOld( -measuredYaw_deg)).start();
    }
    
    private void distanceCorrect( double dRemain) {
    	if(debug)
    		System.out.format( "Scripter.distanceCorrect %f\n", measuredDistance_inch - dRemain);
    	(new DriveStraight( measuredDistance_inch - dRemain)).start();
    }
    
    private void endState() {
    	if(debug)
    		System.out.println("Scripter.endState");
    	(new End()).start();
    }  
}
