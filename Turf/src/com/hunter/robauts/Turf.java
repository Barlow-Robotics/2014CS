/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.hunter.robauts;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Turf extends IterativeRobot {
    //control constants
    private final int LIFT_ARM = 3;
    private final int DROP_ARM = 2;
    private final int[] KICK_OVERRIDE = {4, 5};
    private final int[] KICK_NORMAL = {8, 12};
    private final int ARM_PRECISION_MODE = 2;
    
    private final int ARM_LIMIT_SWITCH = 2;
    
    private final double ARM_SPEED = 0.50;
    private final double ARM_MOD = 0.50; //Mode 2 for the arm (left trigger down) is ARM_SPEED * ARM_MOD
    private final double SHOOTER_SPEED = 0.1;
    
    
    private RobotDrive baseDrive;
    private Victor shooterVictor;
    private Victor armVictor;
    private DigitalInput[] digitalIO = new DigitalInput[14];
    private Timer timerTest;
    
    private Joystick joystickLeft;
    private Joystick joystickRight;
    private Joystick joystickAlt;
    
    private Relay cameraLight;
    private Camera camera;
    
    private Solenoid openSolenoid;
    private Solenoid closeSolenoid;
    private Compressor compressor;
    
    private boolean liftInterrupt = false; //If the lift has been interrupted by the limit switch
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        getWatchdog().setEnabled(true);
        getWatchdog().setExpiration(100); //Watchdog will function for 100 milliseconds between feeds
        
        baseDrive = new RobotDrive(
                new Victor(1), //front left motor
                new Victor(2)  //rear left motor
                );
        //baseDrive.setExpiration(0.100); // Set the safety expiration to 100 milliseconds
        baseDrive.setSafetyEnabled(false);

        shooterVictor = new Victor(4);
        armVictor = new Victor(3);
        
        openSolenoid = new Solenoid(1);
        closeSolenoid = new Solenoid(2);
        compressor = new Compressor(1,1);
        
        joystickLeft = new Joystick(1);
        joystickRight = new Joystick(2);
        joystickAlt = new Joystick(3);
        
        for (int i = 0; i < 14; i++) {
            digitalIO[i] = new DigitalInput(i + 1);
        }
        
        cameraLight = new Relay(1);
        camera = new Camera();
    }

    
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousInit() {
        compressor.start();
    }
    public void autonomousPeriodic() {
        camera.autonomous();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopInit() {
        compressor.start();
    }
    public void teleopPeriodic() {
        getWatchdog().feed();
        grabInput();
        driveMotors();
        
        cameraLight.set(Relay.Value.kForward);
        //getWatchdog().feed();
        
        //SmartDashboard.putString("TEST", camera.hotOrNot() ? "HOT": "NOT HOT");
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        getWatchdog().feed();
        grabInput();
        driveMotors();
    }
    
    private void driveMotors() {
        getWatchdog().feed();
        baseDrive.tankDrive(joystickRight, joystickRight.getAxisChannel(Joystick.AxisType.kY), joystickLeft, joystickLeft.getAxisChannel(Joystick.AxisType.kY));
        dispMessage(2, 1, "Right Stick: " + Double.toString(joystickRight.getY()));
        dispMessage(1, 1, "Left Stick: " + Double.toString(joystickLeft.getY()));
    }

    private void grabInput() {
        getWatchdog().feed();
        shooterControl();
        armControl();
    }
    
    private void shooterControl() {
        //Drive the shooter
        if(joystickRight.getRawButton(LIFT_ARM) && !joystickRight.getRawButton(DROP_ARM)) {
            //Slow forward drive for the kicker
            armVictor.set(0.10);
        } else if(joystickRight.getRawButton(DROP_ARM) && !joystickRight.getRawButton(LIFT_ARM)) {
            //Slow back drive for the kicker
            armVictor.set(-0.10);
        } else if(isPressed(joystickAlt, KICK_OVERRIDE) || (joystickLeft.getRawButton(KICK_NORMAL[0]) && joystickRight.getRawButton(KICK_NORMAL[1]))) { //Reg kick
            //Full shoot for the kicker
            armVictor.set(SHOOTER_SPEED);
        } else {
            //Stop shooter if nothing is being pressed
            shooterVictor.set(0);
        }
    }
    private void armControl() {
        //Drive the arm (OLD LEFT JOYSTICK CONTROLS)
        /*if(joystickLeft.getRawButton(LIFT_ARM) && !joystickLeft.getRawButton(DROP_ARM) && canLift()) {
            armVictor.set(ARM_SPEED * (joystickLeft.getButton(Joystick.ButtonType.kTrigger) ? ARM_MOD : 1)); //If left trigger down, multiple ARM_SPEED by ARM_MOD
        } else if(joystickLeft.getRawButton(DROP_ARM) && !joystickLeft.getRawButton(LIFT_ARM)) {
            armVictor.set((ARM_SPEED * (joystickLeft.getButton(Joystick.ButtonType.kTrigger) ? ARM_MOD : 1)) * -1);
        } else {
            armVictor.set(0);
        }*/
        
        
        //Drive the arm (NEW ALT JOYSTICK CONTROLS)
        double armY = -joystickAlt.getAxis(Joystick.AxisType.kY);
        if((armY > 0 && canLift()) || armY < 0) {
            armVictor.set((ARM_SPEED * (joystickAlt.getRawButton(ARM_PRECISION_MODE) ? ARM_MOD : 1)));
        } else {
            armVictor.set(0);
        }
        
        //Drive the gripper
        if(joystickAlt.getButton(Joystick.ButtonType.kTrigger)) {
            //Close solenoid opens
            closeSolenoid.set(true);
            openSolenoid.set(false);
        } else {
            //Open solenoid closes 
            closeSolenoid.set(false);
            openSolenoid.set(true);
        }
        
    }
    
    //Returns true when every item in an int array is pressed (for multi-button controls)
    private boolean isPressed(Joystick js, int[] input) {
        for(int i = 0; i < input.length; i++) {
            if(!js.getRawButton(input[i])) {
                return false;
            }
        }
        
        return true;
    }
    
    //Determines whether or not we can continue to lift the arm further
    private boolean canLift() {
        boolean can = !digitalIO[ARM_LIMIT_SWITCH].get();
        if(can) {
            liftInterrupt = false;
        } else {
            if(!liftInterrupt) { //if this is the first instance of the interrupt
                //Kick the motor back a bit
                armVictor.set(ARM_SPEED * -1);
            }
            liftInterrupt = true;
        }
        return can;
    }
    
    private void dispMessage(int lineNumber, int startingCollumn, String message) {
        DriverStationLCD.Line ln;
        switch (lineNumber) {
            case (0):
                ln = DriverStationLCD.Line.kUser1;
                break;
            case (1):
                ln = DriverStationLCD.Line.kUser2;
                break;
            case (2):
                ln = DriverStationLCD.Line.kUser3;
                break;
            case (3):
                ln = DriverStationLCD.Line.kUser4;
                break;
            case (4):
                ln = DriverStationLCD.Line.kUser5;
                break;
            case (5):
                ln = DriverStationLCD.Line.kUser6;
                break;
            default:
                ln = DriverStationLCD.Line.kUser1;
                break;
        }
        if (startingCollumn < 1 || startingCollumn > 21) {
            startingCollumn = 1;
        }
        DriverStationLCD.getInstance().println(ln, startingCollumn, "                                    ");
        DriverStationLCD.getInstance().println(ln, startingCollumn, message);
        DriverStationLCD.getInstance().updateLCD();
    }
    
    public void disabledInit() {
        
    }
    
}
