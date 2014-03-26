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
import edu.wpi.first.wpilibj.DriverStation;

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
    private final int ARM_PRECISION_MODE = 2;
    
    private final int ARM_LIMIT_SWITCH = 2;
    
    private double ARM_SPEED = 0.50;
    private final double ARM_MOD = 0.5; //Mode 2 for the arm (left trigger down) is ARM_SPEED * ARM_MOD
    private double SHOOTER_SPEED = 1.0;
    
    private RobotDrive baseDrive;
    private Victor shooterVictor;
    private Victor armVictor;
    private DigitalInput compressorSwitch;
    private DigitalInput armSwitch;
    
    
    private Joystick joystickLeft;
    private Joystick joystickRight;
    private Joystick joystickAlt;
    
    private Relay cameraLight;
    private Camera camera;
    
    private Solenoid openSolenoid;
    private Solenoid closeSolenoid;
    private Solenoid openPeachSolenoid;
    private Solenoid closePeachSolenoid;
    private Compressor compressor;
    
    private DriverStation ds;
    
    private boolean liftInterrupt = false; //If the lift has been interrupted by the limit switch
    private Timer timer;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        getWatchdog().setEnabled(false);
        getWatchdog().setExpiration(100); //Watchdog will function for 100 milliseconds between feeds
        baseDrive = new RobotDrive(
                new Victor(1), //front left motor
                new Victor(8), //rear left
                new Victor(2),  //front right
                new Victor(9) // rear right
                );
                //baseDrive.setExpiration(0.100); // Set the safety expiration to 100 milliseconds
        baseDrive.setSafetyEnabled(false);

        shooterVictor = new Victor(6);
        armVictor = new Victor(3);
        
        openSolenoid = new Solenoid(1);
        closeSolenoid = new Solenoid(2);
        openPeachSolenoid = new Solenoid(3);
        closePeachSolenoid = new Solenoid(4);
        
        compressor = new Compressor(14,1);
        
        joystickLeft = new Joystick(1);
        joystickRight = new Joystick(2);
        joystickAlt = new Joystick(3);       
        
        cameraLight = new Relay(2);
        camera = new Camera();
        timer = new Timer();
        
        armSwitch = new DigitalInput(2);
        
        SmartDashboard.putNumber("Maximum_Shoot", SHOOTER_SPEED);
        SmartDashboard.putNumber("Maximum_Arm_Speed", ARM_SPEED);
        
        ds = DriverStation.getInstance();
        
    }

    public double getKickSpeed() {
        return -1 * SHOOTER_SPEED * (((-1 * joystickAlt.getAxis(Joystick.AxisType.kThrottle))+1)/2);
    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousInit() {
        compressor.start();
        openArm(false);
        moveDistance(2.0); 
        
    }
    public void autonomousPeriodic() {
        
        //camera.autonomous();
        
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopInit() {
        compressor.start();
        
        openArm(true);
    }
    public void teleopPeriodic() {
        getWatchdog().feed();
        grabInput();
        driveMotors();
        
        cameraLight.set(Relay.Value.kForward);
        
        SmartDashboard.putNumber("Kick_Power", (getKickSpeed()));
        dispMessage(4, 1, "Kick ");
        dispMessage(5, 3, "Kick Speed: " + (getKickSpeed()));
        
        SHOOTER_SPEED = SmartDashboard.getNumber("Maximum_Shoot");
        ARM_SPEED = SmartDashboard.getNumber("Maximum_Arm_Speed");
        
        SmartDashboard.putBoolean("Limit_Switch", armSwitch.get());
        SmartDashboard.putBoolean("Pressure_Switch_Tripped", compressor.getPressureSwitchValue());
        SmartDashboard.putString("Time", getTime());
        SmartDashboard.putNumber("Voltage", ds.getBatteryVoltage());
        
       /*if(ds.getBatteryVoltage() < 9) { //Emergency voltage cutoff
            compressor.stop();
        }
        * */
    }
    
    public String getTime() {
        double time = ds.getMatchTime();
        double hours = Math.floor(time/60);
        double minutes = Math.floor(time - hours*60);
        
        String finalTime = hours + ":" + (minutes < 10 ? "0" : "") + minutes;
        
        return finalTime;
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
        //baseDrive.tankDrive(joystickLeft, (-1)*joystickLeft.getAxisChannel(Joystick.AxisType.kY), joystickRight, (-1)*joystickRight.getAxisChannel(Joystick.AxisType.kY));
        baseDrive.tankDrive(joystickLeft.getY()*(-1), joystickRight.getY() *(-1));
        dispMessage(1, 1, "Movement ");
        dispMessage(3, 3, "Right Stick: " + Double.toString(joystickRight.getY()));
        dispMessage(2, 3, "Left Stick: " + Double.toString(joystickLeft.getY()));
        
        //Hat switch micro control
    }
    
    private void grabInput() {
        getWatchdog().feed();
        shooterControl();
        armControl();
    }
    
    private void shooterControl() {
        //Drive the shooter
        if((joystickRight.getRawButton(LIFT_ARM) && !joystickRight.getRawButton(DROP_ARM)) || joystickAlt.getRawButton(10)) {
            //Slow forward drive for the kicker
            //shooterVictor.set(-0.5);
        } else if((joystickRight.getRawButton(DROP_ARM) && !joystickRight.getRawButton(LIFT_ARM)) || joystickAlt.getRawButton(10)) {
            //Slow back drive for the kicker
            //shooterVictor.set(0.5);
        } else if((joystickAlt.getRawButton(8) && joystickAlt.getRawButton(12)) || (joystickLeft.getRawButton(4) && joystickRight.getRawButton(5))) { //Reg kick
            //Full shoot for the kicker
            //shooterVictor.set(getKickSpeed());
        } else {
            //Stop shooter if nothing is being pressed
            //shooterVictor.set(0);
        }
        
        shoot(joystickAlt.getRawButton(3) || joystickRight.getButton(Joystick.ButtonType.kTrigger));
        
        //SmartDashboard.putNumber("KICKER_CURRENT", shooterVictor.get());
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
        double armY = joystickAlt.getAxis(Joystick.AxisType.kY);
        SmartDashboard.putNumber("ARM_Y", armY);
        if((armY > 0 && canLift()) || armY < 0) {
            armVictor.set((ARM_SPEED * (joystickAlt.getRawButton(ARM_PRECISION_MODE) ? ARM_MOD : 1) * armY));
        } else {
            armVictor.set(0);
        }
        
        //Drive the gripper
        openArm(!joystickAlt.getButton(Joystick.ButtonType.kTrigger));
    }
    
    //Determines whether or not we can continue to lift the arm further
    private boolean canLift() {
        boolean can = !armSwitch.get();
        
        //Flyback
        /*if(can) {
            liftInterrupt = false;
        } else {
            if(!liftInterrupt) { //if this is the first instance of the interrupt
                //Kick the motor back a bit
                armVictor.set(ARM_SPEED * -1);
            }
            liftInterrupt = true;
        }*/
        return can;
    }
    
    private void openArm(boolean open) {
        closeSolenoid.set(!open);
        openSolenoid.set(open);
    }
    
    private void shoot(boolean open) {
    	closePeachSolenoid.set(!open);
    	openPeachSolenoid.set(open);
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
    private void moveDistance(double time) {
        //Distance must be in feet
        //feed watchdog
        //feed motor
        //add to the distance
        double timeTravelled = 0.0;
        while (timeTravelled < time) {
            getWatchdog().feed();
            baseDrive.drive(1.0, 0.0);
            Timer.delay(.05);
            timeTravelled += .05;
        }
        baseDrive.stopMotor();
    }
    
}
