/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.hunter.robauts;

/**
 *
 * @author Henry Mound
 */
import edu.wpi.first.wpilibj.DigitalModule;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;

public class Arduino {

    I2C i2c;
    byte[] toSend = new byte[1];
    private SerialPort serial;
    private String recentRead;

    public Arduino() {
	// Use requires() here to declare subsystem dependencies
	// eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    public void initialize() {
	DigitalModule module = DigitalModule.getInstance(2);
	i2c = module.getI2C(168);

	try {
	    serial = new SerialPort(115200);
	    this.serial.disableTermination();
	    this.serial.print("h");
	    System.out.println("Arduino Starting, waiting 0.5 seconds to get data");
	    String e = this.getData();
	    edu.wpi.first.wpilibj.Timer.delay(0.125);

	    if (e.equals("h")) {
		System.out.println("Arduino communications locked in");
	    }
	} catch (Exception e) {
	    System.out.println("something went wrong, " + e.getMessage());
	}
    }

    public void loop() {
	System.out.println("Starting arduino communication");
	recentRead = getData();
    }

    public String getLine() {
	return recentRead;
    }

    //Displays message to the LCD
    public void LCDMessage(String message) {
	try {
	    serial.print("LCD!" + message + "!");
	} catch (Exception e) {
	    System.out.println("Could not send message to LCD");
	}
    }

    public String getData() {
	try {
	    return this.serial.readString();
	} catch (Exception e) {
	    System.out.println("something went wrong, " + e.getMessage());
	    return null;
	}
    }

    public boolean sendData(byte[] buffer) throws Exception {
	try {
	    int count = buffer.length;
	    this.serial.write(buffer, count);
	    return true;
	} catch (Exception e) {
	    System.out.println("something went wrong, " + e.getMessage());
	    return false;
	}
    }

    public boolean printf(String data) {
	try {
	    this.serial.print(data);
         
	    return true;
	} catch (Exception e) {
	    System.out.println("something went wrong, " + e.getMessage());
	    return false;
	}
    }

    public String requestData() {
	try {
	    this.serial.print("r");
	    return this.serial.readString();
	} catch (Exception e) {
	    System.out.println("something went wrong, " + e.getMessage());
	    return null;
	}
    }

//    public int requestData(String request) {
//        try {
//            this.serial.print(request);
//            return NumberUtils.stringToInt(this.getData());
//        } catch (Exception e) {
//            System.out.println("something went wrong, " + e.getMessage());
//            return 0;
//        }
//    }
}