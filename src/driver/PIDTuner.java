package driver;

import java.util.ArrayList;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;


public class PIDTuner {
	TextLCD ev3Screen = LocalEV3.get().getTextLCD();
	RegulatedMotor headMotor = new EV3MediumRegulatedMotor(MotorPort.A);
	RegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
	RegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
	
	Port colourSensorPort = LocalEV3.get().getPort("S3");
	
	EV3ColorSensor colourSensor;
	SampleProvider colourSensorProvider;
	float[] colourSample;
	
	double bwMidPoint = 0.5;
	double motorPower = 50;
	double propConstant = 100;
	double diffConstant = 6000;
	
	private ArrayList<Long> oscPeriods = new ArrayList<Long>();
	
	
	
	public static void main(String[] args) {
		PIDTuner pidTuner = new PIDTuner();
	}
		
	public PIDTuner() {
		createSensor();
		runTrack();
	}

	private void createSensor() {
		ev3Screen.drawString("START" , 0, 0);
		colourSensor = new EV3ColorSensor(colourSensorPort);
		colourSensorProvider = colourSensor.getRedMode();
		colourSample = new float[colourSensorProvider.sampleSize()];
	}

	private float getColourReading(){
		colourSensorProvider.fetchSample(colourSample, 0);
		return colourSample[0];
	}
	
	private void runTrack() {
		new Thread(new Runnable() {		
			public void run() {
				double currentReading = 0;
				double currentError = 0;
				double trackCorrection = 0;
				//double derivative = 0;
				//double lastError = 0;
				
				leftMotor.forward();
				rightMotor.forward();

				while(true){
					currentReading = getColourReading();
					long startTime = 0;
					if (currentReading > 0.5){
						startTime = System.currentTimeMillis();
					}
					currentError = currentReading - bwMidPoint;
					//derivative = currentError - lastError;
					trackCorrection = (propConstant * currentError); // + (diffConstant * derivative);
					
					leftMotor.setSpeed((int) (motorPower - trackCorrection));
					rightMotor.setSpeed((int) (motorPower + trackCorrection));
					
					//lastError = currentError;
					
					currentReading = getColourReading();
					long stopTime = 0;
					if (currentReading < 0.5){
						stopTime = System.currentTimeMillis();
					    oscPeriods.add((long) ((stopTime - startTime) * 0.001));
					    long sum = 0;
					    for (int i = 0; i< oscPeriods.size();++i){
					    	sum += oscPeriods.get(i);
					    }
					    ev3Screen.drawString("Osc Period " + (sum  / oscPeriods.size()) , 0, 0);
					}
				}
			}			
		}).start();
	}
}
