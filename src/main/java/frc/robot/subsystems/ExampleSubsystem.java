// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class ExampleSubsystem extends SubsystemBase {

  // Declaring motors
  private final CANSparkMax mainMotor;  // This is the motor with id (#3) and is the main motor
  private final CANSparkMax motorId13;  // This is the motor with id (#13)

  // Declaring controller
  private final XboxController controller; //This is the Xbox Controller

  // State variables
  private boolean isItOn;  // Motor running state
  private boolean eBrake;  // Emergency brake state
  private int direction;   // Direction (-1 for reverse, 1 for forward)
  private int speed;       // Speed (1-100)

  //Motor Stats varriables
  private double motorVoltage;
  private double motorCurrent;

  public ExampleSubsystem() {

    // Setting motor info
    mainMotor = new CANSparkMax(3, MotorType.kBrushless);  // This is the motor with id (#3)
    motorId13 = new CANSparkMax(13, MotorType.kBrushless); // This is the motor with id (#13)

    mainMotor.setIdleMode(CANSparkMax.IdleMode.kCoast); //turning braking to Coast
    motorId13.setIdleMode(CANSparkMax.IdleMode.kCoast); //turning braking to Coast

    motorId13.follow(mainMotor); //motor #13 will reflect mainMotor movment (not including e-brakes)

    // Setting controller info
    controller = new XboxController(0); 

    // Default states
    isItOn = false;   // Motors off by default
    eBrake = false;   // E-brake off by default
    direction = 1;    // Default to forward direction
    speed = 50;       // Default speed at 50%

    // Default Voltage and Current
    motorVoltage = (mainMotor.getBusVoltage() + motorId13.getBusVoltage())/2;
    motorCurrent = (mainMotor.getOutputCurrent() + motorId13.getOutputCurrent())/2;
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    //Gettig Current Voltage and Current
    motorVoltage = (mainMotor.getBusVoltage() + motorId13.getBusVoltage())/2;
    motorCurrent = (mainMotor.getOutputCurrent() + motorId13.getOutputCurrent())/2;
    System.out.println("Voltage: " + motorVoltage +" Current: " + motorCurrent);

    //EBRAKE ACTIVATION
    // Cannot be undone unless system restarted
    if(controller.getAButtonPressed()){ // turns OFF motors and turns brakes ON
      isItOn = false;
      eBrake = true;
    }

    // Motor E-Brakes Controls
    if(eBrake){ 
      //have to set each motor individualy
      mainMotor.set(0);
      mainMotor.setIdleMode(CANSparkMax.IdleMode.kBrake); 
      motorId13.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    // Motor On/OFF Controls Checking

    if(controller.getRightBumperPressed()){//turns motor OFF
      isItOn = false; 

    } else if(controller.getLeftBumperPressed() && !eBrake){ //turns motors on var ON only if ebrake is OFF
      isItOn = true; 

    }

    // Controls for if motor is on and ebrake is off

    if(isItOn && !eBrake){ //functions for motor speed and direction
      mainMotor.set(direction * (0.01 * speed)); //0.01 is the scale factor eg: 50% -> 0.5
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
