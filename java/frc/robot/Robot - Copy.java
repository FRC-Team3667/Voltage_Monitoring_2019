/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import com.ctre.phoenix.motorcontrol.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  WPI_TalonSRX mixTalon;
  WPI_VictorSPX misterVictor;
  Joystick missController;
  PowerDistributionPanel peedeepee;

    double motorCurrent ;
    double pdpCurrent ;
    double currentTime;
    double currentInput;
    Timer robotTimer = new Timer(); 
    double currents[][] = new double[4][10];
    double avgCurrents[][] = new double[4][10];
    int currentCount;
    int avgCount;
    public static final int MOTOR_VAL = 0;
    public static final int PDP_VAL = 1;
    public static final int TIME_VAL = 2;
    public static final int INPUT_VAL = 3;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    robotTimer.start();
    mixTalon = new WPI_TalonSRX(10);
    mixTalon.setNeutralMode(NeutralMode.Coast);
    missController = new Joystick(0);
    peedeepee = new PowerDistributionPanel();

    
    currentCount = 0;
    avgCount = 0;
    currentTime = 0;
    currentInput =0;

    //init arrays
    for(int i = 0; i < 10; i++) {
      for(int j = 0; j<4 ;j++) {
        currents[j][i]=0;
        avgCurrents[j][i]=0;
      }
    }

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
//todo average current only over 1 sec see how often updates and average data before displaying
    SmartDashboard.putNumber("Current Motor Current", motorCurrent);
    SmartDashboard.putNumber("Current PDP Current", pdpCurrent);
    SmartDashboard.putNumber("Current Timestamp", pdpCurrent);
    SmartDashboard.putNumber("Current Input", pdpCurrent);

    for(int i = 0; i < 10; i++) {
    SmartDashboard.putNumber("Avg Motor Current " + i, avgCurrents[MOTOR_VAL][i]);
    SmartDashboard.putNumber("Avg PDP Current " + i, avgCurrents[PDP_VAL][i]);
    SmartDashboard.putNumber("Avg Timestamp " + i, avgCurrents[TIME_VAL][i]);
    SmartDashboard.putNumber("Avg Input " + i, avgCurrents[INPUT_VAL][i]);
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    //get speed input
    if (missController.getRawButton(1))
    {
      currentInput = 0.5; // Half!
    } else if  (Math.abs(missController.getRawAxis(1))>.1) {
      currentInput = missController.getRawAxis(1);
    } else {
      currentInput = 0.0; 
    }

    mixTalon.set(currentInput);


    double motorCurrent = mixTalon.getOutputCurrent();
    double pdpCurrent = peedeepee.getCurrent(0);
    currentTime = robotTimer.get();
    currentInput = mixTalon.get();

    currents[MOTOR_VAL][currentCount] = motorCurrent;
    currents[PDP_VAL][currentCount] = pdpCurrent;
    currents[TIME_VAL][currentCount] = currentTime;
    currents[INPUT_VAL][currentCount] = pdpCurrent;
    currentCount++;
    if(currentCount ==10) {
      currentCount = 0;
      double totMotorCurrent = 0;
      double totPdpCurrent = 0;
      ;
      double totInput = 0;
      for(int i = 0; i < 10; i++) {
        totMotorCurrent = currents[MOTOR_VAL][i] + totMotorCurrent;
        totPdpCurrent = currents[PDP_VAL][i] + totPdpCurrent;
        totInput = currents[INPUT_VAL][i] + totInput;
      }
      avgCurrents[MOTOR_VAL][avgCount] = totMotorCurrent/10;
      avgCurrents[PDP_VAL][avgCount] = totPdpCurrent/10;
      avgCurrents[TIME_VAL][avgCount] = (currents[INPUT_VAL][0] + currents[INPUT_VAL][9])/2;
      avgCurrents[INPUT_VAL][avgCount] = totInput/10;
      avgCount++;
      if(avgCount ==10) {
        avgCount = 0;
      }
    }
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
