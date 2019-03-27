package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.analog.adis16448.frc.ADIS16448_IMU;

public class Robot extends TimedRobot {
  // Victors
  WPI_VictorSPX _frontTLeftMotor;
  WPI_VictorSPX _frontTRightMotor;
  WPI_VictorSPX _rearTRightMotor;
  WPI_VictorSPX _rearTLeftMotor;
  WPI_VictorSPX _intakeUpperMotor;
  WPI_VictorSPX _intakeLowerMotor;

  // Talons
  WPI_TalonSRX _frontLifterOne;
  WPI_TalonSRX _frontLifterTwo;
  WPI_TalonSRX _rearLifterMotor;
  WPI_TalonSRX _intakeLifterMotor;
  SpeedControllerGroup frontLifterMotors;
  MecanumDrive _mDrive;
  Joystick _joy1;
  Joystick _joy2;
  private UsbCamera camera;

  // Network Tables
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;
  NetworkTable table;

  // Serial Port Information
  SerialPort theThePort;

  public enum RunningInMode {
    none, teleop, auton, test;
  }

  RunningInMode runMode = RunningInMode.none;

  // Line Tracker
  DigitalInput lineTracker0;
  DigitalInput lineTracker1;
  DigitalInput lineTracker2;
  DigitalInput lineTracker3;
  DigitalInput lineTracker4;

  // Lifter Logic
  double frontLiftSpeed = 0.50;
  double rearLiftSpeed = 0;
  double maxFrontLiftSpeed = 0.9;
  double maxRearLiftSpeed = 0.9;

  // 10 Degrees of Freedom
  ADIS16448_IMU imu;
  double zDegree = 0;
  double xDegree = 0;
  double yDegree = 0;
  boolean didItAlready = false;
  boolean imuIsWorkingCorrectly = true; // IMU is Working or Not
  Timer robotTimer = new Timer();

  // Line Tracker Values
  double pastXDegree = xDegree;
  int xDegreeIterations = 0;
  double targetDegree = 0;
  double rotationCounter = 1;
  double turnRotation = 0;
  double forwardMotion = 0;
  double lineTrackerEndTime = 0;
  boolean lTrack0 = false;
  boolean lTrack1 = false;
  boolean lTrack2 = false;
  boolean lTrack3 = false;
  boolean lTrack4 = false;

  // Sections of code to include or exclude
  boolean nTables = false; // Network Tables in Use
  boolean cServer = true; // Camera Server
  boolean jCam = false; // Jevois Camera
  boolean lTrack = true; // Line Tracker
  boolean tenDegrees = true; // 10 degrees of freedom
  boolean pneumatics = true; // Pneumatics System
  boolean limitSwitches = true; // limit switches

  // Pneumatics
  Compressor scottCompressor;
  DoubleSolenoid pneuVacuum;
  DoubleSolenoid pneuHatchPanelTop;
  DoubleSolenoid pneuHatchPanelBottom;
  boolean pneuEnabled = false;
  boolean pneuVaccumeIsOn = false;
  double vaccumeEndTime = 0;

  // climbing vars
  double stopClimbTime = 0;
  double startClimbDegree = 0;
  boolean climbInitialize = true;

  // Limit switches
  DigitalInput limitSwitchRearLift;
  DigitalInput limitSwitchFrontLift;
  // DigitalInput limitSwitchRearDrop;
  // DigitalInput limitSwitchIntakeUp;
  // DigitalInput limitSwitchIntakeDown;

  // Digital Limit
  
   double motorCurrents[][] = new double[3][10]; 
   double avgMotorCurrents[][] = new double[3][10];
   double maxMotorCurrentThreshold[] = new double[3];
   public static final int MOTOR_POGO = 0; 	//Rear Lifter
   public static final int MOTOR_FANGS = 1; //Front lifter 
   public static final int MOTOR_INTAKE_ARM = 2;	//Intake arm  
   public static final int MOTOR_TIMEOUT = 2000; //in ms
   
   int currentIndex = 0;
   int avgCurrentIndex = 0;
   boolean pogoEnabled = true;
   boolean fangsEnabled = true;
   boolean intakeArmEnabled = true;

  // Intake
  int cycles = 0;
  double continueFullSpeedUntil = 0;

  @Override
  public void robotInit() {

    robotTimer.start(); // Start the timer for IMU Calibration Safeguard.

    // Setup the joystick
    try {
      _joy1 = new Joystick(0);
    } catch (Exception ex) {
    }
    try {
      _joy2 = new Joystick(1);
    } catch (Exception ex) {
    }

    // Setup the Drive System
    _frontTLeftMotor = new WPI_VictorSPX(13);
    _frontTRightMotor = new WPI_VictorSPX(12);
    _rearTRightMotor = new WPI_VictorSPX(11);
    _rearTLeftMotor = new WPI_VictorSPX(10);
    // Invert all the motors, they're probably wired wrong
    _frontTLeftMotor.setInverted(true);
    _frontTRightMotor.setInverted(true);
    _rearTLeftMotor.setInverted(true);
    _rearTRightMotor.setInverted(true);
    _frontTLeftMotor.setNeutralMode(NeutralMode.Brake);
    _frontTRightMotor.setNeutralMode(NeutralMode.Brake);
    _rearTLeftMotor.setNeutralMode(NeutralMode.Brake);
    _rearTRightMotor.setNeutralMode(NeutralMode.Brake);
    _mDrive = new MecanumDrive(_frontTLeftMotor, _rearTLeftMotor, _frontTRightMotor, _rearTRightMotor);

    // Create front Lifter motors
    _frontLifterOne = new WPI_TalonSRX(23);
    _frontLifterTwo = new WPI_TalonSRX(22);
    frontLifterMotors = new SpeedControllerGroup(_frontLifterOne, _frontLifterTwo);

    // Create rear Lifter Motors
    _rearLifterMotor = new WPI_TalonSRX(20);

    // Create the Intake Motors
    _intakeLifterMotor = new WPI_TalonSRX(21);
    _intakeLowerMotor = new WPI_VictorSPX(31);
    _intakeUpperMotor = new WPI_VictorSPX(30);
    _intakeLowerMotor.setNeutralMode(NeutralMode.Brake);
    _intakeUpperMotor.setNeutralMode(NeutralMode.Brake);

    // Create the line tracker sensors
    if (lTrack) {
      try {
        lineTracker0 = new DigitalInput(0);
      } catch (Exception ex) {
      }
      try {
        lineTracker1 = new DigitalInput(1);
      } catch (Exception ex) {
      }
      try {
        lineTracker2 = new DigitalInput(2);
      } catch (Exception ex) {
      }
      try {
        lineTracker3 = new DigitalInput(3);
      } catch (Exception ex) {
      }
      try {
        lineTracker4 = new DigitalInput(4);
      } catch (Exception ex) {
      }
    }

    // Create the 10 Degrees of Freedom
    if (tenDegrees) {
      try {
        imu = new ADIS16448_IMU();
      } catch (Exception ex) {
        imu = new ADIS16448_IMU();
      }
      try {
        imu.reset();
        imu.calibrate();
      } catch (Exception ex) {
      }
    }

    // If our camera is onboard the roborio - This year it's not
    if (cServer) {
      try {
        camera = CameraServer.getInstance().startAutomaticCapture(0);
      } catch (Exception ex) {
      }
      if (camera == null) {
        camera = null; // This will prevent a "Warning" error during compilation
      }
    }

    // We might need network tables in our near future, but not yet
    if (nTables) {
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      table = inst.getTable("deepSpace");
      xEntry = table.getEntry("X");
      yEntry = table.getEntry("Y");
    }

    // Serial Port Logic for our JeVois Camera System
    if (jCam) {
      try {
        theThePort = new SerialPort(115200, Port.kUSB);
      } catch (Exception e) {
        // jCamString = e.toString();
      }
      int retval = 0;
      if (theThePort != null) {
        retval = theThePort.writeString("ping\n");
      }
      if (retval > 0) {
        SmartDashboard.putString("Error", "The the error: " + retval);
        SmartDashboard.putString("Error2", theThePort.readString());
      }
	  
	  //initilize current arrays
	  for(i=0;1<10;i++;)  {
		  motorCurrents[MOTOR_POGO][i] = 0;
		  motorCurrents[MOTOR_FANGS][i] = 0;
		  motorCurrents[MOTOR_INTAKE_ARM][i] = 0;
		  avgMotorCurrents[MOTOR_POGO][i] = 0;
		  avgMotorCurrents[MOTOR_FANGS][i] = 0;
		  avgMotorCurrents[MOTOR_INTAKE_ARM][i] = 0;
	  }
	  
    }

    // Create our Pneumatics controls
    if (pneumatics) {
      scottCompressor = new Compressor(0);
      scottCompressor.setClosedLoopControl(true);
      pneuVacuum = new DoubleSolenoid(4, 5);
      pneuVacuum.set(DoubleSolenoid.Value.kOff);
      pneuHatchPanelTop = new DoubleSolenoid(2, 3);
      pneuHatchPanelBottom = new DoubleSolenoid(0, 1);
      pneuHatchPanelTop.set(DoubleSolenoid.Value.kReverse);
      pneuHatchPanelBottom.set(DoubleSolenoid.Value.kReverse);
    }

    if (limitSwitches) {
      limitSwitchRearLift = new DigitalInput(5);
      limitSwitchFrontLift = new DigitalInput(6);
      // limitSwitchRearDrop = new DigitalInput(6);
      // limitSwitchIntakeUp = new DigitalInput(7);
      // limitSwitchIntakeDown = new DigitalInput(8);
    }
    climbInitialize = true;
  }
  
  private void digitalMotorLimits () {
	  //check values and turn on or off motors
	  for (i=0; i<10; i++;){
		  
	  }
	  
  }
  
  private double getAvgCurrent (motorIndex int) {
	  double totCurrent = 0;
	  for(int i = 0; i < 10; i++) {
		  totCurrent = motorCurrents[motorIndex][i] + totCurrent;
	  }
	  return (totCurrent/10);
  }
  
  private void chkMotorCurrents() {
	  double fangCurrent = Math.max(_frontLifterOne.getOutputCurrent() , _frontLifterOne.getOutputCurrent()); //greatest value since motors work in pair
	  double pogoCurrent = _rearLifterMotor.getOutputCurrent();
	  double armCurrent = _intakeLifterMotor.getOutputCurrent();
	  if (currentIndex < 10) {
		  motorCurrents[MOTOR_FANGS][currentIndex] = fangCurrent;
		  motorCurrents[MOTOR_POGO][currentIndex] = pogoCurrent;
		  motorCurrents[MOTOR_INTAKE_ARM][currentIndex] = armCurrent;
		  if(currentIndex = 9) {
			  currentIndex = 0;
			  avgMotorCurrents[MOTOR_FANGS][avgCurrentIndex] = getAvgCurrent(MOTOR_FANGS);
			  avgMotorCurrents[MOTOR_POGO][avgCurrentIndex] = getAvgCurrent(MOTOR_POGO);
			  avgMotorCurrents[MOTOR_INTAKE_ARM][avgCurrentIndex] = getAvgCurrent(MOTOR_INTAKE_ARM);
			  
			  if(avgCurrentIndex== 9) {
				  avgCurrentIndex =0;
			  } else {
				  avgCurrentIndex ++;
			  }
		  } else {
			  currentIndex++;
		  }
		  
	  }
  }

  @Override
  public void robotPeriodic() {

    // Perform a full IMU reset and calibration joy2 "Start" pressed
    // This might take up to 9 seconds
    if (_joy1.getRawButton(8) || _joy2.getRawButton(8)) {
      manualImuCalibration();
      imu.reset();
    }
    if (!didItAlready) {
      imuCalibration();
    }

    // Reset the imu when the "Y" yellow button is pressed
    if ((_joy1.getRawButton(3) || _joy2.getRawButton(3)) && tenDegrees) {
      imu.reset();
      imuIsWorkingCorrectly = true;
      SmartDashboard.putBoolean("IMU Working", imuIsWorkingCorrectly);
    }

    // limit switch display
    if (limitSwitches) {
      SmartDashboard.putBoolean("Rear Limit", limitSwitchRearLift.get());
    }
    // Show the needed data to the Smart Dashboard
    SmartDashboard.putNumber("zDegree", zDegree);
    SmartDashboard.putNumber("xDegree", xDegree);
    SmartDashboard.putNumber("yDegree", yDegree);
    SmartDashboard.putNumber("targetDegree", targetDegree);
    SmartDashboard.putBoolean("Line Tracker 0", lTrack0);
    SmartDashboard.putBoolean("Line Tracker 1", lTrack1);
    SmartDashboard.putBoolean("Line Tracker 2", lTrack2);
    SmartDashboard.putBoolean("Line Tracker 3", lTrack3);
    SmartDashboard.putBoolean("Line Tracker 4", lTrack4);
    SmartDashboard.putBoolean("IMU Working", imuIsWorkingCorrectly);
    SmartDashboard.putNumber("frontspeed", frontLiftSpeed);
    SmartDashboard.putNumber("rearspeed", rearLiftSpeed);
    SmartDashboard.putBoolean("pneu Enabled", pneuEnabled);

    // Turn Compresser on/off
    if (pneumatics) {
      if (_joy2.getRawButton(2)) {
        pneuVaccumeIsOn = true;
        pneuVacuum.set(DoubleSolenoid.Value.kForward);
        pneuHatchPanelTop.set(DoubleSolenoid.Value.kReverse);
        pneuHatchPanelBottom.set(DoubleSolenoid.Value.kReverse);
      } else if (pneuVaccumeIsOn) {
        pneuHatchPanelTop.set(DoubleSolenoid.Value.kForward);
        pneuHatchPanelBottom.set(DoubleSolenoid.Value.kForward);
        vaccumeEndTime = System.currentTimeMillis() + 4000;
        pneuVaccumeIsOn = false;
      } else {
        if (vaccumeEndTime == 0 || vaccumeEndTime <= System.currentTimeMillis()) {
          pneuVacuum.set(DoubleSolenoid.Value.kReverse);
        }
      }

      if (_joy2.getRawButton(1)) {
        pneuHatchPanelTop.set(DoubleSolenoid.Value.kReverse);
        pneuHatchPanelBottom.set(DoubleSolenoid.Value.kReverse);
        pneuVacuum.set(DoubleSolenoid.Value.kReverse);
      }
    }
  }

  // This, before match has begun, should go periodically until done once
  private void imuCalibration() {
    if (Timer.getMatchTime() > 0) {
      robotTimer.stop();
      didItAlready = true;
    } else if (robotTimer.get() > 300.0) // if 5+ mins have passed since power on
    {
      didItAlready = true;
      manualImuCalibration();
      robotTimer.stop();
    }
  }

  // Manually calibrate the IMU - Robot should be oriented away from Driver
  private void manualImuCalibration() {
    imuIsWorkingCorrectly = false;
    SmartDashboard.putBoolean("IMU Working", imuIsWorkingCorrectly);
    try {
      imu.reset();
      imu.calibrate();
    } catch (Exception e) {
    }
    imuIsWorkingCorrectly = true;
    SmartDashboard.putBoolean("IMU Working", imuIsWorkingCorrectly);
    xDegreeIterations = 0;
  }

  // Locate the nearest target angle for our line tracker
  public double getPOVDegree(double xDegree) {
    double retDoub = -1;
    int povVal2 = _joy2.getPOV(); // If driver indicate override use it
    if (povVal2 >= 0) {
      retDoub = povVal2;
    }
    return retDoub;
  }

  @Override
  public void autonomousInit() {
    runMode = RunningInMode.auton;
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void teleopInit() {
    runMode = RunningInMode.teleop;
  }

  @Override
  public void teleopPeriodic() {
    forwardMotion = -_joy1.getRawAxis(1);

    // Platform Climb Logic
    // This is our platform climb at the end
    if (_joy1.getRawButton(4)) { // Automated Climb
      // store starting climb angle
      if (climbInitialize) {
        startClimbDegree = Math.round(imu.getAngleY()) % 360;
        climbInitialize = false;
      }
      // get climb speeds
      rearLiftSpeed = frontLiftSpeed * 1.0;
      yDegree = Math.round(imu.getAngleY()) % 360;
      if (yDegree > startClimbDegree + 5 && yDegree < startClimbDegree + 180) {
        rearLiftSpeed = rearLiftSpeed + 0.4; // Increase Speed to rear if front is too fast
      } else if (yDegree < startClimbDegree - 5.0) {
        rearLiftSpeed = 0.0; // Turn off rear if front is too slow
      } else if (yDegree < startClimbDegree) {
        rearLiftSpeed = rearLiftSpeed - 0.25; // Decrease speed to rear if front is lagging slightly
      }
      if (rearLiftSpeed > maxRearLiftSpeed) {
        rearLiftSpeed = maxRearLiftSpeed; // Enforce max speed limits
      }
      if (frontLiftSpeed > maxFrontLiftSpeed) {
        frontLiftSpeed = maxFrontLiftSpeed; // Enforce max speed limits
      }
      // perform climb
      if (limitSwitches && !limitSwitchFrontLift.get()) {
        frontLifterMotors.set(frontLiftSpeed);
      } else if (!limitSwitches) {
        frontLifterMotors.set(frontLiftSpeed);
      }
      _rearLifterMotor.set(rearLiftSpeed * -1.0);
      if (forwardMotion < 0.5 && forwardMotion > -0.5) {
        forwardMotion = 0.2; // This should cause a slow forward wheel spin while climbing
      }
    } else if (_joy1.getRawButton(1)) { // Revers both Claw and Pogo
      frontLifterMotors.set(frontLiftSpeed * -1.0);
      _rearLifterMotor.set(rearLiftSpeed);
    } else {
      if (Math.abs(_joy1.getRawAxis(3)) > 0.1) {
        // deploy front lifter
        if (limitSwitches && !limitSwitchFrontLift.get()) {
          frontLifterMotors.set(_joy1.getRawAxis(3));
        }
        _rearLifterMotor.set(0.0);
      } else if (_joy1.getRawButton(6)) {
        // retract front lifter
        frontLifterMotors.set(-0.4);
        _rearLifterMotor.set(0.0);
        climbInitialize = true;
      } else {
        frontLifterMotors.set(0.0);
      }
      if (Math.abs(-_joy1.getRawAxis(2)) > 0.1) {
        // deploy rear lifter
        frontLifterMotors.set(0.0);
        _rearLifterMotor.set(_joy1.getRawAxis(2));
      } else if (_joy1.getRawButton(5)) {
        // retract rear lifter
        frontLifterMotors.set(0.0);
        if (limitSwitches && !limitSwitchRearLift.get()) {
          _rearLifterMotor.set(0.7);
        } else if (limitSwitches && limitSwitchRearLift.get()) {
          _rearLifterMotor.set(0);
        } else if (!limitSwitches) {
          _rearLifterMotor.set(0.3);
        }
        forwardMotion = 0.2;
        climbInitialize = true;
      } else {
        _rearLifterMotor.set(0.0);
      }
    }

    // Intake Logic Begins Here
    double lowerInTake = _joy2.getRawAxis(1);
    if (lowerInTake > 0.05 || lowerInTake < -0.05) {
      if (lowerInTake < 0) {
        _intakeLifterMotor.set(lowerInTake / 3); // Down
        cycles++;
      } else {
        if (cycles > 50) {
          _intakeLifterMotor.set(lowerInTake); // Full Speed Up
          continueFullSpeedUntil = System.currentTimeMillis() + 400;
        } else {
          if (System.currentTimeMillis() <= continueFullSpeedUntil) {
            _intakeLifterMotor.set(lowerInTake); // Continue Full Speed Up
          } else {
            _intakeLifterMotor.set(lowerInTake / 4); // Normal Up
          }
        }
        cycles = 0;
      }
    } else {
      _intakeLifterMotor.set(0); // Stop motion
    }

    // Intake logic to receive and give a ball
    double ballInTakeIn = _joy2.getRawAxis(2);
    double ballInTakeOut = _joy2.getRawAxis(3);
    if (ballInTakeIn > 0.05 || ballInTakeIn < -0.05) {
      _intakeLowerMotor.set(ballInTakeIn);
      _intakeUpperMotor.set(ballInTakeIn);
    } else if (ballInTakeOut > 0.05 || ballInTakeOut < -0.05) {
      _intakeLowerMotor.set(-ballInTakeOut / 2);
      _intakeUpperMotor.set(-ballInTakeOut / 2);
    } else if (_joy2.getRawButton(5)) {
      _intakeLowerMotor.set(1.0);
      _intakeUpperMotor.set(1.0);
    } else if (_joy2.getRawButton(6)) {
      _intakeLowerMotor.set(-0.5);
      _intakeUpperMotor.set(-0.5);
    } else {
      _intakeLowerMotor.set(0);
      _intakeUpperMotor.set(0);
    }

    // Setup Stafe values
    double strafe = 0;
    if (_joy1.getRawAxis(0) > 0.05 || _joy1.getRawAxis(0) < -0.05) {
      strafe = _joy1.getRawAxis(0) * 1.25;
      if (strafe > 1.0) {
        strafe = 1.0;
      }
    }
    turnRotation = _joy1.getRawAxis(4) * 0.4;
    if (tenDegrees) {
      zDegree = Math.round(imu.getAngleZ()) % 360;
      xDegree = Math.round(imu.getAngleX()) % 360;
      // Invert due to roborio orentation/position/location/placement
      if (xDegree <= 0) {
        xDegree = Math.abs(xDegree);
      } else if (xDegree == 360) {
        xDegree = 0;
      } else {
        xDegree = 360 - xDegree;
      }
      yDegree = Math.round(imu.getAngleY()) % 360;
      if (xDegree < 0) {
        xDegree += 360;
      }
      targetDegree = getPOVDegree(xDegree);
      if (lTrack) {
        try {
          lTrack0 = lineTracker0.get();
          lTrack1 = lineTracker1.get();
          lTrack2 = lineTracker2.get();
          lTrack3 = lineTracker3.get();
          lTrack4 = lineTracker4.get();
        } catch (Exception ex) {
          targetDegree = -1;
        }
      }
      if (targetDegree >= 0 && imuIsWorkingCorrectly) { // Line Tracker Enabled
        /*if (lTrack0) {
          turnRotation = turnRotation + turnSpeed(0.3);
          strafe += 0.6;
          _mDrive.driveCartesian(strafe, forwardMotion, turnRotation, 0);
        } else if (lTrack4) {
          turnRotation = turnRotation + turnSpeed(0.3);
          strafe -= 0.6;
          _mDrive.driveCartesian(strafe, forwardMotion, turnRotation, 0);
        } else if (lTrack1) {
          turnRotation = turnRotation + turnSpeed(0.2);
          strafe += 0.4;
          _mDrive.driveCartesian(strafe, forwardMotion, turnRotation, 0);
        } else if (lTrack3) {
          turnRotation = turnRotation + turnSpeed(0.2);
          strafe -= 0.4;
          _mDrive.driveCartesian(strafe, forwardMotion, turnRotation, 0);
        } else if (lTrack2) {
          turnRotation = turnRotation + turnSpeed(0.1);
          _mDrive.driveCartesian(0, forwardMotion, turnRotation, 0);
        } else {
          _mDrive.driveCartesian(strafe, forwardMotion, turnRotation, 0);
        }
      } else {
        _mDrive.driveCartesian(strafe, forwardMotion, turnRotation, 0);
      }
    } else {
      // The the mecanum drive is listed below
      _mDrive.driveCartesian(strafe, forwardMotion, turnRotation, 0);
    }*/
    if (lTrack0) {
      turnRotation = -(turnRotation + turnSpeed(0.3));
      strafe = strafe + 0.4;
      _mDrive.driveCartesian(strafe, forwardMotion, turnRotation, 0);
    } else if (lTrack4) {
      turnRotation = -(turnRotation + turnSpeed(0.3));
      strafe = strafe - 0.4;
      _mDrive.driveCartesian(strafe, forwardMotion, turnRotation, 0);
    } else if (lTrack1) {
      turnRotation = -(turnRotation + turnSpeed(0.2));
      strafe = strafe + 0.25;
      _mDrive.driveCartesian(strafe, forwardMotion, turnRotation, 0);
    } else if (lTrack3) {
      turnRotation = -(turnRotation + turnSpeed(0.2));
      strafe = strafe - 0.25;
      _mDrive.driveCartesian(strafe, forwardMotion, turnRotation, 0);
    } else if (lTrack2) {
      turnRotation = -(turnRotation + turnSpeed(0.1));
      _mDrive.driveCartesian(0, forwardMotion, turnRotation, 0);
    } else {
      _mDrive.driveCartesian(strafe, forwardMotion, turnRotation, 0);
    }
  } else {
    _mDrive.driveCartesian(strafe, forwardMotion, turnRotation, 0);
  }
} else {
  // The the mecanum drive is listed below
  _mDrive.driveCartesian(strafe, forwardMotion, turnRotation, 0);
}
    if (turnRotation < -0.2 || turnRotation > 0.2) {
      if (pastXDegree == xDegree) {
        xDegreeIterations++;
        if (xDegreeIterations > 15) {
          imuIsWorkingCorrectly = false; // We have a real Problem
        }
      } else {
        pastXDegree = xDegree;
        xDegreeIterations = 0;
      }
    }
  }

  public double turnSpeed(double fullspeed) {
    double returnSpeed = fullspeed;
    if (targetDegree == 0 && xDegree > 270) {
      returnSpeed = returnSpeed * -1.0; // we are overlapping zero
    }
    if (xDegree > targetDegree) {
      returnSpeed = returnSpeed * -1.0;
    }
    return returnSpeed;
  }

  @Override
  public void testPeriodic() {
    teleopPeriodic();
    runMode = RunningInMode.test;
  }
}