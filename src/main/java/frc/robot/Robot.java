package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  //private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  Compressor c;
  Solenoid dogShifter;
  Solenoid intakeSolenoid;
  TalonSRX LeftMaster = new TalonSRX(1);
  TalonSRX RightMaster = new TalonSRX(2);
  VictorSPX LeftSlave1 = new VictorSPX(5);
  VictorSPX LeftSlave2 = new VictorSPX(8);
  VictorSPX RightSlave1 = new VictorSPX(6);
  VictorSPX RightSlave2 = new VictorSPX(4);
  TalonSRX intakeMotor = new TalonSRX(3);
  TalonSRX lowerHopper = new TalonSRX(0);
  VictorSPX upperHopper = new VictorSPX(7);
  double horizontalOffset = 0;
  VictorSPX feed = new VictorSPX(2);
  CANSparkMax leftNeo = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax rightNeo = new CANSparkMax(2, MotorType.kBrushless);
  SparkMaxPIDController leftPID = leftNeo.getPIDController();
  RelativeEncoder leftNeoEncoder = leftNeo.getEncoder();
  RelativeEncoder rightNeoEncoder = rightNeo.getEncoder();
 
  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  TalonSRX hoodMotor = new TalonSRX(9);
  double hoodPosition = hoodMotor.getSelectedSensorPosition();
  double hopperSpeed = 0.6;
  double intakeSpeed = 0.5;
  XboxController xbox;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry tv = table.getEntry("tv");
Encoder leftEncoder, rightEncoder;
double targetOffsetAngle_Vertical = ty.getDouble(0.0);

// how many degrees back is your limelight rotated from perfectly vertical?
double limelightMountAngleDegrees = 45.0;

// distance from the center of the Limelight lens to the floor
double limelightLensHeightInches = 39.0;

// distance from the target to the floor
double goalHeightInches = 92.0;
PIDController limeController = new PIDController(.1, .00, .01 );
PIDController hPidController = new PIDController(0.0008, 0, 0);
PIDController autoPID;
double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
double leftMax = 0;
double rightMax = 0;
double feetPerSecond = 0;
double hoodTarget = 5;
double targetRPM = 0;
ShuffleboardTab tab = Shuffleboard.getTab("BB");
Joystick js = new Joystick(0);
double setPoint = 9000;
double[] LeftEncoderTarget, RightEncoderTarget;
SlewRateLimiter filter = new SlewRateLimiter(0.5);
int step;
double[][] lookup = {
  {0, 23,2150},
  {43, 23, 2150},
  {68.0,30,2450},
  {80.0, 32, 2450},
  {90.0, 33, 2525},
  {100.5, 35, 2700},
  {119.5, 37, 2750},
  {129.5, 41, 2850},
  {141.5, 42, 2900},
  {153, 43, 3000},
  {162, 45, 3100},
  {172, 45, 3200},
  {182, 45, 3250},
  {192, 46, 3350},
  {210, 47, 3450},
  {243, 50, 3800},
  {300, 50, 3800}};
int index = 0;
Timer autonTimer;
//calculate distance
double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

  @Override
  public void robotInit() {
    
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    c = new Compressor(1, PneumaticsModuleType.CTREPCM);
    xbox = new XboxController(1);
    rightEncoder = new Encoder(0,1 , true,EncodingType.k4X);
    leftEncoder = new Encoder(8,9, false, EncodingType.k4X);
    dogShifter = new Solenoid(1, PneumaticsModuleType.CTREPCM, 0);
    intakeSolenoid = new Solenoid(1, PneumaticsModuleType.CTREPCM, 1);
    LeftSlave1.follow(LeftMaster);
    LeftSlave2.follow(LeftMaster);
    RightSlave1.follow(RightMaster);
    RightSlave2.follow(RightMaster);
    LeftMaster.setNeutralMode(NeutralMode.Brake);
    RightMaster.setNeutralMode(NeutralMode.Brake);
    RightMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
    LeftMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
    LeftMaster.setInverted(false);
    upperHopper.setInverted(true);
    lowerHopper.setInverted(true);
    intakeMotor.setInverted(false);
    leftNeo.setIdleMode(IdleMode.kCoast);
    rightNeo.setIdleMode(IdleMode.kCoast);
    rightNeo.follow(leftNeo, true);
    kP = 0.000000;
    kI = 0.00000002;
    kD = 0.000000;
    kIz = 100;
    kFF = 0.000265;
    kMaxOutput = 1;
    kMinOutput = 0;
    maxRPM = 5700;
    leftPID.setP(kP);
    leftPID.setI(kI);
    leftPID.setD(kD);
    leftPID.setIZone(kIz);
    leftPID.setFF(kFF);
    leftPID.setOutputRange(kMinOutput, kMaxOutput);
    leftNeo.enableVoltageCompensation(9);
    rightNeo.enableVoltageCompensation(9);
    leftNeo.burnFlash();
    rightNeo.burnFlash();
    UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
    mjpegServer1.setSource(usbCamera);
    CvSink cvSink = new CvSink("opencv_USB Camera 0");
    cvSink.setSource(usbCamera);

    // Creates the CvSource and MjpegServer [2] and connects them
    CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
    mjpegServer2.setSource(outputStream);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("drivetrain solenoid", dogShifter.get());
    SmartDashboard.putNumber("HOOD TARGET", hoodTarget);
    SmartDashboard.putNumber("hood angle", hoodMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Target", tv.getDouble(0)==1);
    SmartDashboard.putNumber("FLYWHEEL TARGET", targetRPM);
    SmartDashboard.putBoolean("Intake", intakeSolenoid.get());
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putBoolean("Conpressor On", c.enabled());
    SmartDashboard.putNumber("left", leftEncoder.get());
    SmartDashboard.putNumber("right", rightEncoder.get());
    SmartDashboard.putNumber("distance", distanceFromLimelightToGoalInches);
  }
  @Override
  public void autonomousInit() {
    LeftEncoderTarget = new double[10];
    RightEncoderTarget = new double[10];
    rightEncoder.setReverseDirection(false);
    leftEncoder.setReverseDirection(true);
    rightEncoder.setDistancePerPulse(6*Math.PI/2048);
    leftEncoder.setDistancePerPulse(6*Math.PI/2048);
    LeftEncoderTarget[0] = leftEncoder.getDistance();
    RightEncoderTarget[0] = rightEncoder.getDistance();
    LeftEncoderTarget[1] = LeftEncoderTarget[0] - 36;
    //RightEncoderTarget[1] = RightEncoderTarget[0] - 4900;
    LeftEncoderTarget[2] = LeftEncoderTarget[1] + 36;
    LeftEncoderTarget[3] = LeftEncoderTarget[2] -13*3.14;
    //RightEncoderTarget[2] = RightEncoderTarget[1] + 5800;
    step = 0;
    autoPID = new PIDController(.05, 0, 0);
    autonTimer = new Timer();
    autonTimer.start();
  }
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("step", step);
    if(step==0){
      intakeSolenoid.toggle();
      step++;
    }
    if(autonTimer.get()<5){
      double rpm = leftNeo.getEncoder().getVelocity();
      SmartDashboard.putNumber("Actual RPM", rpm);
      feetPerSecond = SmartDashboard.getNumber("FLYWHEEL TARGET", 0);
      if(tv.getDouble(0)==1){
         horizontalOffset= tx.getDouble(0);
        if(horizontalOffset <=2 && horizontalOffset >= -2){
          LeftMaster.set(ControlMode.PercentOutput, 0);
          RightMaster.set(ControlMode.PercentOutput, 0);
        }else{
          LeftMaster.set(ControlMode.PercentOutput, -1*limeController.calculate(horizontalOffset, 0));
          RightMaster.set(ControlMode.PercentOutput, -1*limeController.calculate(horizontalOffset, 0));
        } //hoodMotor.set(ControlMode.PercentOutput, 0);
          //leftNeo.set(0);
        //}
          //LeftMaster.set(ControlMode.PercentOutput, 0);
          //RightMaster.set(ControlMode.PercentOutput,0);
          ty = table.getEntry("ty");
          double verticalOffset = ty.getDouble(0.0);
          double angle = (verticalOffset+40)*(3.14159/180);
          double distanceFromLimelightToGoalInches = (104-29.5)/Math.tan(angle);
          SmartDashboard.putNumber("distance", distanceFromLimelightToGoalInches);
          //hoodTarget = distanceToHoodAngle(distanceFromLimelightToGoalInches);
          double[] arr = distanceToHoodAngle(distanceFromLimelightToGoalInches);
          hoodTarget = arr[0];
          targetRPM = arr[1];
          if((hoodTarget-1)<=hoodReadingtoDegrees(hoodMotor.getSelectedSensorPosition()) && (hoodTarget+1)>=hoodReadingtoDegrees(hoodMotor.getSelectedSensorPosition()) && Math.abs(horizontalOffset) < 2){
            SmartDashboard.putBoolean("hood +rpm", true);
            feed.set(ControlMode.PercentOutput, .5);
            lowerHopper.set(ControlMode.PercentOutput, hopperSpeed);
            upperHopper.set(ControlMode.PercentOutput, hopperSpeed);
          }else{
            SmartDashboard.putBoolean("hood +rpm", false);
            feed.set(ControlMode.PercentOutput, 0);
            upperHopper.set(ControlMode.PercentOutput, 0);
           lowerHopper.set(ControlMode.PercentOutput, 0);
          }
          SmartDashboard.putNumber("setpoint", setPoint);
          leftPID.setReference(targetRPM, ControlType.kVelocity);
          SmartDashboard.putNumber("voltage", leftNeo.getAppliedOutput());
          //leftPID.setReference(5, ControlType.kVoltage);
          if(degreesToHoodReading(hoodTarget)-15>hoodMotor.getSelectedSensorPosition() || degreesToHoodReading(hoodTarget)+15<hoodMotor.getSelectedSensorPosition()){
            hoodMotor.set(ControlMode.PercentOutput, hPidController.calculate(hoodMotor.getSelectedSensorPosition(), degreesToHoodReading(hoodTarget)));
          }else{
            hoodMotor.set(ControlMode.PercentOutput, 0);
          }
        
      }
    }
    if(step==1 && autonTimer.get()>=5){
      //step++;
      leftNeo.set(0);
      feed.set(ControlMode.PercentOutput, 0);
      hoodMotor.set(ControlMode.PercentOutput, 0);
      upperHopper.set(ControlMode.PercentOutput, 0);
      lowerHopper.set(ControlMode.PercentOutput, 0);
    }
    if(step==2){
      //intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
      LeftMaster.set(ControlMode.PercentOutput,  -.45 * autoPID.calculate(leftEncoder.getDistance()-LeftEncoderTarget[0], LeftEncoderTarget[1]-LeftEncoderTarget[0]));
      RightMaster.set(ControlMode.PercentOutput, -.5 * autoPID.calculate(leftEncoder.getDistance()-LeftEncoderTarget[0], LeftEncoderTarget[1]-LeftEncoderTarget[0]));
    }
    if(step > 1 && step < 3){
      intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
      lowerHopper.set(ControlMode.PercentOutput, hopperSpeed);
    }
    if(step ==3){
      LeftMaster.set(ControlMode.PercentOutput,  -.45 * autoPID.calculate(leftEncoder.getDistance()-LeftEncoderTarget[1], LeftEncoderTarget[2]-LeftEncoderTarget[1]));
      RightMaster.set(ControlMode.PercentOutput, .5 * autoPID.calculate(leftEncoder.getDistance()-LeftEncoderTarget[1], LeftEncoderTarget[2]-LeftEncoderTarget[1]));
    }
    if( Math.abs(leftEncoder.getDistance()-LeftEncoderTarget[1])<=1 && step == 2){
      step++;
    }
    if(Math.abs(leftEncoder.getDistance()-LeftEncoderTarget[2])<=1 && step == 3){
      step++;
    }
    if(Math.abs(leftEncoder.getDistance()-LeftEncoderTarget[3])<=1 && step == 4){
      step++;
    }
    if(step == 3){
      LeftMaster.set(ControlMode.PercentOutput,  -.45 * autoPID.calculate(leftEncoder.getDistance()-LeftEncoderTarget[2], LeftEncoderTarget[3]-LeftEncoderTarget[2]));
      RightMaster.set(ControlMode.PercentOutput, -.5 * autoPID.calculate(leftEncoder.getDistance()-LeftEncoderTarget[2], LeftEncoderTarget[3]-LeftEncoderTarget[2]));
      
    }
    if(step==4){
      if(tv.getDouble(0)==1){
        horizontalOffset= tx.getDouble(0);
       if(horizontalOffset <=2 && horizontalOffset >= -2){
         LeftMaster.set(ControlMode.PercentOutput, 0);
         RightMaster.set(ControlMode.PercentOutput, 0);
       }else{
         LeftMaster.set(ControlMode.PercentOutput, -1*limeController.calculate(horizontalOffset, 0));
         RightMaster.set(ControlMode.PercentOutput, -1*limeController.calculate(horizontalOffset, 0));
       } //hoodMotor.set(ControlMode.PercentOutput, 0);
         //leftNeo.set(0);
       //}
         //LeftMaster.set(ControlMode.PercentOutput, 0);
         //RightMaster.set(ControlMode.PercentOutput,0);
         ty = table.getEntry("ty");
         double verticalOffset = ty.getDouble(0.0);
         double angle = (verticalOffset+40)*(3.14159/180);
         double distanceFromLimelightToGoalInches = (104-29.5)/Math.tan(angle);
         SmartDashboard.putNumber("distance", distanceFromLimelightToGoalInches);
         //hoodTarget = distanceToHoodAngle(distanceFromLimelightToGoalInches);
         double[] arr = distanceToHoodAngle(distanceFromLimelightToGoalInches);
         hoodTarget = arr[0];
         targetRPM = arr[1];
         if((hoodTarget-1)<=hoodReadingtoDegrees(hoodMotor.getSelectedSensorPosition()) && (hoodTarget+1)>=hoodReadingtoDegrees(hoodMotor.getSelectedSensorPosition()) && Math.abs(horizontalOffset) < 2){
           SmartDashboard.putBoolean("hood +rpm", true);
           feed.set(ControlMode.PercentOutput, .5);
           upperHopper.set(ControlMode.PercentOutput, hopperSpeed);
           lowerHopper.set(ControlMode.PercentOutput, hopperSpeed);
         }else{
           SmartDashboard.putBoolean("hood +rpm", false);
           feed.set(ControlMode.PercentOutput, 0);
           upperHopper.set(ControlMode.PercentOutput, 0);
           lowerHopper.set(ControlMode.PercentOutput, 0);
         }
         SmartDashboard.putNumber("setpoint", setPoint);
         leftPID.setReference(targetRPM, ControlType.kVelocity);
         SmartDashboard.putNumber("voltage", leftNeo.getAppliedOutput());
         //leftPID.setReference(5, ControlType.kVoltage);
         if(degreesToHoodReading(hoodTarget)-15>hoodMotor.getSelectedSensorPosition() || degreesToHoodReading(hoodTarget)+15<hoodMotor.getSelectedSensorPosition()){
           hoodMotor.set(ControlMode.PercentOutput, hPidController.calculate(hoodMotor.getSelectedSensorPosition(), degreesToHoodReading(hoodTarget)));
         }else{
           hoodMotor.set(ControlMode.PercentOutput, 0);
         }
       
     }
    }
  }
  @Override
  public void teleopInit() {
    
  }
  @Override
  public void teleopPeriodic() {
    if(js.getTriggerPressed() && !intakeSolenoid.get()){
      intakeSolenoid.toggle();
    } else if(js.getRawButtonPressed(2) && intakeSolenoid.get()){
      intakeSolenoid.toggle();
    }
    if(xbox.getLeftBumperPressed() && !dogShifter.get()){
      dogShifter.toggle();
    }else if(xbox.getRightBumperPressed() && dogShifter.get()){
      dogShifter.toggle();
    }
    if(js.getRawButton(3)){
      lowerHopper.set(ControlMode.PercentOutput, hopperSpeed);
      upperHopper.set(ControlMode.PercentOutput, hopperSpeed);
    }else if(js.getRawButton(5)){
      lowerHopper.set(ControlMode.PercentOutput, -hopperSpeed);
      upperHopper.set(ControlMode.PercentOutput, -hopperSpeed);
    }else if(!(xbox.getRawAxis(3)>0)){
      lowerHopper.set(ControlMode.PercentOutput, 0);
      upperHopper.set(ControlMode.PercentOutput, 0);
    }
    if(js.getRawButton(4)){
      intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
    }else if(js.getRawButton(6)){
      intakeMotor.set(ControlMode.PercentOutput, -intakeSpeed);
    }
    else{
      intakeMotor.set(ControlMode.PercentOutput, 0);
    }
    if(js.getRawButtonPressed(7)){
      targetRPM+=50;
    }
    else if(js.getRawButtonPressed(8)){
      targetRPM-=50;
    }
    if(js.getRawButtonPressed(9)){
      distanceFromLimelightToGoalInches += 1;
    }else if(js.getRawButtonPressed(10)){
      distanceFromLimelightToGoalInches -= 1;
    }
    if(js.getRawButton(12)){
      feed.set(ControlMode.PercentOutput, 0.5);
    }else if(js.getRawButton(11)){
      feed.set(ControlMode.PercentOutput, -.5);
    }
    else{
      feed.set(ControlMode.PercentOutput, 0);
    }
    if(xbox.getXButtonPressed()){
      c.disable();
    }
    if(xbox.getYButtonPressed()){
      c.enableAnalog(100, 120);
      c.enableDigital();
    }
    if(xbox.getStartButton()){
      ty = table.getEntry("ty");
      double verticalOffset = ty.getDouble(0.0);
      double angle = (verticalOffset+50)*(3.14159/180);
      double distanceFromLimelightToGoalInches = (104-29.5)/Math.tan(angle);
      SmartDashboard.putNumber("distance", distanceFromLimelightToGoalInches);
    }
    if(xbox.getRawAxis(3)> 0){
      double rpm = leftNeo.getEncoder().getVelocity();
      SmartDashboard.putNumber("Actual RPM", rpm);
      feetPerSecond = SmartDashboard.getNumber("FLYWHEEL TARGET", 0);
      if(tv.getDouble(0)==1){
         horizontalOffset= tx.getDouble(0);
        if(horizontalOffset <=2 && horizontalOffset >= -2){
          LeftMaster.set(ControlMode.PercentOutput, 0);
          RightMaster.set(ControlMode.PercentOutput, 0);
        }else{
          LeftMaster.set(ControlMode.PercentOutput, -1*limeController.calculate(horizontalOffset, 0));
          RightMaster.set(ControlMode.PercentOutput, -1*limeController.calculate(horizontalOffset, 0));
        } //hoodMotor.set(ControlMode.PercentOutput, 0);
          //leftNeo.set(0);
        //}
          //LeftMaster.set(ControlMode.PercentOutput, 0);
          //RightMaster.set(ControlMode.PercentOutput,0);
          ty = table.getEntry("ty");
          double verticalOffset = ty.getDouble(0.0);
          double angle = (verticalOffset+40)*(3.14159/180);
          double distanceFromLimelightToGoalInches = (104-29.5)/Math.tan(angle);
          SmartDashboard.putNumber("distance", distanceFromLimelightToGoalInches);
          //hoodTarget = distanceToHoodAngle(distanceFromLimelightToGoalInches);
          double[] arr = distanceToHoodAngle(distanceFromLimelightToGoalInches);
          hoodTarget = arr[0];
          targetRPM = arr[1];
          if((hoodTarget-1)<=hoodReadingtoDegrees(hoodMotor.getSelectedSensorPosition()) && (hoodTarget+1)>=hoodReadingtoDegrees(hoodMotor.getSelectedSensorPosition()) && Math.abs(horizontalOffset) < 2){
            SmartDashboard.putBoolean("hood +rpm", true);
            //feed.set(ControlMode.PercentOutput, .5);
          }else{
            SmartDashboard.putBoolean("hood +rpm", false);
            //feed.set(ControlMode.PercentOutput, 0);
          }
          SmartDashboard.putNumber("setpoint", setPoint);
          leftPID.setReference(targetRPM, ControlType.kVelocity);
          SmartDashboard.putNumber("voltage", leftNeo.getAppliedOutput());
          //leftPID.setReference(5, ControlType.kVoltage);
          if(degreesToHoodReading(hoodTarget)-15>hoodMotor.getSelectedSensorPosition() || degreesToHoodReading(hoodTarget)+15<hoodMotor.getSelectedSensorPosition()){
            hoodMotor.set(ControlMode.PercentOutput, hPidController.calculate(hoodMotor.getSelectedSensorPosition(), degreesToHoodReading(hoodTarget)));
          }else{
            hoodMotor.set(ControlMode.PercentOutput, 0);
          }
        
      }
    }
    else{
      leftNeo.set(0);
      feed.set(ControlMode.PercentOutput, 0);
      hoodMotor.set(ControlMode.PercentOutput, 0);
      //lowerHopper.set(ControlMode.PercentOutput, 0);
      //upperHopper.set(ControlMode.PercentOutput, 0);  
        double speed, turn;
         if(xbox.getRawAxis(1) >= .1 || xbox.getRawAxis(1) <= -.1){
          speed = xbox.getRawAxis(1) * .65;
          //speed = (xbox.getRawAxis(1) < 0)?(speed * -1):speed;
          turn = -.25 * (xbox.getRawAxis(4)-.1);
          LeftMaster.set(ControlMode.PercentOutput, -(speed+turn));
          RightMaster.set(ControlMode.PercentOutput, ((speed-turn)));
         }else if(xbox.getRawAxis(4) >= .1 || xbox.getRawAxis(4) <= -.1){
          turn = (xbox.getRawAxis(4)-.1) * .65;
          //turn = (xbox.getRawAxis(4) < 0)?(turn * -1):turn;
          LeftMaster.set(ControlMode.PercentOutput, (turn));
          RightMaster.set(ControlMode.PercentOutput, (turn));
         }
         else{
           LeftMaster.set(ControlMode.PercentOutput, 0);
           RightMaster.set(ControlMode.PercentOutput, 0);
         }
    } 
  }
  @Override
  public void disabledInit() {
  }
  @Override
  public void disabledPeriodic() {}
  @Override
  public void testInit() {}
  @Override
  public void testPeriodic() {}
  @Override
  public void simulationInit() {}
  @Override
  public void simulationPeriodic() {}
  public double FPSTORPM(double fps){
    return ((fps * 60 * 12)/3.14159)/4;
  }
  public double hoodReadingtoDegrees(double hoodReading){
    return 5d+((hoodReading/7672d)*45d);
  }
  public double degreesToHoodReading(double degrees){
    if(degrees<5){
      return 0;
    }
    return (degrees-5d)*7672d/45d;
  }
  // public double distanceToRPM(double distance){
  //   distance/=12;
  //   if(distance<6.5){
  //     return 1000*(.25*(distance-6)+3.35);
  //   }else if(distance <= 8.5){
  //     return 1000*(.145*(distance-8)+3.27);
  //   }
  //   else if(distance <= 10.5){
  //     return 1000*(.15*(distance-9)+3.25);
  //   }
  //   else if(distance <= 13.5){
  //     return 1000*(.125*(distance-13)+3.65);
  //   }
  //   else{
  //     return 1000*(.21*(distance-14)+3.55);
  //   }
  // }
  public double[] distanceToHoodAngle(double distance){
    //distance/=12;
    int index = lookup.length/2;
    for(int i = 0; i < lookup.length;i++){
      if(distance>=lookup[i][0]&&distance<=lookup[i+1][0]){
        index = i;
        break;
      }
    }
    double rpm = (lookup[index+1][2]-lookup[index][2])/(lookup[index+1][0]-lookup[index][0])*(distance-lookup[index][0])+(lookup[index][2]);
    double hoodAngle = (lookup[index+1][1]-lookup[index][1])/(lookup[index+1][0]-lookup[index][0])*(distance-lookup[index][0])+(lookup[index][1]);
    return new double[] {hoodAngle, rpm};
  }
  public boolean isWithin (double target, double value, double range){
    return ((target-range) <= value) && ((target+range) >= value);
  }
}
