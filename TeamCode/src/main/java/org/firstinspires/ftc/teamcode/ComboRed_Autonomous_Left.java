package org.firstinspires.ftc.teamcode;
/*
Made By Cameron Doughty 1/15/2018
Added to by dcole, 1-23/2018
Added to by dcole, 1-2/2018
*/

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="ComboRed_Autonomous_Left")
public class ComboRed_Autonomous_Left extends LinearOpMode {   //Linear op mode is being used so the program does not get stuck in loop()
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    public DcMotor FleftDrive = null;
    public DcMotor FrightDrive = null;
    public DcMotor BleftDrive = null;
    public DcMotor BrightDrive = null;
    private Servo RelicServo = null;
    private Servo Blockservo_left = null;
    private Servo Blockservo_right = null;
    public Servo Color_sensor_servo = null;
    ModernRoboticsI2cColorSensor Color_sensor = null;
    public double speed = .5;
    int zAccumulated;  //Total rotation left/right
    int target = 0;  //Desired angle to turn to
    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()

    // Define class members
    DeviceInterfaceModule   Sensors;                //
    DigitalChannel          eyeBall;                // Device Object photocell



    boolean Relicarmon;


    void stopDrive() {
        FleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
    }
    void forward(long time) {
        FleftDrive.setPower(-speed);
        FrightDrive.setPower(-speed);
        BleftDrive.setPower(-speed);
        BrightDrive.setPower(-speed);
        sleep(time);
    }
    void reverse(long time) {
        FleftDrive.setPower(speed);
        FrightDrive.setPower(speed);
        BleftDrive.setPower(speed);
        BrightDrive.setPower(speed);
        sleep(time);
    }
    void strafeleft(long time) {
        FleftDrive.setPower(speed);
        FrightDrive.setPower(-speed);
        BleftDrive.setPower(-speed);
        BrightDrive.setPower(speed);
        sleep(time);
    }
    void straferight(long time) {
        FleftDrive.setPower(-speed);
        FrightDrive.setPower(speed);
        BleftDrive.setPower(speed);
        BrightDrive.setPower(-speed);
        sleep(time);
    }
    void TurnLeft(long time) {
        FleftDrive.setPower(speed);
        FrightDrive.setPower(-speed);
        BleftDrive.setPower(speed);
        BrightDrive.setPower(-speed);
        sleep(time);
    }
    void TurnRight(long time) {
        FleftDrive.setPower(-speed);
        FrightDrive.setPower(speed);
        BleftDrive.setPower(-speed);
        BrightDrive.setPower(speed);
        sleep(time);
    }

    @Override
    public void runOpMode() {    //-------------------  runOpMode  ---------------
        telemetry.addData("Status", "Initialized");
        FleftDrive = hardwareMap.get(DcMotor.class, "FleftDrive");
        FrightDrive = hardwareMap.get(DcMotor.class, "FrightDrive");
        BleftDrive = hardwareMap.get(DcMotor.class, "BleftDrive");
        BrightDrive = hardwareMap.get(DcMotor.class, "BrightDrive");
        RelicServo = hardwareMap.get(Servo.class, "RelicServo");
        Blockservo_left = hardwareMap.get(Servo.class, "Blockservo_left");
        Blockservo_right = hardwareMap.get(Servo.class, "Blockservo_right");
        Color_sensor_servo = hardwareMap.get(Servo.class, "Color_sensor_servo");
        Color_sensor = (ModernRoboticsI2cColorSensor) hardwareMap.get(ColorSensor.class, "Color_sensor");
        Color_sensor.enableLight(true);
        FrightDrive.setDirection(DcMotor.Direction.REVERSE);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);
        sensorGyro = hardwareMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;      //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        mrGyro.calibrate();  //Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        // setup for the photocell
        Sensors = hardwareMap.get(DeviceInterfaceModule.class, "Sensors");
        eyeBall = hardwareMap.get(DigitalChannel.class, "eyeBall");     //  Use generic form of device mapping
        eyeBall.setMode(DigitalChannel.Mode.INPUT);      // Set the direction of this channel

        while (!isStopRequested() && mrGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();
        waitForStart();
        runtime.reset();
        Color_sensor.red();   // Red channel value
        Color_sensor.green(); // Green channel value
        Color_sensor.blue();  // Blue channel value
        Color_sensor.alpha(); // Total luminosity
        Color_sensor.argb();  // Combined color value
        Color_sensor.enableLight(true);
        int loopCount = 0;
        while(opModeIsActive()){
            telemetry.update();
            Color_sensor.enableLight(true);
            Blockservo_left.setPosition(.5);
            Blockservo_right.setPosition(.5);
            RelicServo.setPosition(0.5);
//----------------gyro way points-----------------------------------

            runtime.reset();
            straightAbsolute(3,10,.5);
            allStop();
            eyeScan();
            stop();

//-----------------------------------------------------------------
        }
    }
    // end of opMode

    //This function turns a number of degrees compared to where the robot is. Positive numbers trn left.
    public void turn(int target) throws InterruptedException {
        turnAbsolute(target + mrGyro.getIntegratedZValue());
    }
    //This function turns a number of degrees compared to where the robot was when the program started. Positive numbers trn left.
    public void turnAbsolute(int target){
        zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
        double turnSpeed = 0.4;
        while (Math.abs(zAccumulated - target) > 3 && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
            telemetry.addData("Target", target);
            if (zAccumulated < target) {  //if gyro is positive, we will turn right
                FleftDrive.setPower(turnSpeed);
                BleftDrive.setPower(turnSpeed);
                FrightDrive.setPower(-turnSpeed);
                BrightDrive.setPower(-turnSpeed);
            }
            if (zAccumulated > target) {  //if gyro is positive, we will turn left
                FleftDrive.setPower(-turnSpeed);
                BleftDrive.setPower(-turnSpeed);
                FrightDrive.setPower(turnSpeed);
                BrightDrive.setPower(turnSpeed);
            }
            zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
        }
       allStop();
    }



    /*target is the desired heading
      drivetime is the driving time in mill
      speed while going straight
      */

    public void allStop() {
        FleftDrive.setPower(0);    // all stop
        BleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BrightDrive.setPower(0);

    }

    public void straightAbsolute(int target, long driveTime, double straightSpeed) {
    // target = deviation
        double knudge = .1;              //coarse correction value
        zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
        straightSpeed = - straightSpeed;     //  straightSpeed seems to be inverted??
        FleftDrive.setPower(straightSpeed);
        BleftDrive.setPower(straightSpeed);
        FrightDrive.setPower(straightSpeed);
        BrightDrive.setPower(straightSpeed);

        // runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < driveTime)) {

           while (Math.abs(zAccumulated) > target && opModeIsActive()) {

               if (zAccumulated < target) {  //if gyro is positive, we will drift right
                   FleftDrive.setPower(straightSpeed + knudge);
                   BleftDrive.setPower(straightSpeed +knudge);
                   FrightDrive.setPower(straightSpeed - knudge);
                   BrightDrive.setPower(straightSpeed - knudge);
               }
               if (zAccumulated  > target) {  //if gyro is positive, we will drift left
                   FleftDrive.setPower(straightSpeed + knudge);
                   BleftDrive.setPower(straightSpeed + knudge);
                   FrightDrive.setPower(straightSpeed - knudge);
                   BrightDrive.setPower(straightSpeed - knudge);
               }

               zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
               telemetry.addData("accu", String.format("%03d", zAccumulated));
               telemetry.update();
           }
       }
       allStop();
    }

    public void Jewel(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        telemetry.update();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            telemetry.update();
            if (Color_sensor.blue() >= 1) {
                target = target + 20;
                turnAbsolute(target);
                sleep(500);
                Color_sensor_servo.setPosition(0.83);
                target = target - 20;
                turnAbsolute(target);
            } else if (Color_sensor.red() >= 1) {
                target = target - 20;
                turnAbsolute(target);
                sleep(500);
                Color_sensor_servo.setPosition(0.83);
                target = target + 20;
                turnAbsolute(target);
            }
        }
    }
    public void Armdown(double holdTime) {
        //CameraDevice.getInstance().setFlashTorchMode(false);
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            Color_sensor_servo.setPosition(0);
        }
    }
   /* public void PictoGraph(double holdTime) {
        //CameraDevice.getInstance().setFlashTorchMode(true);
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        Color_sensor_servo.setPosition(1);
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            VuforiaTest test = new VuforiaTest();
            VuforiaTest.type type = test.runOpMode(hardwareMap);
            if (type == VuforiaTest.type.LEFT) {
                Armdown(2);
                Jewel(0.5);
                Glyph(2, 1);
            }
            if (type == VuforiaTest.type.CENTER) {
                Armdown(2);
                Jewel(0.5);
                Glyph(2, 2);
            }
            if (type == VuforiaTest.type.RIGHT) {
                Armdown(2);
                Jewel(0.5);
                Glyph(2, 3);
            }
            if (type == VuforiaTest.type.ERROR) {
                Armdown(2);
                Jewel(0.5);
                Glyph(2, 2);
            }
        }
    }  */
    public void Glyph(double holdTime, int Direction) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        Color_sensor_servo.setPosition(1);
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            if (Direction == 1) {
                forward(1500);
                target = target - 85;
                turnAbsolute(target);
                strafeleft(1500);
            }
            if (Direction == 2) {
                forward(1500);
                target = target - 85;
                turnAbsolute(target);
                strafeleft(1000);
            }
            if (Direction == 3) {
                forward(1500);
                target = target - 85;
                turnAbsolute(target);
                strafeleft(500);
            }
        }
    }

    //  photocell method
    public boolean eyeScan() {

        // Input State
        boolean eye = eyeBall.getState();    //  Read the input pin 2, slot 3  (starts with 0)
        telemetry.addData("I see ", eye);
        telemetry.update();
        return eye;
    }
}
