package org.firstinspires.ftc.teamcode;
/*
Made By Cameron Doughty 1/15/2018
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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="NearBlueAuto")
public class NearBlueAuto extends LinearOpMode {   //Linear op mode is being used so the program does not get stuck in loop()
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
    private DcMotor RelicMotor = null;
    private DcMotor BlockMotor = null;
    public double speed = .25;
    int zAccumulated;  //Total rotation left/right
    int target = 0;  //Desired angle to turn to
    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
    boolean random = true;
    boolean Relicarmon;

    // stuff for the eyeballs and LED's
    static final int BLUE_LED = 0;     // Blue LED channel on DIM
    static final int RED_LED = 1;     // Red LED Channel on DIM
    int slot = 0;

    // Define class members
    DeviceInterfaceModule Sensors;
    DigitalChannel eyeBall;                // Device Object
    DigitalChannel eyeBall2;               // Device Object


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

    void Block_motor(long time) {
        BlockMotor.setPower(1);
        sleep(time);
    }

    void ServoTurn(long time) {
        Blockservo_left.setPosition(0);
        Blockservo_right.setPosition(1);
        sleep(time);
    }

    @Override
    public void runOpMode() {
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
        BlockMotor = hardwareMap.get(DcMotor.class, "Block_motor");
        RelicMotor = hardwareMap.get(DcMotor.class, "RelicMotor");
        Color_sensor.enableLight(true);
        FrightDrive.setDirection(DcMotor.Direction.REVERSE);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);
        sensorGyro = hardwareMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;      //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        mrGyro.calibrate();  //Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID
        Sensors = hardwareMap.get(DeviceInterfaceModule.class, "Sensors");
        eyeBall = hardwareMap.get(DigitalChannel.class, "eyeBall");     //  Use generic form of device mapping
        eyeBall.setMode(DigitalChannel.Mode.INPUT);      // Set the direction of this channel
        eyeBall2 = hardwareMap.get(DigitalChannel.class, "eyeBall2");     //  Use generic form of device mapping
        eyeBall2.setMode(DigitalChannel.Mode.INPUT);      // Set the direction of this channel


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
        Blockservo_left.setPosition(.5);
        Blockservo_right.setPosition(.5);
        RelicServo.setPosition(0.5);


        //PictoGraph(2);
        TestGyro(2);

    }

    public void PictoGraph(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        Color_sensor_servo.setPosition(.83);
        VuforiaTest.type type = new VuforiaTest().vuforiarun(hardwareMap);
        if (type == VuforiaTest.type.LEFT) {
            telemetry.addData("Type", "Left");
            Armdown(2);
            Jewel(0.5);
            LiftDown(2);
            sleep(300);
            LiftUp(2);
            sleep(300);
            //Glyph(2, 1);
        }
        if (type == VuforiaTest.type.CENTER) {
            telemetry.addData("Type", "Center");
            Armdown(2);
            Jewel(0.5);
            LiftDown(2);
            sleep(300);
            LiftUp(2);
            sleep(300);
            //Glyph(2, 2);
        }
        if (type == VuforiaTest.type.RIGHT) {
            telemetry.addData("Type", "Right");
            Armdown(2);
            Jewel(0.5);
            LiftDown(2);
            sleep(300);
            LiftUp(2);
            sleep(300);
            //Glyph(2, 3);
        }
        if (type == VuforiaTest.type.ERROR) {
            telemetry.addData("Type", "Error");
            Armdown(2);
            Jewel(0.5);
            LiftDown(2);
            sleep(300);
            LiftUp(2);
            sleep(300);
            //Glyph(2, 2);
        }
    }
    //  }

    public void TestGyro(int sleepTime) {

        Color_sensor_servo.setPosition(0.83);

        mrGyro.resetZAxisIntegrator();

        target = target - 20;
        turnAbsolute(target);

        sleep(1000);

        //mrGyro.resetZAxisIntegrator();

        target = target + 20;
        turnAbsolute(target);

        sleep(1000);

        Forward(500, .20);

        sleep(1000);

        //mrGyro.resetZAxisIntegrator();

        target = target - 85;
        turnAbsolute(target);

    }

    public void LiftDown(int sleepTime) {
        Color_sensor_servo.setPosition(.30);
        long time = System.currentTimeMillis();
        while (System.currentTimeMillis() < time + 800) {
            BlockMotor.setPower(-0.4);
        }
        BlockMotor.setPower(0.0);

        /*BlockMotor.setPower(-0.4);
        sleep(sleepTime);
        BlockMotor.setPower(0.0);
        */
    }

    public void LiftUp(int sleepTime) {
        long time = System.currentTimeMillis();
        while (System.currentTimeMillis() < time + 1000) {
            BlockMotor.setPower(0.4);
        }
        BlockMotor.setPower(0.0);

        /*BlockMotor.setPower(0.4);
        sleep(sleepTime);
        BlockMotor.setPower(0.0);
        */
    }

    public void Forward(int sleepTime, double Speed) {
        long time = System.currentTimeMillis();
        while (System.currentTimeMillis() < time + sleepTime) {
            FleftDrive.setPower(-Speed);
            FrightDrive.setPower(-Speed);
            BleftDrive.setPower(-Speed);
            BrightDrive.setPower(-Speed);
        }
        FleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
    }

    //This function turns a number of degrees compared to where the robot is. Positive numbers trn left.
    public void turn(int target) throws InterruptedException {
        turnAbsolute(target + mrGyro.getIntegratedZValue());
    }

    //This function turns a number of degrees compared to where the robot was when the program started. Positive numbers trn left.
    public void turnAbsolute(int target) {
        zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
        double turnSpeed = 0.25;
        while (Math.abs(zAccumulated - target) > 1.5 /* Was 3*/ && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
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
        FleftDrive.setPower(0);
        BleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BrightDrive.setPower(0);
    }


    public void Jewel(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        telemetry.update();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            telemetry.update();
            if (Color_sensor.blue() >= 0.3) {
                target = target + 20;
                turnAbsolute(target);
                sleep(500);
                Color_sensor_servo.setPosition(0.83);
                //mrGyro.resetZAxisIntegrator();
                target = target - 20;
                turnAbsolute(target);
                Forward(1500, .20);
            } else if (Color_sensor.red() >= 0.3) {
                target = target - 20;
                turnAbsolute(target);
                sleep(500);
                Color_sensor_servo.setPosition(0.83);
                //mrGyro.resetZAxisIntegrator();
                target = target + 20;
                turnAbsolute(target);
                Forward(1500, .20);
            }
        }
    }

    public void Armdown(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            Color_sensor_servo.setPosition(0.05);
        }
    }

    public void Glyph(double holdTime, int Direction) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        Color_sensor_servo.setPosition(.83);
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            if (Direction == 1) {
                forward(1000);
                //mrGyro.resetZAxisIntegrator();
                sleep(300);
                target = target - 80;
                turnAbsolute(target);
                strafeleft(1500);
                if (eyeBall.getState() && eyeBall2.getState()) {
                    slot = slot + 1;
                    if (slot == 3) {
                        stopDrive();
                    }
                    sleep(500);
                    ServoTurn(1500);
                    forward(300);
                }
                if (Direction == 2) {
                    forward(1000);
                    //mrGyro.resetZAxisIntegrator();
                    sleep(300);
                    target = target - 80;
                    turnAbsolute(target);
                    strafeleft(1000);
                    if (eyeBall.getState() && eyeBall2.getState()) {
                        slot = slot + 1;
                        if (slot == 2) {
                            stopDrive();
                            sleep(500);
                            ServoTurn(1500);
                            forward(300);
                        }
                        if (Direction == 3) {
                            forward(1000);
                            //mrGyro.resetZAxisIntegrator();
                            sleep(300);
                            target = target - 80;
                            turnAbsolute(target);
                            strafeleft(500);
                            if (eyeBall.getState() && eyeBall2.getState()) {
                                stopDrive();
                                sleep(500);
                                ServoTurn(1500);
                                forward(300);
                            }
                        }
                    }
                }
            }
        }
    }
}