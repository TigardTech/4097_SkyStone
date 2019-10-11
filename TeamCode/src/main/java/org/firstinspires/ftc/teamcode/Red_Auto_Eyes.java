/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red_Auto_Eyes")
public class Red_Auto_Eyes extends LinearOpMode {
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
    private Servo Paddle_servo = null;
    private DcMotor RelicMotor = null;
    private DcMotor BlockMotor = null;

    static final int    BLUE_LED    = 0;     // Blue LED channel on DIM
    static final int    RED_LED     = 1;     // Red LED Channel on DIM

    DeviceInterfaceModule   Sensors;
    DigitalChannel          eyeBall;                // Device Object
    DigitalChannel          eyeBall2;               // Device Object


    public double speed = .25;
    int zAccumulated;  //Total rotation left/right
    int target = 0;  //Desired angle to turn to
    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
    boolean random = true;
    boolean Relicarmon;

    void stopDrive() {
        FleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
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
        Paddle_servo = hardwareMap.get(Servo.class, "Paddle_servo");
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
        eyeBall2 = hardwareMap.get(DigitalChannel.class, "eyeBall2");     //  Use generic form of device mapping

        eyeBall.setMode(DigitalChannel.Mode.INPUT);
        eyeBall2.setMode(DigitalChannel.Mode.INPUT); // Set the direction of this channel
        boolean  eye;
        boolean  eye2;                               // Input State


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
        Color_sensor.enableLight(true);
        Blockservo_left.setPosition(.5);
        Blockservo_right.setPosition(.5);
        RelicServo.setPosition(0.5);

        Paddle_servo.setPosition(0.7);

        PictoGraph(2);
        //TestGyro(2);

    }

    public void PictoGraph(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        //Paddle_servo.setPosition(0.7);
        Color_sensor_servo.setPosition(.83);
        VuforiaTest.type type = new VuforiaTest().vuforiarun(hardwareMap);
        if (type == VuforiaTest.type.LEFT) {
            telemetry.addData("Type", "Left");
            Armdown(2);
            sleep(1000);
            Jewel(0.5);
            LiftDown(2);
            sleep(300);
            LiftUp(2);
            sleep(300);
            Glyph(2, 1);
        }
        if (type == VuforiaTest.type.CENTER) {
            telemetry.addData("Type", "Center");
            Armdown(2);
            sleep(1000);
            Jewel(0.5);
            LiftDown(2);
            sleep(300);
            LiftUp(2);
            sleep(300);
            Glyph(2, 2);
        }
        if (type == VuforiaTest.type.RIGHT) {
            telemetry.addData("Type", "Right");
            Armdown(2);
            sleep(1000);
            Jewel(0.5);
            LiftDown(2);
            sleep(300);
            LiftUp(2);
            sleep(300);
            Glyph(2, 3);
        }
        if (type == VuforiaTest.type.ERROR) {
            telemetry.addData("Type", "Error");
            Armdown(2);
            sleep(1000);
            Jewel(0.5);
            LiftDown(2);
            sleep(300);
            LiftUp(2);
            sleep(300);
            Glyph(2, 2);
        }
    }
    //  }

    public void TestGyro(int sleepTime) {
        Color_sensor_servo.setPosition(0.83);
        mrGyro.resetZAxisIntegrator();
        turnAbsolute(-20);
        sleep(1000);
        turnAbsolute(0);
        sleep(1000);
        Forward(1500, .20);
        sleep(1000);
        //reverse(1000, .20);
        //sleep(1000);
        strafeleft(1500, .50);
    }

    public void LiftDown(int sleepTime) {
        Color_sensor_servo.setPosition(.30);
        long time = System.currentTimeMillis();
        while (System.currentTimeMillis() < time + 800) {
            BlockMotor.setPower(-0.4);
        }
        BlockMotor.setPower(0.0);
    }

    public void LiftUp(int sleepTime) {
        long time = System.currentTimeMillis();
        while (System.currentTimeMillis() < time + 1000) {
            BlockMotor.setPower(0.4);
        }
        BlockMotor.setPower(0.0);
    }
    public void strafeleft(int sleepTime, double Speed) {
        long time = System.currentTimeMillis();
        while (System.currentTimeMillis() < time + sleepTime) {
            FleftDrive.setPower(Speed);
            FrightDrive.setPower(-Speed);
            BleftDrive.setPower(-Speed);
            BrightDrive.setPower(Speed);
        }
        FleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
    }

    public void straferight(int sleepTime) {
        long time = System.currentTimeMillis();
        while (System.currentTimeMillis() < time + sleepTime) {
            FleftDrive.setPower(-speed);
            FrightDrive.setPower(speed);
            BleftDrive.setPower(speed);
            BrightDrive.setPower(-speed);
        }
        FleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
    }

    public void reverse(int sleepTime, double Speed) {
        long time = System.currentTimeMillis();
        while (System.currentTimeMillis() < time + sleepTime) {
            FleftDrive.setPower(Speed);
            FrightDrive.setPower(Speed);
            BleftDrive.setPower(Speed);
            BrightDrive.setPower(Speed);
        }
        FleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
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
        while (Math.abs(zAccumulated - target) > 0.5 /* Was 3*/ && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
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
                //target = target + 20;
                //turnAbsolute(target);
                Paddle_servo.setPosition(1);
                sleep(1000);
                Paddle_servo.setPosition(0.70);
                Color_sensor_servo.setPosition(0.83);

                //target = target - 20;
                //turnAbsolute(target);
                Forward(1150, .20);
            } else if (Color_sensor.red() >= 0.3) {
                //target = target - 20;
                //turnAbsolute(target);
                Paddle_servo.setPosition(0);
                sleep(1000);
                Paddle_servo.setPosition(0.70);
                Color_sensor_servo.setPosition(0.83);

                //target = target + 20;
                //turnAbsolute(target);
                Forward(1150, .20);
            }
        }
    }

    public void Armdown(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        //while (opModeIsActive() && holdTimer.time() < holdTime) {
            Color_sensor_servo.setPosition(0.05);
        //}
    }

    public void Glyph(double holdTime, int Direction) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        Color_sensor_servo.setPosition(.83);
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            if (Direction == 1) {

                strafeleft(2500, .50);
                if (eyeBall.getState() && eyeBall2.getState())   // this is the slot detector
                    stop();
                /*Forward(1000 , .20);
                //mrGyro.resetZAxisIntegrator();
                sleep(300);
                target = target - 80;
                turnAbsolute(target);
                strafeleft(1500, .50);
                sleep(500);
                ServoTurn(1500);
                Forward(300, .20);
                */
            }
            if (Direction == 2) {

                strafeleft(1500, .50);

                /*Forward(1000, .20);
                //mrGyro.resetZAxisIntegrator();
                sleep(300);
                target = target - 80;
                turnAbsolute(target);
                strafeleft(1000, .50);
                sleep(500);
                ServoTurn(1500);
                Forward(300, .20);
                */
            }
            if (Direction == 3) {

                strafeleft(1000, .50);

                /*Forward(1000, .20);
                //mrGyro.resetZAxisIntegrator();
                sleep(300);
                target = target - 80;
                turnAbsolute(target);
                strafeleft(500, .50);
                sleep(500);
                ServoTurn(1500);
                Forward(300, .20);
                */
            }
        }
    }
}









