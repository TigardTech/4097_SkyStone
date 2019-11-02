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
import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.ForkJoinPool;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Rev_Autonomous")
public class Rev_Autonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    public double speed = 1;
    public DcMotor FleftDrive = null;
    public DcMotor FrightDrive = null;
    public DcMotor BleftDrive = null;
    public DcMotor BrightDrive = null;

    private DcMotor LiftMotor = null;

    private Servo BallServo = null;

    private Servo LatchServo = null;

    private Servo BallServoTurn = null;

    private DcMotor SlideMotor = null;

    LynxI2cColorRangeSensor Color_sensor;
    ModernRoboticsI2cRangeSensor Range_sensor;

    //LynxI2cColorRangeSensor Color_sensor = null;

    private boolean Huenum = false;
    private double rannum = 0;

    int steps = 0;

    void stopDrive() {
        FleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);

        LiftMotor.setPower(0);

        SlideMotor.setPower(0);

        SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void Forward(long time) {
        FleftDrive.setPower(-speed);
        FrightDrive.setPower(-speed);
        BleftDrive.setPower(-speed);
        BrightDrive.setPower(-speed);
        sleep(time);
    }

    void ForwardTime(double Speed) {
        FleftDrive.setPower(-Speed);
        FrightDrive.setPower(-Speed);
        BleftDrive.setPower(-Speed);
        BrightDrive.setPower(-Speed);
    }

    void Reverse(long time, double Speed) {
        FleftDrive.setPower(Speed);
        FrightDrive.setPower(Speed);
        BleftDrive.setPower(Speed);
        BrightDrive.setPower(Speed);
        sleep(time);
    }

    void StrafeLeft(long time) {
        FleftDrive.setPower(speed);
        FrightDrive.setPower(-speed);
        BleftDrive.setPower(-speed);
        BrightDrive.setPower(speed);
        sleep(time);
    }

    void StrafeRight(long time) {
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

    void TurnArm(long time, int power){
        // 1 Is toward front
        // -1 Is toward back
        LiftMotor.setPower(power);
        sleep(time);
    }

    void ReleaseBall(long time){
        BallServoTurn.setPosition(0);
        sleep(time);
    }

    void SpoolOut(long time){
        SlideMotor.setPower(-1);
        sleep(time);
    }
    void BucketStop(){
        BallServoTurn.setPosition(0.5);
    }


    //float hsvValues[] = {0F, 0F, 0F};

    //final double SCALE_FACTOR = 255;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        FleftDrive = hardwareMap.get(DcMotor.class, "FleftDrive");
        FrightDrive = hardwareMap.get(DcMotor.class, "FrightDrive");
        BleftDrive = hardwareMap.get(DcMotor.class, "BleftDrive");
        BrightDrive = hardwareMap.get(DcMotor.class, "BrightDrive");

        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");

        BallServo = hardwareMap.get(Servo.class, "BallServo");

        BallServoTurn = hardwareMap.get(Servo.class, "BallServoTurn");

        SlideMotor = hardwareMap.get(DcMotor.class, "SlideMotor");

        SlideMotor.setDirection(DcMotor.Direction.FORWARD);

        LiftMotor.setDirection(DcMotor.Direction.FORWARD);

        FleftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrightDrive.setDirection(DcMotor.Direction.REVERSE);

        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);


        LatchServo = hardwareMap.get(Servo.class, "LatchServo");

        Color_sensor = (LynxI2cColorRangeSensor) hardwareMap.get(ColorSensor.class, "Color_sensor");
        Range_sensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Range_sensor");


        //Color_sensor = (LynxI2cColorRangeSensor) hardwareMap.get(ColorSensor.class, "Color_sensor");
        //Color_sensor.enableLed(true);



         /*Color.RGBToHSV((int) (red * SCALE_FACTOR),
                (int) (green * SCALE_FACTOR),
                (int) (blue * SCALE_FACTOR),
                hsvValues);
        */


        telemetry.update();
        waitForStart();
        runtime.reset();

        TurnRight(680);
        stopDrive();
        sleep(1000);

        while (steps < 120 && opModeIsActive()) {
            telemetry.addData("Red  ", Color_sensor.red());
            telemetry.addData("Blue ", Color_sensor.blue());
            telemetry.update();
            steps += 1;
            if ((Color_sensor.blue() < 105) && (Color_sensor.red() < 105)) {
                ForwardTime(.2);
                if (Range_sensor.getDistance(DistanceUnit.CM) > 16){
                    StrafeRight(400);
                    stopDrive();
                }
            }
            else {
                stopDrive();
                sleep(1000);
                Reverse(500,.5);
                stopDrive();
                sleep(300);
                TurnArm(200,-1);
                stopDrive();
                sleep(1000);
                SpoolOut(3750);
                stopDrive();
                sleep(500);
                //TurnArm(100,1);
                //stopDrive();
                //sleep(1000);
                ReleaseBall(950);
                sleep(1000);
                //BucketStop();
                stopDrive();
                BallServoTurn.setPosition(0.5);
                sleep(1000);
                Reverse(2000,1);
                stopDrive();




                /*TurnArm(190,-1);
                stopDrive();
                sleep(1000);
                ReleaseBall(2000);
                sleep(1000);
                Reverse(3000,1);
                stopDrive();
                */
                break;
            }
        }
        stopDrive();

        /*while(opModeIsActive()){
            telemetry.addData("Red  ", Color_sensor.red());
            telemetry.addData("Blue ", Color_sensor.blue());



            telemetry.update();

            while(Color_sensor.red() < 200 || Color_sensor.blue() < 200){
                ForwardTime(1);
            }
            stopDrive();
        }
        */

            //telemetry.update();


        /*int red = Color_sensor.red();   // Red channel value
        int green = Color_sensor.green(); // Green channel value
        int blue = Color_sensor.blue();
        */

            //telemetry.addData("Hue", hsvValues[0]);
            //telemetry.addData("Found Gold", "False");




        /*while (opModeIsActive()) {
            Color.RGBToHSV((int) (Color_sensor.red() * SCALE_FACTOR),
                    (int) (Color_sensor.green() * SCALE_FACTOR),
                    (int) (Color_sensor.blue() * SCALE_FACTOR),
                    hsvValues);

        if (hsvValues[0] < 110){
            Forward(100);
            //stopDrive();
        }


            telemetry.addData("Hue", hsvValues[0]);
        }
        */




        /*LatchServo.setPosition(0.2);
        sleep(1800);
        //Reverse(200,0.75);
        Reverse(200,.60);
        stopDrive();
        sleep(1000);

        TurnRight(620);


        stopDrive();
        sleep(1000);

        Forward(2000);
        stopDrive();
        */

        /*//TurnRight(560);
        StrafeRight(400);

        stopDrive();
        sleep(1000);
        Forward(225); //was 250

        stopDrive();
        sleep(1000);
        */

            //TEST COLOR
        /*
        TestColor(2,Color_sensor.red(), Color_sensor.green(),Color_sensor.blue());
        if (hsvValues[0] < 110 ) {
            Forward(300);
            stopDrive();
            //sleep(1000);
            //Reverse(100,.50);
            stop();
            telemetry.addData("Found Gold", "True");
        }
        else if (hsvValues[0] > 110 ) {
            sleep(1000);
            StrafeLeft(675); // was 625
            stopDrive();
            sleep(1000);

            //Forward(25);
            //stopDrive();
            sleep(1000);


            TestColor(2,Color_sensor.red(), Color_sensor.green(),Color_sensor.blue());
            if (hsvValues[0] < 110) {
                Forward(300);
                stopDrive();
                stop();
                telemetry.addData("Found Gold", hsvValues[0]);
            } else if (hsvValues[0] > 110) {
                sleep(1000);
                StrafeLeft(675); //was 675
                stopDrive();

                sleep(1000);
                Forward(300);
                stopDrive();

            }

        }



            stopDrive();
            sleep(1000);
            StrafeLeft(625);

            Color.RGBToHSV((int) (Color_sensor.red() * SCALE_FACTOR),
                    (int) (Color_sensor.green() * SCALE_FACTOR),
                    (int) (Color_sensor.blue() * SCALE_FACTOR),
                    hsvValues);
            if (hsvValues[0] < 110 ) {
                Forward(100);
                stop();
            }

            */

        /*StrafeLeft(1300);
        if (rannum < 110 ) {
            Forward(100);
        }
        */

        /*while(rannum > 110){
            StrafeLeft(1300);
        }
        */
        /*while (rannum < 110 ) {
            Forward(100);
        }
        StrafeLeft(1300);

        */
       /* while (hsvValues[0] < 110 ) {
            Forward(100);
        }
        StrafeLeft(1250);
        */

            //stopDrive();

        }
    /*
    public void TestColor(double holdTime,int red, int green, int blue) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        //int red = Color_sensor.red();   // Red channel value
        //int green = Color_sensor.green(); // Green channel value
        //int blue = Color_sensor.blue();

        Color.RGBToHSV((int) (red * SCALE_FACTOR),
                (int) (green * SCALE_FACTOR),
                (int) (blue * SCALE_FACTOR),
                hsvValues);
        telemetry.addData("Hue", hsvValues[0]);

    }
    */

}








