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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Basic: Test Linear OpMode", group="Linear Opmode")

public class OpMode_Linear_Test extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor FleftDrive = null;
    private DcMotor FrightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;

    private DcMotor LiftMotor = null;

    private DcMotor SlideMotor = null;

    //private DcMotor BallMotor = null;
    private Servo BallServo = null;
    private Servo BallServoTurn = null;

    private Servo LatchServo = null;

    private int state = 1;

    boolean toggle = true;
    boolean on = false;

    int ths = 0;
    boolean last = false;
    boolean output = false;


    LynxI2cColorRangeSensor Color_sensor;
    ModernRoboticsI2cRangeSensor Range_sensor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Drive Motors
        FleftDrive  = hardwareMap.get(DcMotor.class, "FleftDrive");
        FrightDrive = hardwareMap.get(DcMotor.class, "FrightDrive");
        BleftDrive  = hardwareMap.get(DcMotor.class, "BleftDrive");
        BrightDrive = hardwareMap.get(DcMotor.class, "BrightDrive");

        //Latching Motor
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");

        //Latching Servo
        LatchServo = hardwareMap.get(Servo.class, "LatchServo");

        //Ball Servo
        BallServo = hardwareMap.get(Servo.class, "BallServo");
        BallServoTurn = hardwareMap.get(Servo.class, "BallServoTurn");

        SlideMotor = hardwareMap.get(DcMotor.class, "SlideMotor");

        SlideMotor.setDirection(DcMotor.Direction.FORWARD);

        LiftMotor.setDirection(DcMotor.Direction.FORWARD);

        FleftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrightDrive.setDirection(DcMotor.Direction.REVERSE);

        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);


        Color_sensor = (LynxI2cColorRangeSensor) hardwareMap.get(ColorSensor.class, "Color_sensor");
        Range_sensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Range_sensor");

        //Color_sensor = (LynxI2cColorRangeSensor) hardwareMap.get(ColorSensor.class, "Color_sensor");
       // Color_sensor.enableLed(true);

        //float hsvValues[] = {0F, 0F, 0F};

        //final double SCALE_FACTOR = 255;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Color.RGBToHSV((int) (Color_sensor.red() * SCALE_FACTOR),
                  //  (int) (Color_sensor.green() * SCALE_FACTOR),
                   // (int) (Color_sensor.blue() * SCALE_FACTOR),
                   // hsvValues);

            //telemetry.addData("raw ultrasonic", Range_sensor.rawUltrasonic());
            //telemetry.addData("raw optical", Range_sensor.rawOptical());
            //telemetry.addData("cm optical", "%.2f cm", Range_sensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", Range_sensor.getDistance(DistanceUnit.CM));

            telemetry.addData("Red  ", Color_sensor.red());
            telemetry.addData("Green", Color_sensor.green());
            telemetry.addData("Blue ", Color_sensor.blue());

            //telemetry.update();

            switch(state) {
                case 1:
                    /*telemetry.addData("Alpha", Color_sensor.alpha());
                    telemetry.addData("Red  ", Color_sensor.red());
                    telemetry.addData("Green", Color_sensor.green());
                    telemetry.addData("Blue ", Color_sensor.blue());
                    telemetry.addData("Hue", hsvValues[0]);


                    if (hsvValues[0] < 110) {
                       telemetry.addData("I see yellow", "True");
                    }
                    */

                    if(gamepad1.right_trigger > .4) {
                        LatchServo.setPosition(0.3);
                    }
                    //else if (gamepad1.left_trigger > .4) {
                       // LatchServo.setPosition(1);
                    //}
                    else{
                        LatchServo.setPosition(0.70);
                    }

                    if (gamepad1.right_bumper) {
                        LiftMotor.setPower(1);
                    } else if (gamepad1.left_bumper) {
                        LiftMotor.setPower(-1);
                    } else {
                        LiftMotor.setPower(0);
                    }

                    //START TEST

                    if (gamepad1.x) {
                        BallServo.setPosition(0);
                    } else if (gamepad1.b) {
                        BallServo.setPosition(1);
                    } else {
                        BallServo.setPosition(.5);
                    }


                    /*
                    if (gamepad1.x){
                        ths = true;
                    }
                    else{
                        ths = false;
                    }


                    if ((ths == true) && (last = false)){
                        if (output == false){
                            output = true;
                        }
                        else{
                            output = false;
                        }
                    }

                    last = ths;

                    if (output = true){
                        BallServo.setPosition(0);
                    }else if(output = false){
                        BallServo.setPosition(.5);
                    }
                    */

                    /*switch(ths){
                        case 0: BallServo.setPosition(.5);
                            break;

                        case 1: BallServo.setPosition(0);
                            break;
                    }

                    if (gamepad1.x){
                        if (ths == 0) {
                            ths = 1;
                        }
                        if (ths == 1){
                            ths = 0;
                        }
                    }
                    */

                    //END TEST

                    if (gamepad1.a) {
                        BallServoTurn.setPosition(0);
                    } else if (gamepad1.y) {
                        BallServoTurn.setPosition(1);
                    } else {
                        BallServoTurn.setPosition(.5);
                    }

                    if (gamepad1.dpad_up){
                        SlideMotor.setPower(1);
                    }
                    if (gamepad1.dpad_down){
                        SlideMotor.setPower(-1);
                    }
                    else {
                        SlideMotor.setPower(0);
                    }


                    double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                    double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                    double rightX = gamepad1.right_stick_x;
                    final double v1 = r * Math.cos(robotAngle) - rightX;
                    final double v2 = r * Math.sin(robotAngle) + rightX;
                    final double v3 = r * Math.sin(robotAngle) - rightX;
                    final double v4 = r * Math.cos(robotAngle) + rightX;

                    FleftDrive.setPower(v1);
                    FrightDrive.setPower(v2);
                    BleftDrive.setPower(v3);
                    BrightDrive.setPower(v4);

                    SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    FleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    FrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    break;

                case 2:
                    if (gamepad1.right_bumper) {
                        FleftDrive.setPower(1);
                    } else if (gamepad1.left_bumper) {
                        FleftDrive.setPower(-1);
                    } else {
                        FleftDrive.setPower(0);
                    }

                    break;
            }
            /*
            if (gamepad1.dpad_up){
                state = 2;
            }
            if (gamepad1.dpad_down){
                state = 1;
            }
            */

            telemetry.addData("State", state );

            telemetry.addData("FLeft Power", FleftDrive.getPower());
            telemetry.addData("FRight Power", FrightDrive.getPower());
            telemetry.addData("BLeft Power", BleftDrive.getPower());
            telemetry.addData("BRight Power", BrightDrive.getPower());

            telemetry.addData("LatchServo Pos:", LatchServo.getPosition());

            telemetry.addData("Bucket Servo Pos: ", BallServoTurn.getPosition());

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}