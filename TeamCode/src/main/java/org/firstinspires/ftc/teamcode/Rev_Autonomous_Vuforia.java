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
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Rev_Autonomous_Vuforia")
public class Rev_Autonomous_Vuforia extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = " Ad/gkcD/////AAAAGQpe/pNamULdiyrqdpPPZw5at/xrLsowCA4BJKo/xSWBdYU8l7kxaSh43LFrNFu4wMzkHABuk6LjKLzG5725DTSobQ2SsuUYI71nmHbwMM7TTbx1hShJCtcg1x4YC4qsaEfWCPy5TdNoDGGg3QGGIyMbxuH7QVbzmRq4gwUFNWqr5aiV3e2EpyEt8oJrD34Vq3zwPmCSi1kwbWiVahln9lVfIjH38YNuE2zopplY/54FEWwqU7eueRiuipJkffI7hwICa4t9Nh0Vfh2VOgAI49iACtS6HeYTs4y9X7532GTGLdw7CAqvBd2fLJoQaJmwvC4YlP+WVRXJEd4eksLVRWTlTT1FVHWnVDEArMTO3hak ";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    public double speed = 1;
    public DcMotor FleftDrive = null;
    public DcMotor FrightDrive = null;
    public DcMotor BleftDrive = null;
    public DcMotor BrightDrive = null;

    private Servo BallServoTurn = null;

    private DcMotor SlideMotor = null;

    private Servo LatchServo = null;

    private DcMotor LiftMotor = null;

    private Servo BallServo = null;

    LynxI2cColorRangeSensor Color_sensor = null;
    ModernRoboticsI2cRangeSensor Range_sensor = null;

    int steps = 0;

    void stopDrive() {
        FleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
        LiftMotor.setPower(0);
        BallServo.setPosition(.5);

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
    void Right(long time, double Speed) {
        FleftDrive.setPower(-Speed);
        FrightDrive.setPower(Speed);
        BleftDrive.setPower(-Speed);
        BrightDrive.setPower(Speed);
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
    void ForwardTime(double Speed) {
        FleftDrive.setPower(-Speed);
        FrightDrive.setPower(-Speed);
        BleftDrive.setPower(-Speed);
        BrightDrive.setPower(-Speed);
    }

    void SpoolOut(long time){
        SlideMotor.setPower(-1);
        sleep(time);
    }


    float hsvValues[] = {0F, 0F, 0F};

    final double SCALE_FACTOR = 255;




    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        FleftDrive = hardwareMap.get(DcMotor.class, "FleftDrive");
        FrightDrive = hardwareMap.get(DcMotor.class, "FrightDrive");
        BleftDrive = hardwareMap.get(DcMotor.class, "BleftDrive");
        BrightDrive = hardwareMap.get(DcMotor.class, "BrightDrive");

        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");

        BallServoTurn = hardwareMap.get(Servo.class, "BallServoTurn");

        SlideMotor = hardwareMap.get(DcMotor.class, "SlideMotor");

        SlideMotor.setDirection(DcMotor.Direction.FORWARD);

        FleftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrightDrive.setDirection(DcMotor.Direction.REVERSE);

        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);

        LiftMotor.setDirection(DcMotor.Direction.FORWARD);



        BallServo = hardwareMap.get(Servo.class, "BallServo");


        LatchServo = hardwareMap.get(Servo.class, "LatchServo");

        Color_sensor = (LynxI2cColorRangeSensor) hardwareMap.get(ColorSensor.class, "Color_sensor");
        Range_sensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Range_sensor");


        //Color_sensor.enableLed(true);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();


       //START
        waitForStart();
        runtime.reset();
        timer.reset();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3  ) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    //Was Left
                                    //sleep(1000);
                                    LatchServo.setPosition(0.2);
                                    sleep(1500);
                                    //was 1800
                                    Reverse(150,.60);
                                    //was 180
                                    //Reverse was 250
                                    stopDrive();
                                    sleep(1000);

                                    StrafeRight(400);
                                    stopDrive();
                                    sleep(750);

                                    TurnRight(720);
                                    //was 680
                                    stopDrive();
                                    sleep(500);

                                    StrafeRight(300);
                                    //was 120
                                    stopDrive();
                                    sleep(200);

                                    Forward(610);
                                    //was 500
                                    stopDrive();
                                    sleep(1000);

                                    Reverse(350,1);
                                    stopDrive();
                                    sleep(500);

                                    TurnLeft(620);
                                    //was 580
                                    //was 520
                                    //was 680
                                    stopDrive();
                                    sleep(1000);

                                    Forward(450);
                                    stopDrive();
                                    sleep(400);

                                    TurnRight(300);

                                    Forward(1200);
                                    //was 1250
                                    //was 1400
                                    stopDrive();
                                    sleep(500);

                                    TurnLeft(515);
                                    //was 400
                                    stopDrive();
                                    sleep(500);


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
                                            //sleep(500);
                                            TurnArm(130,-1);
                                            stopDrive();
                                            //sleep(1000);
                                            Reverse(500,.5);
                                            stopDrive();
                                            //sleep(300);
                                            SpoolOut(2750);
                                            //was 3750
                                            stopDrive();
                                            sleep(500);
                                            ReleaseBall(300);
                                            //was 950
                                            //sleep(750);
                                            stopDrive();
                                            //BallServoTurn.setPosition(0.5);
                                            sleep(500);
                                            Reverse(1600,1);
                                            stopDrive();

                                            break;
                                        }
                                    }
                                    stopDrive();

                                    stop();
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    //Was Right
                                    //sleep(1000);
                                    LatchServo.setPosition(0.2);
                                    sleep(1500);
                                    //was 1800
                                    Reverse(150,.60);
                                    //was 180
                                    //Reverse was 250
                                    stopDrive();
                                    sleep(1000);

                                    StrafeRight(400);
                                    stopDrive();
                                    sleep(750);

                                    TurnRight(720);
                                    //was 680
                                    stopDrive();
                                    sleep(500);

                                    StrafeLeft(750);
                                    //was 120
                                    stopDrive();
                                    sleep(200);

                                    Forward(610);
                                    //was 500
                                    stopDrive();
                                    sleep(1000);

                                    Reverse(350,1);
                                    stopDrive();
                                    sleep(500);

                                    TurnLeft(400);
                                    //was 620
                                    //was 580
                                    //was 520
                                    //was 680
                                    stopDrive();
                                    sleep(1000);

                                    Forward(900);
                                    //was 1250
                                    //was 1400
                                    stopDrive();
                                    sleep(500);

                                    TurnLeft(515);
                                    //was 400
                                    stopDrive();
                                    sleep(500);


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
                                            //sleep(500);
                                            TurnArm(130,-1);
                                            stopDrive();
                                            //sleep(1000);
                                            Reverse(500,.5);
                                            stopDrive();
                                            //sleep(300);
                                            SpoolOut(2750);
                                            //was 3750
                                            stopDrive();
                                            sleep(500);
                                            ReleaseBall(300);
                                            //was 950
                                            //sleep(750);
                                            stopDrive();
                                            //BallServoTurn.setPosition(0.5);
                                            sleep(500);
                                            Reverse(1600,1);
                                            stopDrive();

                                            break;
                                        }
                                    }
                                    stopDrive();

                                    stop();
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    //sleep(1000);
                                    LatchServo.setPosition(0.2);
                                    sleep(1500);
                                    //was 1800
                                    Reverse(150,.60);
                                    //was 180
                                    //Reverse was 250
                                    stopDrive();
                                    sleep(1000);

                                    StrafeRight(400);
                                    stopDrive();
                                    sleep(750);

                                    TurnRight(720);
                                    //was 680
                                    stopDrive();
                                    sleep(500);

                                    StrafeLeft(450);
                                    //was 120
                                    stopDrive();
                                    sleep(200);

                                    Forward(610);
                                    //was 500
                                    stopDrive();
                                    sleep(1000);

                                    Reverse(350,1);
                                    stopDrive();
                                    sleep(500);

                                    TurnLeft(620);
                                    //was 580
                                    //was 520
                                    //was 680
                                    stopDrive();
                                    sleep(1000);

                                    Forward(1250);
                                    //was 1400
                                    stopDrive();
                                    sleep(500);

                                    TurnLeft(515);
                                    //was 400
                                    stopDrive();
                                    sleep(500);


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
                                            //sleep(500);
                                            TurnArm(130,-1);
                                            stopDrive();
                                            //sleep(1000);
                                            Reverse(500,.5);
                                            stopDrive();
                                            //sleep(300);
                                            SpoolOut(3000);
                                            //was 3750
                                            stopDrive();
                                            sleep(500);
                                            ReleaseBall(300);
                                            //was 950
                                            //sleep(750);
                                            stopDrive();
                                            //BallServoTurn.setPosition(0.5);
                                            sleep(500);
                                            Reverse(1600,1);
                                            stopDrive();

                                            break;
                                        }
                                    }
                                    stopDrive();

                                    stop();

                                }
                            }
                        }
                        else if ((updatedRecognitions.size() != 3  ) && timer.time() > 1.5){
                            //sleep(1000);
                            LatchServo.setPosition(0.2);
                            sleep(1500);
                            //was 1800
                            Reverse(150,.60);
                            //was 180
                            //Reverse was 250
                            stopDrive();
                            sleep(1000);

                            StrafeRight(400);
                            stopDrive();
                            sleep(750);

                            TurnRight(720);
                            //was 680
                            stopDrive();
                            sleep(500);

                            StrafeLeft(450);
                            //was 120
                            stopDrive();
                            sleep(200);

                            Forward(610);
                            //was 500
                            stopDrive();
                            sleep(1000);

                            Reverse(350,1);
                            stopDrive();
                            sleep(500);

                            TurnLeft(620);
                            //was 580
                            //was 520
                            //was 680
                            stopDrive();
                            sleep(1000);

                            Forward(1250);
                            //was 1400
                            stopDrive();
                            sleep(500);

                            TurnLeft(515);
                            //was 400
                            stopDrive();
                            sleep(500);


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
                                    //sleep(500);
                                    TurnArm(130,-1);
                                    stopDrive();
                                    //sleep(1000);
                                    Reverse(500,.5);
                                    stopDrive();
                                    //sleep(300);
                                    SpoolOut(3000);
                                    //was 3750
                                    stopDrive();
                                    sleep(500);
                                    ReleaseBall(300);
                                    //was 950
                                    //sleep(750);
                                    stopDrive();
                                    //BallServoTurn.setPosition(0.5);
                                    sleep(500);
                                    Reverse(1600,1);
                                    stopDrive();

                                    break;
                                }
                            }
                            stopDrive();

                            stop();

                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }


    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}









