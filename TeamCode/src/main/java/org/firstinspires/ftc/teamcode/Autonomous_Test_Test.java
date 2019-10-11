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

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Rev_AutonomousTestTest")
public class Autonomous_Test_Test extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    public double speed = 1;
    public DcMotor FleftDrive = null;
    public DcMotor FrightDrive = null;
    public DcMotor BleftDrive = null;
    public DcMotor BrightDrive = null;

    private Servo LatchServo = null;

    void stopDrive() {
        FleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);

        LatchServo.setPosition(.5);
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


    void ReleaseLatch(long time) {
        LatchServo.setPosition(0);
        sleep(time);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        FleftDrive = hardwareMap.get(DcMotor.class, "FleftDrive");
        FrightDrive = hardwareMap.get(DcMotor.class, "FrightDrive");
        BleftDrive = hardwareMap.get(DcMotor.class, "BleftDrive");
        BrightDrive = hardwareMap.get(DcMotor.class, "BrightDrive");


        FleftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrightDrive.setDirection(DcMotor.Direction.REVERSE);

        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);


        LatchServo = hardwareMap.get(Servo.class, "LatchServo");


        telemetry.update();
        waitForStart();
        runtime.reset();

        ReleaseLatch(1000);
        stopDrive();
        sleep(1000);

        TurnLeft(500);
        stopDrive();
        sleep(1000);

        Forward(2000);



    }
}








