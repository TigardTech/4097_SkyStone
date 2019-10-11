package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name="JAutonomus", group="Autonomous")
//@Disabled
public class kAutonomus extends LinearOpMode {

    double speed = 0.1;
    DcMotor FleftDrive, BleftDrive, FrightDrive, BrightDrive;
    ColorSensor sensorColor;

    void Forward() {
        FleftDrive.setPower(-speed);
        FrightDrive.setPower(speed);
        BleftDrive.setPower(-speed);
        BrightDrive.setPower(speed);
    }

    void Stop() {
        FleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
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


    void ForwardTime(double Speed) {
        FleftDrive.setPower(-Speed);
        FrightDrive.setPower(-Speed);
        BleftDrive.setPower(-Speed);
        BrightDrive.setPower(-Speed);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        sensorColor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        FleftDrive = hardwareMap.dcMotor.get("FleftDrive");
        BleftDrive = hardwareMap.dcMotor.get("BleftDrive");
        FrightDrive = hardwareMap.dcMotor.get("FrightDrive");
        BrightDrive = hardwareMap.dcMotor.get("BrightDrive");
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        waitForStart();


        while (opModeIsActive()) {



            if (sensorColor.blue() > sensorColor.red()) {
                Stop();
                speed=0;
            }
            else{
                Forward();
            }




            /*Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.

            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();


            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });

            }*/


        }


    }

}




