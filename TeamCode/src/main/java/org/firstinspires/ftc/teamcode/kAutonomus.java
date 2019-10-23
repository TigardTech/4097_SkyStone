package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Autonomous(name="JAutonomus", group="Autonomous")
//@Disabled
public class kAutonomus extends LinearOpMode {

    int i=0;
    DcMotor FleftDrive, BleftDrive, FrightDrive, BrightDrive;
    ColorSensor sensorColor;
    ModernRoboticsI2cRangeSensor rangeSensor;
    public List<Component> components = new ArrayList<Component>();
    protected MecanumDrive mecanum;
    protected GyroSensor gyro;

    //Gryo Strafe
    private int maintainHeading;
    private boolean headingSet = false;

    //Functions:
    void Forward(double speed) {
        FleftDrive.setPower(-speed);
        FrightDrive.setPower(-speed);
        BleftDrive.setPower(-speed);
        BrightDrive.setPower(-speed);
    }

    void ForwardLeftTime(double speed) {
        FleftDrive.setPower(-speed);
        BleftDrive.setPower(-speed);


    }

    void Stop() {
        FleftDrive.setPower(0);
        FrightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
    }

    void Reverse(long time, double speed) {
        FleftDrive.setPower(speed);
        FrightDrive.setPower(speed);
        BleftDrive.setPower(speed);
        BrightDrive.setPower(speed);
        sleep(time);
    }

    void StrafeLeft(double speed) {
        FleftDrive.setPower(speed);
        FrightDrive.setPower(-speed);
        BleftDrive.setPower(-speed);
        BrightDrive.setPower(speed);
    }

    void StrafeRight(double speed) {
        FleftDrive.setPower(-speed);
        FrightDrive.setPower(speed);
        BleftDrive.setPower(speed);
        BrightDrive.setPower(-speed);

    }

    void TurnLeft(long time, double speed) {
        FleftDrive.setPower(speed);
        FrightDrive.setPower(-speed);
        BleftDrive.setPower(speed);
        BrightDrive.setPower(-speed);
        sleep(time);
    }

    void TurnRight(long time, double speed) {
        FleftDrive.setPower(-speed);
        FrightDrive.setPower(speed);
        BleftDrive.setPower(-speed);
        BrightDrive.setPower(speed);
        sleep(time);
    }

    void Right(long time, double speed) {
        FleftDrive.setPower(-speed);
        FrightDrive.setPower(speed);
        BleftDrive.setPower(-speed);
        BrightDrive.setPower(speed);
        sleep(time);
    }


    void ForwardTime(double speed) {
        FleftDrive.setPower(-speed);
        FrightDrive.setPower(-speed);
        BleftDrive.setPower(-speed);
        BrightDrive.setPower(-speed);
    }

    void Back(double speed) {
        FleftDrive.setPower(speed);
        FrightDrive.setPower(-speed);
        BleftDrive.setPower(speed);
        BrightDrive.setPower(-speed);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        //SENSORS
        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        sensorColor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        //rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        //Drive
     /*   DcMotor fl_motor = hardwareMap.dcMotor.get("FleftDrive");
        DcMotor fr_motor = hardwareMap.dcMotor.get("FrightDrive");
        DcMotor rl_motor = hardwareMap.dcMotor.get("BleftDrive");
        DcMotor rr_motor = hardwareMap.dcMotor.get("BrightDrive");*/


        FleftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrightDrive.setDirection(DcMotor.Direction.REVERSE);

        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);

        mecanum = new MecanumDrive(FleftDrive, FrightDrive, BleftDrive, BrightDrive, gyro);

        components.add(mecanum);

        //Values
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        waitForStart();


        while (opModeIsActive()) {

            //StrafeLeft(5000, 1);


            //Color Sensor Part:
            if (sensorColor.blue() > sensorColor.red()) {
                Back(0.1);
                if(rangeSensor.rawUltrasonic()==3) {
                    Stop();
                }
            }
            else{
                if(rangeSensor.rawUltrasonic()>=9 && rangeSensor.rawUltrasonic()<=23){
                    StrafeLeft(0.5);
                    ForwardLeftTime(0.6);
                }
                else if(rangeSensor.rawUltrasonic()<=8){
                    Forward(0.5);
                }
                else if(rangeSensor.rawUltrasonic()>26){
                    Back(0.5);
                }

            }

            //Update:
            /*telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.update();*/

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
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

            //Components
            for (Component component : components) {
                component.doit();
            }

            //Push data
            pushTelemetry();



        }


    }




    private void pushTelemetry() {
        telemetry.addData("Gyro Heading", gyro.getHeading());
    }

}




