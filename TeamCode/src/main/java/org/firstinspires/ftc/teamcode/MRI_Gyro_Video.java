package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Denney Cole on 1/9/2018.
 */



@Autonomous(name="Auto Gyro", group="dcole")

public class MRI_Gyro_Video extends LinearOpMode {



    public void runOpMode() throws InterruptedException {

        int zAccumulated;
        int heading;
        int xVal, yVal, zVal;

        GyroSensor sensorGyro;
        ModernRoboticsI2cGyro mrGyro;

        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;
        mrGyro.calibrate();

        waitForStart();

        while (mrGyro.isCalibrating()) {
        }

        while (opModeIsActive()) {

            zAccumulated = mrGyro.getIntegratedZValue();
            heading = mrGyro.getHeading();

            xVal = mrGyro.rawX();
            yVal = mrGyro.rawY();
            zVal = mrGyro.rawZ();

            telemetry.addData("1. heading", String.format("%03d", heading));
            telemetry.addData("2. accumul", String.format("%03d", zAccumulated));
            telemetry.addData("4. X", String.format("%03d", xVal));
            telemetry.addData("5. Y", String.format("%03d", yVal));
            telemetry.addData("6. Z", String.format("%03d", zVal));


        }
    }
}

