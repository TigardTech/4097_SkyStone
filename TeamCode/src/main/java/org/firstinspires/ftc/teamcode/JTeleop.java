package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "JLinearTest", group = "Tele")
public class JTeleop extends OpMode {

    DcMotor Fleftmotor, Bleftmotor, Frightmotor, Brightmotor;

    @Override
    public void init() {

        Fleftmotor=hardwareMap.dcMotor.get("FleftDrive");
        Bleftmotor=hardwareMap.dcMotor.get("BleftDrive");
        Frightmotor=hardwareMap.dcMotor.get("FrightDrive");
        Brightmotor=hardwareMap.dcMotor.get("BrightDrive");

        Fleftmotor.setDirection(DcMotor.Direction.FORWARD);
        Frightmotor.setDirection(DcMotor.Direction.REVERSE);

        Bleftmotor.setDirection(DcMotor.Direction.FORWARD);
        Brightmotor.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        //Moving Part
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = r * Math.cos(robotAngle) + rightX;

        Fleftmotor.setPower(v1);
        Frightmotor.setPower(v2);
        Bleftmotor.setPower(v3);
        Brightmotor.setPower(v4);



    }
}
