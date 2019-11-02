package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "StrafeTest", group = "Tele")
public class StrafeTest extends OpMode {
    DcMotor Fleftmotor, Bleftmotor, Frightmotor, Brightmotor;
    @Override
    public void init() {
        Fleftmotor=hardwareMap.dcMotor.get("FleftDrive");
        Bleftmotor=hardwareMap.dcMotor.get("BleftDrive");
        Frightmotor=hardwareMap.dcMotor.get("FrightDrive");
        Brightmotor=hardwareMap.dcMotor.get("BrightDrive");
    }

    @Override
    public void loop() {
        Fleftmotor.setPower(gamepad1.left_trigger);
        Bleftmotor.setPower(-gamepad1.left_trigger);
        Frightmotor.setPower(gamepad1.left_trigger);
        Brightmotor.setPower(-gamepad1.left_trigger);

        Fleftmotor.setPower(-gamepad1.right_trigger);
        Bleftmotor.setPower(gamepad1.right_trigger);
        Frightmotor.setPower(-gamepad1.right_trigger);
        Brightmotor.setPower(gamepad1.right_trigger);
    }
}
