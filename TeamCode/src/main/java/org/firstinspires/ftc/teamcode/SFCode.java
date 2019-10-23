package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "750Code2", group = "Tele")
public class SFCode extends OpMode {
    DcMotor Leftmotor, Rightmotor;
    @Override
    public void init() {
        Leftmotor = hardwareMap.dcMotor.get("LeftDrive");
        Rightmotor=hardwareMap.dcMotor.get("RightDrive");
    }

    @Override
    public void loop() {
        Leftmotor.setPower(-gamepad1.left_stick_y);
        Rightmotor.setPower(-gamepad1.left_stick_y);
    }
}
