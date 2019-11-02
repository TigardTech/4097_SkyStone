
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Lol welcome to my variable names. -Aaron J.", group="Linear Auto")

public class VufTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        VuforiaTest vu = new VuforiaTest();
        VuforiaTest.type typee = VuforiaTest.type.ERROR;
        typee = vu.vuforiarun(hardwareMap);
        if (typee == VuforiaTest.type.RIGHT) {
            while (opModeIsActive()) {
                telemetry.addData("You did it: ", typee);
                telemetry.update();
            }
        } else {
            while (opModeIsActive()) {
                telemetry.addData("You failed: ", typee);
                telemetry.update();
            }
        }
    }
}