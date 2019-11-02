package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="GyroAssit", group="Tele")
public class GyroAssist extends OpMode {
    public List<Component> components = new ArrayList<Component>();
    protected MecanumDrive mecanum;

    protected GyroSensor gyro;

    //Gryo Strafe
    private int maintainHeading;
    private boolean headingSet = false;
    @Override
    public void init() {
        //SENSORS
        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        //DRIVING
        DcMotor fl_motor = hardwareMap.dcMotor.get("FleftDrive");
        DcMotor fr_motor = hardwareMap.dcMotor.get("FrightDrive");
        DcMotor rl_motor = hardwareMap.dcMotor.get("BleftDrive");
        DcMotor rr_motor = hardwareMap.dcMotor.get("BrightDrive");

        fl_motor.setDirection(DcMotor.Direction.FORWARD);
        fr_motor.setDirection(DcMotor.Direction.REVERSE);

        rl_motor.setDirection(DcMotor.Direction.FORWARD);
        rr_motor.setDirection(DcMotor.Direction.REVERSE);

        mecanum = new MecanumDrive(fl_motor, fr_motor, rl_motor, rr_motor, gyro);

        components.add(mecanum);
    }

    @Override
    public void loop() {
        //Joystick Movement
        mecanum.move(this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, this.gamepad1.right_stick_x);

        //Gyro Assisted Strafing
        if(this.gamepad1.dpad_left || this.gamepad1.dpad_right){
            if(!headingSet){
                maintainHeading = gyro.getHeading();
                headingSet = true;
            }

            if(gamepad1.dpad_right){
                mecanum.move(1, 0, 0);
            }else{
                mecanum.move(-1, 0, 0);
            }

            mecanum.angleRotation(maintainHeading);
        }else{
            if(headingSet){
                headingSet = false;
            }
        }
        //Components
        for (Component component : components)
        {
            component.doit();
        }

        //Push data
        pushTelemetry();
    }

    public void pushTelemetry(){
        telemetry.addData("Gyro Heading", gyro.getHeading());
    }
}
