package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "JLinearTest", group = "Tele")
public class JTeleop extends OpMode {
        //Declaring variables:
        DcMotor Fleftmotor, Bleftmotor, Frightmotor, Brightmotor, George, Grabber;
        Servo LTailGrabber,RTailGrabber;


    @Override
    public void init() {

        //Assigning variables with motors, and setting motors up for controller
        Fleftmotor=hardwareMap.dcMotor.get("FrightDrive");
        Bleftmotor=hardwareMap.dcMotor.get("BrightDrive");
        Frightmotor=hardwareMap.dcMotor.get("FleftDrive");
        Brightmotor=hardwareMap.dcMotor.get("BleftDrive");
        George=hardwareMap.dcMotor.get("GrabberMotor");
        Grabber=hardwareMap.dcMotor.get("Grabber");

        //Servo
        LTailGrabber=hardwareMap.servo.get("LTail");
        //RTailGrabber=hardwareMap.servo.get("RTail");


        //This parts help with me not needing to assing every setpower to a negative or positive
        Fleftmotor.setDirection(DcMotor.Direction.FORWARD);
        Frightmotor.setDirection(DcMotor.Direction.REVERSE);

        Bleftmotor.setDirection(DcMotor.Direction.FORWARD);
        Brightmotor.setDirection(DcMotor.Direction.REVERSE);



    }

    @Override
    public void loop() {
        double temp, temp2;
        double i=0.0005;
        temp=LTailGrabber.getPosition();
        //Moving Part(an equation that calculates the amount of energy needed, so that motors don't
        //contradict each other)
        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        //Setting power for motors
        Fleftmotor.setPower(v1);
        Frightmotor.setPower(v2);
        Bleftmotor.setPower(v3);
        Brightmotor.setPower(v4);

        if(gamepad1.right_bumper){
            George.setPower(1);
        }
        else{
            George.setPower(0);
        }

        if(gamepad1.left_bumper) {
            George.setPower(-1);
        }
        else{
            George.setPower(0);
        }

        if(gamepad1.a){
            temp=LTailGrabber.getPosition() + i;
            LTailGrabber.setPosition(temp);
            //temp2=RTailGrabber.getPosition()+i;
            //if(temp<=0.180 /*&& temp2!=0.180*/){
                //LTailGrabber.setPosition(temp);
                //RTailGrabber.setPosition(temp2);
            //}

        }
        if(gamepad1.y){
            temp = LTailGrabber.getPosition() - i;
            LTailGrabber.setPosition(temp);
            //temp2=RTailGrabber.getPosition() -i;
            //if(temp>=0.00 /*&& temp2!=0.00*/) {
                //LTailGrabber.setPosition(temp);
                //RTailGrabber.setPosition(temp2);
            //}
        }

        if(gamepad1.left_trigger>0){
            Grabber.setPower(0.5);
        }
        else{
            Grabber.setPower(0);
        }
        if(gamepad1.right_trigger>0){
            Grabber.setPower(-0.5);
        }
        else{
            Grabber.setPower(0);
        }

        if(gamepad1.x){
            LTailGrabber.setPosition(0.15);
        }
        if(gamepad1.b){
            LTailGrabber.setPosition(0.90);
        }


    }



}
