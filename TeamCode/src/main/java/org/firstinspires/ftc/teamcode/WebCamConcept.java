package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@TeleOp(name="Concept: VuMark Id Webcam", group ="Concept")
public class WebCamConcept extends LinearOpMode {


    private VuforiaLocalizer vuforia;

    WebcamName webcamName;

    @Override
    public void runOpMode() {

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        parameters.vuforiaLicenseKey = " Ad/gkcD/////AAAAGQpe/pNamULdiyrqdpPPZw5at/xrLsowCA4BJKo/xSWBdYU8l7kxaSh43LFrNFu4wMzkHABuk6LjKLzG5725DTSobQ2SsuUYI71nmHbwMM7TTbx1hShJCtcg1x4YC4qsaEfWCPy5TdNoDGGg3QGGIyMbxuH7QVbzmRq4gwUFNWqr5aiV3e2EpyEt8oJrD34Vq3zwPmCSi1kwbWiVahln9lVfIjH38YNuE2zopplY/54FEWwqU7eueRiuipJkffI7hwICa4t9Nh0Vfh2VOgAI49iACtS6HeYTs4y9X7532GTGLdw7CAqvBd2fLJoQaJmwvC4YlP+WVRXJEd4eksLVRWTlTT1FVHWnVDEArMTO3hak ";


        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {


            telemetry.update();
        }
    }

}