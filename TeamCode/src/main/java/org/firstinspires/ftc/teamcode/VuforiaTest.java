/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

//@Autonomous(name="Concept: VuMark Id", group ="Concept")
public class VuforiaTest {

    public int PictoGraphNumb = 0;

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    public enum type {
        LEFT, CENTER, RIGHT, ERROR
    }

     public type vuforiarun(HardwareMap hardwareMap) {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

         parameters.vuforiaLicenseKey = "Ad/gkcD/////AAAAGQpe/pNamULdiyrqdpPPZw5at/xrLsowCA4BJKo/xSWBdYU8l7kxaSh43LFrNFu4wMzkHABuk6LjKLzG5725DTSobQ2SsuUYI71nmHbwMM7TTbx1hShJCtcg1x4YC4qsaEfWCPy5TdNoDGGg3QGGIyMbxuH7QVbzmRq4gwUFNWqr5aiV3e2EpyEt8oJrD34Vq3zwPmCSi1kwbWiVahln9lVfIjH38YNuE2zopplY/54FEWwqU7eueRiuipJkffI7hwICa4t9Nh0Vfh2VOgAI49iACtS6HeYTs4y9X7532GTGLdw7CAqvBd2fLJoQaJmwvC4YlP+WVRXJEd4eksLVRWTlTT1FVHWnVDEArMTO3hak";
         parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        /*telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart(); */

        relicTrackables.activate();

         boolean bool = true;
         CameraDevice.getInstance().setFlashTorchMode(true);
         long time = System.currentTimeMillis();

         while ((bool) && (System.currentTimeMillis() < (time + 2000))) {
             RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

             if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                 if (vuMark == vuMark.LEFT) {
                     PictoGraphNumb = 1;
                     bool = false;
                 }
                 if (vuMark == vuMark.RIGHT) {
                     PictoGraphNumb = 2;
                     bool = false;
                 }
                 if (vuMark == vuMark.CENTER) {
                     PictoGraphNumb = 3;
                     bool = false;
                 }


                 //TODO: Find out if removing this will break anything
                 OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                 if (pose != null) {
                     VectorF trans = pose.getTranslation();
                     Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                 }
             }
         } //End of while loop
         CameraDevice.getInstance().setFlashTorchMode(false);
         if (PictoGraphNumb == 1) {
             return type.LEFT;
         } else if (PictoGraphNumb == 2) {
             return type.RIGHT;
         } else if (PictoGraphNumb == 3) {
             return type.CENTER;
         } else {
             return type.ERROR;
         }
     }
}
