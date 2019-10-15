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

package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/*
Created by: Ethan Lanting
Date: 10/12/19

Skystone Field Positioning System (FPS) using a Webcam and VuMarks.
 */

@TeleOp(name="Skystone FPS", group ="Concept")
public class SkystoneFPS extends LinearOpMode {

    public static final String TAG = "Vuforia Navigation Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * @see #captureFrameToFile()
     */
    int captureCounter = 0;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;


    VuforiaLocalizer vuforia;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName;

    @Override
    public void runOpMode() {

        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        /**
         * Instantiate the Vuforia engine
         */
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        /**
         * Because this opmode processes frames in order to write them to a file, we tell Vuforia
         * that we want to ensure that certain frame formats are available in the {@link Frame}s we
         * see.
         */
        vuforia.enableConvertFrameToBitmap();

        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);


        /*
        Define VuMarks.
        Rear/Back is in the Building Zone.
        Front is in the Loading Zone.
         */
        VuforiaTrackables skystone = vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable bridgeBlueBack = skystone.get(1);
        bridgeBlueBack.setName("bridgeBlueBack");

        VuforiaTrackable bridgeRedBack = skystone.get(2);
        bridgeRedBack.setName("bridgeRedBack");

        VuforiaTrackable bridgeRedFront = skystone.get(3);
        bridgeRedBack.setName("bridgeRedFront");

        VuforiaTrackable bridgeBlueFront = skystone.get(4);
        bridgeRedBack.setName("bridgeBlueFront");

        VuforiaTrackable RedPerimeterBack = skystone.get(5);
        bridgeRedBack.setName("RedPerimeterBack");

        VuforiaTrackable RedPerimeterFront = skystone.get(6);
        bridgeRedBack.setName("RedPerimeterFront");

        VuforiaTrackable FrontPerimeterRed = skystone.get(7);
        bridgeRedBack.setName("FrontPerimeterRed");

        VuforiaTrackable FrontPerimeterBlue = skystone.get(8);
        bridgeRedBack.setName("FrontPerimeterBlue");

        VuforiaTrackable BluePerimeterFront = skystone.get(9);
        bridgeRedBack.setName("BluePerimeterFront");

        VuforiaTrackable BluePerimeterBack = skystone.get(10);
        bridgeRedBack.setName("BluePerimeterBack");

        VuforiaTrackable RearPerimeterBlue = skystone.get(11);
        bridgeRedBack.setName("RearPerimeterBlue");

        VuforiaTrackable RearPerimeterRed = skystone.get(12);
        bridgeRedBack.setName("RearPerimeterRed");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(skystone);

        //Convert field measurements to mm because Skystone XML file data uses mm.
        final float mmPerInch = 25.4f;
        final float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
        final float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" (142") center-to-center of the glass panels
        final float vumarkHeight = 5.75f * mmPerInch;
        final float vumarkDistanceFromWallCenter = 35 * mmPerInch;
        final float bridgeVumarkXDistanceFromOrigin = 24 * mmPerInch;
        final float bridgeVumarkYDistanceFromOrigin = 9 * mmPerInch;

        final float xOrigin = 0.0f;
        final float yOrigin = 0.0f;
        final float zOrigin = 0.0f;

        final float X_MAX = mmFTCFieldWidth / 2;
        final float Y_MAX = mmFTCFieldWidth / 2;
        final float Z_MAX = mmFTCFieldWidth / 2;

        final float X_MIN = -mmFTCFieldWidth / 2;
        final float Y_MIN = -mmFTCFieldWidth / 2;
        final float Z_MIN = 0.0f;


        //Define Rear Perimeter Target 1 position on field.
        OpenGLMatrix RearPerimeterBlueLocation = OpenGLMatrix
                .translation(-vumarkDistanceFromWallCenter, -mmFTCFieldWidth / 2, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 180, 0));
        RearPerimeterBlue.setLocationFtcFieldFromTarget(RearPerimeterBlueLocation);
        RobotLog.ii(TAG, "Rear Perimeter Blue=%s", format(RearPerimeterBlueLocation));

        //Define Rear Perimeter Target 2 position on field.
        OpenGLMatrix RearPerimeterRedLocation = OpenGLMatrix
                .translation(vumarkDistanceFromWallCenter, -mmFTCFieldWidth / 2, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 180, 0));
        RearPerimeterRed.setLocationFtcFieldFromTarget(RearPerimeterRedLocation);
        RobotLog.ii(TAG, "Rear Perimeter Red=%s", format(RearPerimeterRedLocation));

        //Define Front Perimeter Target 2 position on field.
        OpenGLMatrix FrontPerimeterBlueLocation = OpenGLMatrix
                .translation(-vumarkDistanceFromWallCenter, mmFTCFieldWidth / 2, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        FrontPerimeterBlue.setLocationFtcFieldFromTarget(FrontPerimeterBlueLocation);
        RobotLog.ii(TAG, "Front Perimeter Blue=%s", format(FrontPerimeterBlueLocation));

        //Define Front Perimeter Target 1 position on field.
        OpenGLMatrix FrontPerimeterRedLocation = OpenGLMatrix
                .translation(vumarkDistanceFromWallCenter, mmFTCFieldWidth / 2, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        FrontPerimeterRed.setLocationFtcFieldFromTarget(FrontPerimeterRedLocation);
        RobotLog.ii(TAG, "Front Perimeter Red=%s", format(FrontPerimeterRedLocation));

        //Define Red Perimeter Target 1 position on field.
        OpenGLMatrix RedPerimeterBackLocation = OpenGLMatrix
                .translation(mmFTCFieldWidth / 2, -vumarkDistanceFromWallCenter, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, -90, 0));
        RedPerimeterBack.setLocationFtcFieldFromTarget(RedPerimeterBackLocation);
        RobotLog.ii(TAG, "Red Perimeter Back=%s", format(RedPerimeterBackLocation));

        //Define Red Perimeter Target 2 position on field.
        OpenGLMatrix RedPerimeterFrontLocation = OpenGLMatrix
                .translation(mmFTCFieldWidth / 2, vumarkDistanceFromWallCenter, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, -90, 0));
        RedPerimeterFront.setLocationFtcFieldFromTarget(RedPerimeterFrontLocation);
        RobotLog.ii(TAG, "Red Perimeter Front=%s", format(RedPerimeterFrontLocation));

        //Define Blue Perimeter Target 2 position on field.
        OpenGLMatrix BluePerimeterBackLocation = OpenGLMatrix
                .translation(-mmFTCFieldWidth / 2, -vumarkDistanceFromWallCenter, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, -90, 0));
        BluePerimeterBack.setLocationFtcFieldFromTarget(BluePerimeterBackLocation);
        RobotLog.ii(TAG, "Blue Perimeter Back=%s", format(BluePerimeterBackLocation));

        //Define Blue Perimeter Target 1 position on field.
        OpenGLMatrix BluePerimeterFrontLocation = OpenGLMatrix
                .translation(-mmFTCFieldWidth / 2, vumarkDistanceFromWallCenter, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, -90, 0));
        BluePerimeterFront.setLocationFtcFieldFromTarget(BluePerimeterFrontLocation);
        RobotLog.ii(TAG, "Blue Perimeter Front=%s", format(BluePerimeterFrontLocation));

        //Define Bridge Blue Back position on field.
        OpenGLMatrix BridgeBlueBackLocation = OpenGLMatrix
                .translation(-bridgeVumarkXDistanceFromOrigin, -bridgeVumarkYDistanceFromOrigin, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        bridgeBlueBack.setLocationFtcFieldFromTarget(BridgeBlueBackLocation);
        RobotLog.ii(TAG, "Bridge Blue Back=%s", format(BridgeBlueBackLocation));

        //Define Red Bridge Back position on field.
        OpenGLMatrix BridgeRedBackLocation = OpenGLMatrix
                .translation(bridgeVumarkXDistanceFromOrigin, -bridgeVumarkYDistanceFromOrigin, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        bridgeRedBack.setLocationFtcFieldFromTarget(BridgeRedBackLocation);
        RobotLog.ii(TAG, "Red Bridge Back=%s", format(BridgeRedBackLocation));

        //Define Bridge Blue Front position on field.
        OpenGLMatrix BridgeBlueFrontLocation = OpenGLMatrix
                .translation(-bridgeVumarkXDistanceFromOrigin, bridgeVumarkYDistanceFromOrigin, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 180, 0));
        bridgeBlueFront.setLocationFtcFieldFromTarget(BridgeBlueFrontLocation);
        RobotLog.ii(TAG, "Blue Bridge Front=%s", format(BridgeBlueFrontLocation));

        //Define Red Bridge Front position on field.
        OpenGLMatrix BridgeRedFrontLocation = OpenGLMatrix
                .translation(bridgeVumarkXDistanceFromOrigin, bridgeVumarkYDistanceFromOrigin, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 180, 0));
        bridgeRedFront.setLocationFtcFieldFromTarget(BridgeRedFrontLocation);
        RobotLog.ii(TAG, "Red Bridge Front=%s", format(BridgeRedFrontLocation));


        // Define position of camera in relation to the robot.
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(mmBotWidth / 2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 90, 90, 0));
        RobotLog.ii(TAG, "camera=%s", format(robotFromCamera));

        /**
         * Let the trackable listeners we care about know where the camera is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener) RearPerimeterBlue.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) RearPerimeterRed.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) FrontPerimeterBlue.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) FrontPerimeterRed.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) BluePerimeterBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) BluePerimeterFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) RedPerimeterBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) RedPerimeterFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) bridgeBlueBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) bridgeBlueFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) bridgeRedBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) bridgeRedFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        skystone.activate();

        boolean buttonPressed = false;
        while (opModeIsActive()) {

            if (gamepad1.a && !buttonPressed) {
                captureFrameToFile();
            }
            buttonPressed = gamepad1.a;

            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */

                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }

            float robotXTranslation = lastLocation.getTranslation().get(0);
            float robotYTranslation = lastLocation.getTranslation().get(1);

            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", format(lastLocation));
                telemetry.addData(" ", "");
                telemetry.addData("X-Value: ", robotXTranslation);
                telemetry.addData(" ", "");
                telemetry.addData("Y-Value: ", robotYTranslation);
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
        }
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    public String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    /**
     * Sample one frame from the Vuforia stream and write it to a .PNG image file on the robot
     * controller in the /sdcard/FIRST/data directory. The images can be downloaded using Android
     * Studio's Device File Explorer, ADB, or the Media Transfer Protocol (MTP) integration into
     * Windows Explorer, among other means. The images can be useful during robot design and calibration
     * in order to get a sense of what the camera is actually seeing and so assist in camera
     * aiming and alignment.
     */
    public void captureFrameToFile() {
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>() {
            @Override
            public void accept(Frame frame) {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap != null) {
                    File file = new File(captureDirectory, String.format(Locale.getDefault(), "VuforiaFrame-%d.png", captureCounter++));
                    try {
                        FileOutputStream outputStream = new FileOutputStream(file);
                        try {
                            bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                        } finally {
                            outputStream.close();
                            telemetry.log().add("captured %s", file.getName());
                        }
                    } catch (IOException e) {
                        RobotLog.ee(TAG, e, "exception in captureFrameToFile()");
                    }
                }
            }
        }));
    }

    //Constructor
    public SkystoneFPS() {

    }

    public void initialize(VuforiaLocalizer.Parameters parameters){
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        /**
         * Because this opmode processes frames in order to write them to a file, we tell Vuforia
         * that we want to ensure that certain frame formats are available in the {@link Frame}s we
         * see.
         */
        vuforia.enableConvertFrameToBitmap();

        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
    }

    public void vuMarkInit(double maxTime, float xPosition, float yPosition, VuforiaLocalizer.Parameters parameters) {

        /*
        Define VuMarks.
        Rear/Back is in the Building Zone.
        Front is in the Loading Zone.
         */
        VuforiaTrackables skystone = vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable bridgeBlueBack = skystone.get(1);
        bridgeBlueBack.setName("bridgeBlueBack");

        VuforiaTrackable bridgeRedBack = skystone.get(2);
        bridgeRedBack.setName("bridgeRedBack");

        VuforiaTrackable bridgeRedFront = skystone.get(3);
        bridgeRedBack.setName("bridgeRedFront");

        VuforiaTrackable bridgeBlueFront = skystone.get(4);
        bridgeRedBack.setName("bridgeBlueFront");

        VuforiaTrackable RedPerimeterBack = skystone.get(5);
        bridgeRedBack.setName("RedPerimeterBack");

        VuforiaTrackable RedPerimeterFront = skystone.get(6);
        bridgeRedBack.setName("RedPerimeterFront");

        VuforiaTrackable FrontPerimeterRed = skystone.get(7);
        bridgeRedBack.setName("FrontPerimeterRed");

        VuforiaTrackable FrontPerimeterBlue = skystone.get(8);
        bridgeRedBack.setName("FrontPerimeterBlue");

        VuforiaTrackable BluePerimeterFront = skystone.get(9);
        bridgeRedBack.setName("BluePerimeterFront");

        VuforiaTrackable BluePerimeterBack = skystone.get(10);
        bridgeRedBack.setName("BluePerimeterBack");

        VuforiaTrackable RearPerimeterBlue = skystone.get(11);
        bridgeRedBack.setName("RearPerimeterBlue");

        VuforiaTrackable RearPerimeterRed = skystone.get(12);
        bridgeRedBack.setName("RearPerimeterRed");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(skystone);

        //Convert field measurements to mm because Skystone XML file data uses mm.
        final float mmPerInch = 25.4f;
        final float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
        final float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" (142") center-to-center of the glass panels
        final float vumarkHeight = 5.75f * mmPerInch;
        final float vumarkDistanceFromWallCenter = 35 * mmPerInch;
        final float bridgeVumarkXDistanceFromOrigin = 24 * mmPerInch;
        final float bridgeVumarkYDistanceFromOrigin = 9 * mmPerInch;

        final float xOrigin = 0.0f;
        final float yOrigin = 0.0f;
        final float zOrigin = 0.0f;

        final float X_MAX = mmFTCFieldWidth / 2;
        final float Y_MAX = mmFTCFieldWidth / 2;
        final float Z_MAX = mmFTCFieldWidth / 2;

        final float X_MIN = -mmFTCFieldWidth / 2;
        final float Y_MIN = -mmFTCFieldWidth / 2;
        final float Z_MIN = 0.0f;


        //Define Rear Perimeter Target 1 position on field.
        OpenGLMatrix RearPerimeterBlueLocation = OpenGLMatrix
                .translation(-vumarkDistanceFromWallCenter, -mmFTCFieldWidth / 2, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 180, 0));
        RearPerimeterBlue.setLocationFtcFieldFromTarget(RearPerimeterBlueLocation);
        RobotLog.ii(TAG, "Rear Perimeter Blue=%s", format(RearPerimeterBlueLocation));

        //Define Rear Perimeter Target 2 position on field.
        OpenGLMatrix RearPerimeterRedLocation = OpenGLMatrix
                .translation(vumarkDistanceFromWallCenter, -mmFTCFieldWidth / 2, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 180, 0));
        RearPerimeterRed.setLocationFtcFieldFromTarget(RearPerimeterRedLocation);
        RobotLog.ii(TAG, "Rear Perimeter Red=%s", format(RearPerimeterRedLocation));

        //Define Front Perimeter Target 2 position on field.
        OpenGLMatrix FrontPerimeterBlueLocation = OpenGLMatrix
                .translation(-vumarkDistanceFromWallCenter, mmFTCFieldWidth / 2, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        FrontPerimeterBlue.setLocationFtcFieldFromTarget(FrontPerimeterBlueLocation);
        RobotLog.ii(TAG, "Front Perimeter Blue=%s", format(FrontPerimeterBlueLocation));

        //Define Front Perimeter Target 1 position on field.
        OpenGLMatrix FrontPerimeterRedLocation = OpenGLMatrix
                .translation(vumarkDistanceFromWallCenter, mmFTCFieldWidth / 2, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        FrontPerimeterRed.setLocationFtcFieldFromTarget(FrontPerimeterRedLocation);
        RobotLog.ii(TAG, "Front Perimeter Red=%s", format(FrontPerimeterRedLocation));

        //Define Red Perimeter Target 1 position on field.
        OpenGLMatrix RedPerimeterBackLocation = OpenGLMatrix
                .translation(mmFTCFieldWidth / 2, -vumarkDistanceFromWallCenter, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, -90, 0));
        RedPerimeterBack.setLocationFtcFieldFromTarget(RedPerimeterBackLocation);
        RobotLog.ii(TAG, "Red Perimeter Back=%s", format(RedPerimeterBackLocation));

        //Define Red Perimeter Target 2 position on field.
        OpenGLMatrix RedPerimeterFrontLocation = OpenGLMatrix
                .translation(mmFTCFieldWidth / 2, vumarkDistanceFromWallCenter, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, -90, 0));
        RedPerimeterFront.setLocationFtcFieldFromTarget(RedPerimeterFrontLocation);
        RobotLog.ii(TAG, "Red Perimeter Front=%s", format(RedPerimeterFrontLocation));

        //Define Blue Perimeter Target 2 position on field.
        OpenGLMatrix BluePerimeterBackLocation = OpenGLMatrix
                .translation(-mmFTCFieldWidth / 2, -vumarkDistanceFromWallCenter, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, -90, 0));
        BluePerimeterBack.setLocationFtcFieldFromTarget(BluePerimeterBackLocation);
        RobotLog.ii(TAG, "Blue Perimeter Back=%s", format(BluePerimeterBackLocation));

        //Define Blue Perimeter Target 1 position on field.
        OpenGLMatrix BluePerimeterFrontLocation = OpenGLMatrix
                .translation(-mmFTCFieldWidth / 2, vumarkDistanceFromWallCenter, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, -90, 0));
        BluePerimeterFront.setLocationFtcFieldFromTarget(BluePerimeterFrontLocation);
        RobotLog.ii(TAG, "Blue Perimeter Front=%s", format(BluePerimeterFrontLocation));

        //Define Bridge Blue Back position on field.
        OpenGLMatrix BridgeBlueBackLocation = OpenGLMatrix
                .translation(-bridgeVumarkXDistanceFromOrigin, -bridgeVumarkYDistanceFromOrigin, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        bridgeBlueBack.setLocationFtcFieldFromTarget(BridgeBlueBackLocation);
        RobotLog.ii(TAG, "Bridge Blue Back=%s", format(BridgeBlueBackLocation));

        //Define Red Bridge Back position on field.
        OpenGLMatrix BridgeRedBackLocation = OpenGLMatrix
                .translation(bridgeVumarkXDistanceFromOrigin, -bridgeVumarkYDistanceFromOrigin, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        bridgeRedBack.setLocationFtcFieldFromTarget(BridgeRedBackLocation);
        RobotLog.ii(TAG, "Red Bridge Back=%s", format(BridgeRedBackLocation));

        //Define Bridge Blue Front position on field.
        OpenGLMatrix BridgeBlueFrontLocation = OpenGLMatrix
                .translation(-bridgeVumarkXDistanceFromOrigin, bridgeVumarkYDistanceFromOrigin, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 180, 0));
        bridgeBlueFront.setLocationFtcFieldFromTarget(BridgeBlueFrontLocation);
        RobotLog.ii(TAG, "Blue Bridge Front=%s", format(BridgeBlueFrontLocation));

        //Define Red Bridge Front position on field.
        OpenGLMatrix BridgeRedFrontLocation = OpenGLMatrix
                .translation(bridgeVumarkXDistanceFromOrigin, bridgeVumarkYDistanceFromOrigin, vumarkHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 180, 0));
        bridgeRedFront.setLocationFtcFieldFromTarget(BridgeRedFrontLocation);
        RobotLog.ii(TAG, "Red Bridge Front=%s", format(BridgeRedFrontLocation));


        // Define position of camera in relation to the robot.
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(mmBotWidth / 2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 90, 90, 0));
        RobotLog.ii(TAG, "camera=%s", format(robotFromCamera));

        /**
         * Let the trackable listeners we care about know where the camera is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        ((VuforiaTrackableDefaultListener) RearPerimeterBlue.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) RearPerimeterRed.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) FrontPerimeterBlue.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) FrontPerimeterRed.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) BluePerimeterBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) BluePerimeterFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) RedPerimeterBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) RedPerimeterFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) bridgeBlueBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) bridgeBlueFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) bridgeRedBack.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener) bridgeRedFront.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        skystone.activate();

        float robotXTranslation = 0;
        float robotYTranslation = 0;

        boolean buttonPressed = false;
        double startTime = time;
        while ((time - startTime) < maxTime) {

            if (gamepad1.a && !buttonPressed) {
                captureFrameToFile();
            }
            buttonPressed = gamepad1.a;

            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */

                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }

            robotXTranslation = lastLocation.getTranslation().get(0);
            robotYTranslation = lastLocation.getTranslation().get(1);

            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", format(lastLocation));
                telemetry.addData(" ", "");
                telemetry.addData("X-Value: ", robotXTranslation);
                telemetry.addData(" ", "");
                telemetry.addData("Y-Value: ", robotYTranslation);
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
        }

        xPosition = robotXTranslation;
        yPosition = robotYTranslation;
    }

}
