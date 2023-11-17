/* Copyright (c) 2023 FIRST. All rights reserved.
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

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;



import java.util.List;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.qualcomm.robotcore.hardware.DistanceSensor;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous(name="RR Red BD", group = "Concept")
//@Disabled

public class AutoRedBackDrop extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 5.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    //final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double STRAFE_GAIN =  0.015 ;  //Set as example
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    final double STRAFE_RIGHT = 90;
    final double STRAFE_LEFT = -90;

    private Boolean INITAPRILTAG = false;
    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private  MecanumDrive drive = null;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static  int DESIRED_TAG_ID = 4;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private boolean isStrafe         = true;
    IMU imu;
    private double  targetHeading = 0;
    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;
    private boolean redBackdrop = true;
    private ElapsedTime runtime = new ElapsedTime();

    // private List<Recognition>   currentRecognitions = null;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/circularprop0911.tflite";
    private static final String[] LABELS = {
            "circprop"
    };

    /**
     * The variable to store our instance of the vision portal.
     */


    boolean targetFound     = false;    // Set to true when an AprilTag target is detected

    private double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    private  double  turn            = 0;        // Desired turning power/speed (-1 to +1)
    private double rangeError = 0;
    private double yawError = 0;
    private boolean propDetected = false;
    private DistanceSensor distance;

    private int loop = 0;

    @Override public void runOpMode()
    {

        double heading;



        // Initialize the Apriltag Detection process
        //  initAprilTag();

        runtime.reset();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BackLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BackRight");
        distance = hardwareMap.get(DistanceSensor.class, "distance1");
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

       /* if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
*/      YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        orientation.getYaw(AngleUnit.DEGREES);


        //  initTfod();
        // initAprilTag();
        initDoubleVision(); // inistialize Tensorflow and AprilTag and adds in Processes

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to start OpMode");
        // telemetry.update();

        visionPortal.setProcessorEnabled(tfod, true);
        imu.resetYaw() ;
        waitForStart();

        while (opModeIsActive())
        {
            while(propDetected==false) {


                propTfod();
                //sleep(20);
            }


          /*  moveRobot(0,0,0);

            // Initialize the Apriltag Detection process


            if(INITAPRILTAG==false) {
                visionPortal.setProcessorEnabled(tfod, false);
                sleep(20);
                visionPortal.setProcessorEnabled(aprilTag, true);

                sleep(20);
                targetFound = false;
                desiredTag = null;

                // Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    if ((detection.metadata != null) &&
                            ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                        targetFound = true;
                        // INITAPRILTAG = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                    }
                }

                // Tell the driver what we see, and what to do.
                if (targetFound && desiredTag.ftcPose.range - DESIRED_DISTANCE>=DESIRED_DISTANCE) {
                    //targetFound=false;
                    // INITAPRILTAG = true;
                    telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
                    telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                    telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                    telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                    telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                    double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double  headingError    = desiredTag.ftcPose.bearing;
                    double  yawError        = desiredTag.ftcPose.yaw;


                } else {
                    telemetry.addData(">", "Drive using joysticks to find valid target\n");


                    INITAPRILTAG=true;

                }

            }
            // while(desiredTag.ftcPose.range - DESIRED_DISTANCE>=DESIRED_DISTANCE ) {

            //}
            moveRobot(0, 0, 0);
            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .

            // Determine he ading, rangeand Yaw (tag image rotation) error so we can use them to control the robot automatically.
          /*  rangeError      = 0; // move forward + move backward -
            headingError    = getRawHeading(); // Bearing to correct heading
            yawError        = -90; // Rotation around Z axis


            // headingError = getSteeringCorrection(turn, TURN_GAIN);
            runtime.reset();
         /*   if (redBackdrop == true && propDetected == false) {
                while ((runtime.milliseconds() < 1000)) {

                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                    if (distance.getDistance(DistanceUnit.CM) >= 45 && distance.getDistance(DistanceUnit.CM) <= 58) {
                        telemetry.addData("distance", "%.0f", distance.getDistance(DistanceUnit.CM));
                        telemetry.update();
                        break;
                    }
                    telemetry.update();
                    moveRobot(drive, strafe, turn);
                    headingError = -getRawHeading();

                }
            }

            moveRobot(0, 0, 0);

            //telemetry.addData("distance after strafing complete ", "%.0f", distance.getDistance(DistanceUnit.CM));
            telemetry.update();

           */

        }



    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public double getRawHeading() {
        YawPitchRollAngles angles   = imu.getRobotYawPitchRollAngles();//imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return  angles.getYaw(AngleUnit.DEGREES);

    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }




    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }

    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 800));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        tfod = new TfodProcessor.Builder()
                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(1280, 800))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }





    }
    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private void propTfod() {
        // List<Recognition> currentRecognitions = null;
        // while (runtime.seconds()<=2)
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        // sleep(5000);
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        telemetry.update();


        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.update();
            //sleep(5000);


            if (x>500 && x<750) { // Prop Middle Position

                telemetry.addData("# Middle Position", currentRecognitions.size());
                telemetry.update();



                Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(5, -4))
                        .build());

                Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToX(26)
                        .build());

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .lineToX(18)
                                .build());

                /*Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                               // .splineTo(new Vector2d(18,-36), Math.toRadians(0))
                                .turnTo(Math.toRadians(-90))
                                .build());*/

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineTo(new Vector2d(22,-30), Math.toRadians(-90))
                                .build());


                propDetected = true;
                DESIRED_TAG_ID = 5;


            }
            /*
            else if (x>900 && x<1280) {
                telemetry.addData("# Right Position", currentRecognitions.size());
                telemetry.update();
                // This code is to strafe right
                rangeError      = 0; // move forward + move backward -
                headingError    = getRawHeading(); // Bearing to correct heading
                yawError        = STRAFE_RIGHT; // Rotation around Z axis
                //The code below is to strafe to the prop
                drive = Range.clip(rangeError * SPEED_GAIN, -0.3, 0.3);
                turn = Range.clip(headingError * TURN_GAIN, -0.05, 0.05);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -0.3, 0.3);
                //while((distance.getDistance(DistanceUnit.CM) <= 45 || distance.getDistance(DistanceUnit.CM) >= 58))
                moveRobot(drive, strafe, turn);
                sleep(800);
                moveRobot(0,0,0);
                //The code below is for moving forward to the prop
                rangeError      = distance.getDistance(DistanceUnit.CM)-18; // move forward + move backward -
                headingError    = 0; // Bearing to correct heading
                yawError        = 0; // Rotation around Z axis
                drive = Range.clip(rangeError * SPEED_GAIN, -0.2, 0.2);
                turn = Range.clip(headingError * TURN_GAIN, -0.1, 0.1);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                // headingError = getSteeringCorrection(turn, TURN_GAIN);
                // runtime.reset();

                //  while (distance.getDistance(DistanceUnit.CM) > 20){

                moveRobot(drive,strafe,turn); //Move forward
                sleep(1500);
                moveRobot(-drive,strafe,turn); //Move backward
                sleep(1000);

                rangeError      = 0; // move forward + move backward -
                headingError    = getRawHeading(); // Bearing to correct heading
                yawError        = STRAFE_RIGHT; // Rotation around Z axis
                //The code below is to strafe to the prop
                drive = Range.clip(rangeError * SPEED_GAIN, -0.3, 0.3);
                turn = Range.clip(headingError * TURN_GAIN, -0.05, 0.05);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -0.3, 0.3);
                //while((distance.getDistance(DistanceUnit.CM) <= 45 || distance.getDistance(DistanceUnit.CM) >= 58))
                moveRobot(drive, strafe, turn);
                sleep(1400);

                rangeError      = 0; // move forward + move backward -
                headingError    = -75; // Bearing to correct heading
                yawError        = 0; // Rotation around Z axis
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                // headingError = getSteeringCorrection(turn, TURN_GAIN);
                // runtime.reset();

                //  while (distance.getDistance(DistanceUnit.CM) > 20){

                moveRobot(drive,strafe,turn); //Move forward
                sleep(500);


                //}
                moveRobot(0,0,0);

                propDetected = true;
                DESIRED_TAG_ID = 6;
            } else if (x>100 && x<300)

            {
                propDetected = false;
                //loop = 100;
            }
            /*
                telemetry.addData("# Left Position", currentRecognitions.size());
                telemetry.update();
                //sleep(5000);
                rangeError      = 0; // move forward + move backward -
                headingError    = getRawHeading(); // Bearing to correct heading
                yawError        = STRAFE_RIGHT; // Rotation around Z axis

                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                moveRobot(drive, strafe, turn);
                sleep(600);

                //This code is to move forward
                rangeError      = 100; // move forward + move backward -
                headingError    = 0; // Bearing to correct heading
                yawError        = 0; // Rotation around Z axis

                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                moveRobot(drive, strafe, turn);
                sleep(800);
                moveRobot(0,0,0);

                //This code is to turn
                rangeError      = 0; // move forward + move backward -
                headingError    = 90; // Bearing to correct heading
                yawError        = 0; // Rotation around Z axis

                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                moveRobot(drive, strafe, turn);
                sleep(2500);

                //move forward near prop
                moveRobot(0,0,0);
                rangeError      = 10; // move forward + move backward -
                headingError    = 0; // Bearing to correct heading
                yawError        = 0; // Rotation around Z axis

                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                moveRobot(drive, strafe, turn);
                sleep(300);
                moveRobot(-drive,strafe,turn);
                sleep(500);

                //This code is to turn
                rangeError      = 0; // move forward + move backward -
                headingError    = -90; // Bearing to correct heading
                yawError        = 0; // Rotation around Z axis

                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                moveRobot(drive, strafe, turn);
                sleep(2000);

                moveRobot(0,0,0);

                propDetected = true;
                DESIRED_TAG_ID =4;
            }



            sleep(200);
            moveRobot(0, 0, 0);
            sleep(1000);
        }   // end for() loop

        if (propDetected == false && loop == 200)

        {
            telemetry.addData("# Left Position", currentRecognitions.size());
            telemetry.update();
            //sleep(5000);
            rangeError      = 0; // move forward + move backward -
            headingError    = getRawHeading(); // Bearing to correct heading
            yawError        = STRAFE_RIGHT; // Rotation around Z axis

            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            moveRobot(drive, strafe, turn);
            sleep(500);

            //This code is to move forward
            rangeError      = 30; // move forward + move backward -
            headingError    = 0; // Bearing to correct heading
            yawError        = 0; // Rotation around Z axis

            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            moveRobot(drive, strafe, turn);
            sleep(800);
            moveRobot(0,0,0);

            //This code is to turn
            rangeError      = 0; // move forward + move backward -
            headingError    = 90; // Bearing to correct heading
            yawError        = 0; // Rotation around Z axis

            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            moveRobot(drive, strafe, turn);
            sleep(1250);

            //move forward near prop
            moveRobot(0,0,0);
            rangeError      = 12; // move forward + move backward -
            headingError    = 0; // Bearing to correct heading
            yawError        = 0; // Rotation around Z axis

            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            moveRobot(drive, strafe, turn);
            sleep(300);

            //Reserve robot
            //move forward near prop
            // moveRobot(0,0,0);

            moveRobot(-drive, strafe, turn);
            sleep(600);

            //move forward near prop
            moveRobot(0,0,0);
            rangeError      = 0; // move forward + move backward -
            headingError    = -180; // Bearing to correct heading
            yawError        = 0; // Rotation around Z axis

            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            moveRobot(drive, strafe, turn);
            sleep(3000);
            moveRobot(0,0,0);

            propDetected = true;
        }*/

        }   // end method telemetryTfod()

        /**
         * Initialize the TensorFlow Object Detection processor.
         */


    }
}