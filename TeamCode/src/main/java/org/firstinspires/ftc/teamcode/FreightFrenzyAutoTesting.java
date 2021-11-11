
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark; <- this is why I need to go through imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

//import org.firstinspires.ftc.teamcode.FFHardwareMap;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(name= "FreightFrenzyAutoTesting", group="14174")
//@Disabled  //comment out this line before using
public class FreightFrenzyAutoTesting extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //Declare Sensors
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    OpenCvWebcam depositcam; //EOCV Depo Cam
    WebcamName collectCam;   //Vuforia Collect Cam

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia  = null;
    private VuforiaTrackables targets = null ;

    private boolean targetVisible = false;

    FFHardwareMap robot = new FFHardwareMap();

    //USER GENERATED VALUES//
    double headingResetValue;

    //localization
    double[] position = {0, 0}; //x, y
    double[] lastEncoderVals = {0, 0}; //l, r

    double[][] path1 = {{100, 100, 0.1, 0.75}, {-100, 200, 0.1, 0.75}, {100, 300, 0.1, 0.75}, {150, 250, 0.1, 0.5}, {100, 100, 0.1, 0.5}, {0, 0, 0.1, 0.75}}; //x, y, speedMin, speedMax

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};

    public int valRight;
    public int valMid;
    public int valLeft;

    @Override
    public void runOpMode() throws InterruptedException {

        //EASY OPEN CV INITIALIZATION
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        depositcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "depositCam"), cameraMonitorViewId);
        collectCam = hardwareMap.get(WebcamName.class, "collectCam");

        depositcam.setPipeline(new shippingElementDetection());

        depositcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        depositcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                depositcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        //ADDITIONAL VUFORIA INITIALIZATION
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQdreXP/////AAABmZt6Oecz+kEzpK0JGPmBsiNN7l/NAvoL0zpZPFQAslTHUcNYg++t82d9o6emZcSfRJM36o491JUmYS/5qdxxP235BssGslVIMSJCT7vNZ2iQW2pwj6Lxtw/oqvCLtgGRPxUyVSC1u5QHi+Siktg3e4g9rYzoQ2+kzv2chS8TnNooSoF6YgQh4FXqCYRizfbYkjVWtx/DtIigXy+TrXNn84yXbl66CnjNy2LFaOdBFrl315+A79dEYJ+Pl0b75dzncQcrt/aulSBllkA4f03FxeN3Ck1cx9twVFatjOCFxPok0OApMyo1kcARcPpemk1mqF2yf2zJORZxF0H+PcRkS2Sv92UpSEq/9v+dYpruj/Vr";
        parameters.cameraName = collectCam;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        //CODE FOR SETTING UP AND INITIALIZING IMU
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        robot.init(hardwareMap);

        //DEFINE SENSORS
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);

        //Setup The Telemetry Dashboard
        composeTelemetry();

        //Initilization

        // Wait for the game to start (driver presses PLAY)
        this.headingResetValue = this.getAbsoluteHeading();
        //waitForStart();
        runtime.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status:", "Waiting for start command.");
            telemetry.update();
        };

        runtime.reset();
        targets.activate();

        //AUTONOMOUS
        while (opModeIsActive()) {

            if (valLeft < valMid && valLeft < valRight) {
                telemetry.addData("valLeft:",  "Lowest");
            } else if (valMid < valLeft && valMid < valRight) {
                telemetry.addData("valMid:", "Lowest");
            } else {
                telemetry.addData("valRight:", "Lowest");
            }

            telemetry.addData("valLeft: ", valLeft);
            telemetry.addData("valMid: ", valMid);
            telemetry.addData("valRight: ", valRight);
            telemetry.update();

            OpenGLMatrix vuLocation = readVuforiaPosition(allTrackables);

            sleep(100);
        }
    }

    //Navigation Functions
    public void navigateForward(double[][] targets, double speedMax, double speedMin, double accuracyMultiplier, double timeOut) {
        double distance = 0;
        double speed = Math.cbrt(0.01*distance);
        double heading = getAbsoluteHeading();

        double[] robotOriginXY = {targets[0][0] - position[0], targets[0][1] - position[1]};
        double[] pointVector = {Math.sqrt((robotOriginXY[0]*robotOriginXY[0]) + (robotOriginXY[1]*robotOriginXY[1])), -Math.atan2(robotOriginXY[0], robotOriginXY[1])}; //mag, angle
        double[] localTargetXY = {pointVector[0]*Math.sin(pointVector[1]-heading), pointVector[0]*Math.cos(pointVector[1]-heading)}; //x, y
        double headingError = angleWrap(pointVector[1] - heading);

        double minSpeed = targets[0][2];
        double maxSpeed = targets[0][3];

        double tPower = 0;
        double rPower = 0;

        for (int i = 0; i < targets.length; i++) {
            robotOriginXY[0] = targets[i][0] - position[0];
            robotOriginXY[1] = targets[i][1] - position[1];
            pointVector[0] = Math.sqrt((robotOriginXY[0]*robotOriginXY[0]) + (robotOriginXY[1]*robotOriginXY[1]));
            pointVector[1] = -Math.atan2(robotOriginXY[0], robotOriginXY[1]);
            localTargetXY[0] = pointVector[0]*Math.sin(pointVector[1]-heading);
            localTargetXY[1] = pointVector[0]*Math.cos(pointVector[1]-heading);
            distance = pointVector[0];
            headingError = angleWrap(pointVector[1] - heading);
            minSpeed = targets[0][2];
            maxSpeed = targets[0][3];
            tPower = (maxSpeed-minSpeed)*(Math.pow(0.01*distance, 1/3))+minSpeed;

            while (distance > 30) {
                robotOriginXY[0] = targets[i][0] - position[0];
                robotOriginXY[1] = targets[i][1] - position[1];
                pointVector[0] = Math.sqrt((robotOriginXY[0]*robotOriginXY[0]) + (robotOriginXY[1]*robotOriginXY[1]));
                pointVector[1] = -Math.atan2(robotOriginXY[0], robotOriginXY[1]);
                localTargetXY[0] = pointVector[0]*Math.sin(pointVector[1]-heading);
                localTargetXY[1] = pointVector[0]*Math.cos(pointVector[1]-heading);
                distance = pointVector[0];
                minSpeed = targets[0][2];
                maxSpeed = targets[0][3];
                tPower = Range.clip((maxSpeed-minSpeed)*(Math.pow(0.01*distance, 1/3))+minSpeed, minSpeed, maxSpeed);

                headingError = angleWrap(pointVector[1] - heading);
                rPower = (1-tPower)*Range.clip(Math.pow(0.1*headingError, 1/1.8) ,1,1);

                setMotorSpeed(tPower-rPower, tPower+rPower);
            }

            i++;
        }
        setMotorSpeed(0, 0);
    }

    public void forward(double distance, double speedMod, double error, double timeout) {

    }

    public void turnTest(double target, double speedPercent) {
        double error = Math.toDegrees(angleWrap(Math.toRadians(target - getAbsoluteHeading())));
        final double startError = Math.abs(error);
        while (Math.abs(error) > 4) {
            error = Math.toDegrees(angleWrap(Math.toRadians(target - getAbsoluteHeading())));
            setMotorSpeed((error/startError)*speedPercent, -(error/startError)*speedPercent);
        }
    }

    //Helper Functions
    public void setMotorSpeed(double l, double r) {
        robot.fl.setPower(l);
        robot.fr.setPower(r);
        robot.bl.setPower(l);
        robot.br.setPower(r);
    }

    public void localize() {
        double[] encoderVals = {robot.bl.getCurrentPosition(), robot.br.getCurrentPosition()}; //l, r
        double[] encoderDif = {encoderVals[0] - lastEncoderVals[0],encoderVals[1] - lastEncoderVals[1]}; //delta l, delta r, delta a
        double averageDif = (encoderDif[0] + encoderDif[1])/2;

        position[0] = position[0] + (averageDif * -Math.sin(getAbsoluteHeading()));
        position[1] = position[1] + (averageDif * Math.cos(getAbsoluteHeading()));

        lastEncoderVals[0] = encoderVals[0];
        lastEncoderVals[1] = encoderVals[1];
    }

    //OPENCV SUFFERING
    class shippingElementDetection extends OpenCvPipeline
    {
        Mat yCbCr = new Mat();
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat contoursOnFrameMat = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        int numContoursFound;
        Mat greyScale = new Mat();

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            Imgproc.cvtColor(input, yCbCr, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(yCbCr, yCbCrChan2Mat, 2);

            //get values from frame
            double[] pixMid = yCbCrChan2Mat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = yCbCrChan2Mat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = yCbCrChan2Mat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(yCbCrChan2Mat, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(yCbCrChan2Mat, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(yCbCrChan2Mat, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            /*
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 100, 150, Imgproc.THRESH_BINARY_INV);
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            numContoursFound = contoursList.size();
            input.copyTo(contoursOnFrameMat);
            Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 3, 8);

            return contoursOnFrameMat;
             */

            return yCbCrChan2Mat;
        }

        public int getNumContoursFound()
        {
            return numContoursFound;
        }
    }

    //Vuforia Functions
    OpenGLMatrix readVuforiaPosition(List<VuforiaTrackable> allTrackables) {
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();

        return lastLocation;
    }

    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }


    //FUNCTIONS NEEDED BY THE GYRO
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private double getAbsoluteHeading(){
        return this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private double getRelativeHeading(){
        return this.getAbsoluteHeading() - this.headingResetValue;
    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2*Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2*Math.PI;
        }
        return radians;
    }

    public void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the n    ecessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
}
