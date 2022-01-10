
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
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

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(name= "Blue Carousel", group="14174")
//@Disabled  //comment out this line before using
public class BlueCarousel extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //Declare Sensors
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    OpenCvWebcam depositcam; //EOCV Depo Cam
    WebcamName collectCam;   //Vuforia Collect Cam

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_DM.tflite";
    private static final String[] LABELS = {
            "Duck",
            "Marker"
    };

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    private VuforiaLocalizer vuforia  = null;
    private TFObjectDetector tfod;

    FFHardwareMap robot = new FFHardwareMap();

    //USER GENERATED VALUES//
    double headingResetValue;

    //localization
    double[] position = {0, 0}; //x, y
    double[] lastEncoderVals = {0, 0}; //l, r

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {2f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};

    public int valRight;
    public int valMid;

    char pathDir;

    double scanDirection = -1;

    private int shippingElementPlacement = -1; //-1 is pre-detection; 0: left-bottom; 1: mid-mid; 2: right-top

    @Override
    public void runOpMode() throws InterruptedException {

        //EASY OPEN CV INITIALIZATION
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);
        depositcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "depositCam"), viewportContainerIds[1]);
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
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[0]);
        parameters.vuforiaLicenseKey = "AQdreXP/////AAABmZt6Oecz+kEzpK0JGPmBsiNN7l/NAvoL0zpZPFQAslTHUcNYg++t82d9o6emZcSfRJM36o491JUmYS/5qdxxP235BssGslVIMSJCT7vNZ2iQW2pwj6Lxtw/oqvCLtgGRPxUyVSC1u5QHi+Siktg3e4g9rYzoQ2+kzv2chS8TnNooSoF6YgQh4FXqCYRizfbYkjVWtx/DtIigXy+TrXNn84yXbl66CnjNy2LFaOdBFrl315+A79dEYJ+Pl0b75dzncQcrt/aulSBllkA4f03FxeN3Ck1cx9twVFatjOCFxPok0OApMyo1kcARcPpemk1mqF2yf2zJORZxF0H+PcRkS2Sv92UpSEq/9v+dYpruj/Vr";
        parameters.cameraName = collectCam;
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //initTfod();

        //if (tfod != null) {
            //tfod.activate();
            //telemetry.addData("Tfod: ", "Activated");
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            //tfod.setZoom(1, 16.0/9.0);
        //}

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

        robot.depoCamPivot.setPosition(robot.depoCamStraightOut);

        // Wait for the game to start (driver presses PLAY)
        this.headingResetValue = this.getAbsoluteHeading();
        //waitForStart();
        runtime.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("valMid: ", valMid);
            telemetry.addData("valRight: ", valRight);
            telemetry.addData("Status:", "Waiting for start command.");
            telemetry.update();
        };

        runtime.reset();

        //AUTONOMOUS
        while (opModeIsActive()) {

            if (valRight < 80) {
                telemetry.addData("position:", "Right");
                robot.lift.setTargetPosition(robot.liftTop);
                pathDir = 'l';
            } else if (valMid < 80){
                telemetry.addData("position:", "Mid");
                robot.lift.setTargetPosition(robot.liftMid);
                pathDir = 'r';
            } else {
                telemetry.addData("position:", "Left");
                robot.lift.setTargetPosition(robot.liftBot);
                pathDir = 'r';
            }

            telemetry.addData("valMid: ", valMid);
            telemetry.addData("valRight: ", valRight);

            depositcam.closeCameraDevice();

            if (pathDir == 'l') { //This goes straight to the hub
                driveStraight(1600, 0.7, 10, 4); //Drives straight forward
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(0.8);
                sleep(50);
                arcTurn(45, 0.7, 1, 'l'); //Turns toward the hub
                sleep(50);
                robot.deposit.setPosition(robot.depoDown);
                sleep(50);
                driveStraight(200, 0.7, 10, 4); //drives to the hub
                sleep(50);
                driveStraight(-1950, 0.9, 10, 4); //reverses toward to the carousel
                robot.deposit.setPosition(robot.depoUp);
                robot.lift.setTargetPosition(0);
                sleep(50);
                arcTurn(90, 0.7, 1, 'r'); //turns to be parallel with the alliance wall
                robot.lift.setPower(0);
                sleep(100);
                driveStraight(-1100, 0.8, 10, 4); //drives into the carousel
                sleep(50);
                robot.carousel.setPower(0.5);
                sleep(3000);
                robot.carousel.setPower(0);
                sleep(50);
                arcTurn(0, 1, 1, 'r'); //turns to be parallel with the back
                sleep(50);
                driveStraight(200, 0.7, 10, 2); //drives toward the storage square
                sleep(50);
                arcTurn(-90, 1, 1, 'r'); //turns into storage with deposit facing the wall
                sleep(50);
            } else { //this goes around the two shipping element spots closest to the hub
                arcTurn(-40, 1, 1, 'r'); //turns away from the hub
                sleep(50);
                driveStraight(2500,  1, 10, 6); //drives past the barcode
                sleep(50);
                arcTurn(90, 1, 1, 'l'); //turns to face the hub
                sleep(50);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(0.8);
                robot.deposit.setPosition(robot.depoDownAuto);
                driveStraight(1100, 0.8, 10, 4); //drives to the hub
                sleep(50);
                driveStraight(-1100, 1, 10, 4); //backs away from the hub
                robot.lift.setTargetPosition(0);
                robot.deposit.setPosition(robot.depoUp);
                sleep(50);
                arcTurn(0, 1, 1, 'l'); //turns to be parallel with the storage wall
                sleep(50);
                driveStraight(-1450, 0.8, 10, 3); //drives toward the blue alliance wall
                sleep(50);
                arcTurn(80, 0.9, 1, 'r'); //turns into the carousel
                sleep(50);
                robot.carousel.setPower(0.5);
                sleep(3000);
                robot.carousel.setPower(0);
                sleep(1000);
                arcTurn(0, 1, 1, 'r'); //turns to drive to the storage
                sleep(1000);
                driveStraight(150, 0.8, 10, 4); //drives to storage
                sleep(1000);
                arcTurn(-90, 1, 1, 'r'); //turns into storage
            }



            /*
            if(getRuntime() > 20 && !EOCVstopped) {
                depositcam.closeCameraDevice();
                tfod.activate();
            }

            while(getRuntime()<10 && whileChecks()) {
                List<Recognition> updatedRecognitions = tfod.getRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                    }
                }
                telemetry.addData("runtime: ", getRuntime());
                telemetry.update();
            }

            duckyDrive();
            sleep(1000);
             */
            stop();
        }
    }

    public void duckyDrive () {
        boolean turnFinished = false;
        while (!turnFinished && whileChecks()) {
            double relAng = duckyRelAngle();
            if (relAng == 404) {
                duckyScan();
            } else if (Math.abs(relAng)>5){
                double turnSpeed = 0;
                if(relAng > 0) {turnSpeed = 0.2;}
                else {turnSpeed = -0.2;}
                setMotorSpeed(turnSpeed, -turnSpeed);
            } else {
                setMotorSpeed(0, 0);
                turnFinished = true;
            }
        }
        driveStraight(700, 0.4, 40, 8);
    }

    public double duckyRelAngle () {
        int index = -1;
        List<Recognition> updatedRecognitions = tfod.getRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            for (int i = 0; i < updatedRecognitions.size(); i++) {
                if (updatedRecognitions.get(i).getLabel() == "Duck") {
                    index = i;
                }
            }
        }
        if (index == -1) {
            duckyScan();
            return 404;
        } else {
            double centerPix = (updatedRecognitions.get(index).getLeft() + updatedRecognitions.get(index).getRight())/2;
            double offset = centerPix - 275;
            robot.colCamPivot.setPosition(Range.clip(robot.colCamPivot.getPosition() + (offset/160000), 0, 1));
            return (robot.colCamPivot.getPosition()*190)-95;
        }
    }

    public void duckyScan() {
        int index = -1;
        List<Recognition> updatedRecognitions = tfod.getRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            for (int i = 0; i < updatedRecognitions.size(); i++) {
                if (updatedRecognitions.get(i).getLabel() == "Duck") {
                    index = i;
                }
            }
        }
        if (index == -1) {
            if(robot.colCamPivot.getPosition() < 0.05 || robot.colCamPivot.getPosition() > 0.95) {
                scanDirection = scanDirection * -1;
            }
            robot.colCamPivot.setPosition(robot.colCamPivot.getPosition() + 0.0002*scanDirection);
        } else {
            return;
        }
    }

    public void localizedDrive(double xTarget, double yTarget, double speedPercent, double error, double time) {
        double desiredHeading = Math.atan2(yTarget-position[1], xTarget-position[0]);
        turnTest(desiredHeading, speedPercent);
        sleep(100);
        driveStraight(Math.sqrt(((xTarget-position[0])*(xTarget-position[0]))+((yTarget-position[1])*(yTarget-position[1]))), speedPercent, error, time);
    }

    public void driveStraight (double duration, double speedPercent, double error, double time) {
        double position = robot.fr.getCurrentPosition();
        double target = position + duration;
        double heading = getAbsoluteHeading();
        double distanceToTargetStart = Math.abs(target - position);
        double distanceToTarget = target - position;
        double percentToTarget = distanceToTarget/distanceToTargetStart;
        double speed = 0;
        double[] wheelSpeed = new double[2]; //l, r
        double turnSpeed = ((heading - getAbsoluteHeading())/20);
        double startTime = getRuntime();

        while (Math.abs(distanceToTarget) > error && !isStopRequested() && getRuntime() < startTime + time) {
            position = robot.fr.getCurrentPosition();
            distanceToTarget = target - position;
            percentToTarget = distanceToTarget/distanceToTargetStart;
            if (percentToTarget >= 0) {
                speed = ((-((percentToTarget-1)*(percentToTarget-1)*(percentToTarget-1)*(percentToTarget-1))+1)*speedPercent)*0.9;
                if (Math.abs(speed) < robot.minSpeed) {speed = robot.minSpeed;}
            }else if (percentToTarget < 0) {
                speed = -((-((percentToTarget+1)*(percentToTarget+1)*(percentToTarget+1)*(percentToTarget+1))+1)*speedPercent)*0.9;
                if (Math.abs(speed) < robot.minSpeed) {speed = -robot.minSpeed;}
            }

            turnSpeed = -((heading - getAbsoluteHeading())/20);

            telemetry.addData("DaS:", distanceToTargetStart);
            telemetry.addData("Target:", target);
            telemetry.addData("Position:", position);
            telemetry.addData("Distance:", distanceToTarget);
            telemetry.addData("Percent:", percentToTarget);
            telemetry.addData("Speed:", speed);
            telemetry.addData("Turn Speed:", turnSpeed);
            telemetry.update();

            wheelSpeed[0] = Range.clip(speed, -0.9, 0.9) - Range.clip(turnSpeed, -0.1, 0.1);
            wheelSpeed[1] = Range.clip(speed, -0.9, 0.9) + Range.clip(turnSpeed, -0.1, 0.1);

            setMotorSpeed(wheelSpeed[0], wheelSpeed[1]);
            localize();
        };

        setMotorSpeed(0, 0);
    };

    public void turn (double target, double speedPercent, double error) {
        double DaS = Math.abs(AngleUnit.normalizeDegrees(target - getAbsoluteHeading()));
        double distance = AngleUnit.normalizeDegrees(target - getAbsoluteHeading());
        double percent = distance/DaS;

        double speed = 0;
        double[] wheelSpeed = new double[4]; //fl, fr, bl, br

        while (Math.abs(distance) > error && !isStopRequested()) {
            distance = AngleUnit.normalizeDegrees(target - getAbsoluteHeading());
            percent = distance/DaS;

            speed = speedPercent*(0.9*(Math.cbrt(percent)));

            wheelSpeed[0] = -speed;
            wheelSpeed[1] = speed;


            setMotorSpeed(wheelSpeed[0], wheelSpeed[1]);
        }
        wheelSpeed[0] = 0;
        wheelSpeed[1] = 0;

        setMotorSpeed(wheelSpeed[0], wheelSpeed[1]);
    };

    public void turnTest(double target, double speedPercent) {
        double error = Math.toDegrees(angleWrap(Math.toRadians(target - getAbsoluteHeading())));
        final double startError = Math.abs(error);
        while (Math.abs(error) > 4 && whileChecks()) {
            error = Math.toDegrees(angleWrap(Math.toRadians(target - getAbsoluteHeading())));
            setMotorSpeed((error/startError)*speedPercent, -(error/startError)*speedPercent);
            localize();
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
        double[] encoderVals = {robot.bl.getCurrentPosition(), robot.fr.getCurrentPosition()}; //l, r
        double[] encoderDif = {encoderVals[0] - lastEncoderVals[0],encoderVals[1] - lastEncoderVals[1]}; //delta l, delta r, delta a
        double averageDif = (encoderDif[0] + encoderDif[1])/2;

        position[0] = position[0] + (averageDif * -Math.sin(Math.toRadians(getAbsoluteHeading())));
        position[1] = position[1] + (averageDif * Math.cos(Math.toRadians(getAbsoluteHeading())));

        lastEncoderVals[0] = encoderVals[0];
        lastEncoderVals[1] = encoderVals[1];
    }

    public void arcTurn (double target, double speedPercent, double error, char side) {
        double DaS = Math.abs(AngleUnit.normalizeDegrees(target - getAbsoluteHeading()));
        double distance = AngleUnit.normalizeDegrees(target - getAbsoluteHeading());
        double percent = distance/DaS;

        double speed = 0;
        double[] wheelSpeed = new double[4]; //fl, fr, bl, br

        while (Math.abs(distance) > error && !isStopRequested()) {
            distance = AngleUnit.normalizeDegrees(target - getAbsoluteHeading());
            percent = distance/DaS;

            if (percent >= 0) {
                speed = speedPercent*(0.8*(Math.sqrt(percent)));
                if(Math.abs(speed) < robot.minSpeed + 0.075) {
                    speed = robot.minSpeed+0.075;
                }
            } else {
                speed = speedPercent*(0.8*(-Math.sqrt(Math.abs(percent))));
                if(Math.abs(speed) < robot.minSpeed + 0.075) {
                    speed = -robot.minSpeed-0.075;
                }
            }

            if (side == 'l') {
                wheelSpeed[0] = speed;
                wheelSpeed[1] = 0;
            } else {
                wheelSpeed[0] = 0;
                wheelSpeed[1] = -speed;
            }

            localize();
            setMotorSpeed(wheelSpeed[0], wheelSpeed[1]);
            telemetry.addData("Arc turning to:", target);
            telemetry.addData("DaS:", DaS);
            telemetry.addData("distance:", distance);
            telemetry.addData("percent:", percent);
            telemetry.addData("speed:", speed);
            telemetry.update();
        }
        wheelSpeed[0] = 0;
        wheelSpeed[1] = 0;
        wheelSpeed[2] = 0;
        wheelSpeed[3] = 0;

        setMotorSpeed(wheelSpeed[0], wheelSpeed[1]);
    };

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

            double[] pixRight = yCbCrChan2Mat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(yCbCrChan2Mat, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
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

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
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

    boolean whileChecks() {
        return !isStopRequested() && opModeIsActive();
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
