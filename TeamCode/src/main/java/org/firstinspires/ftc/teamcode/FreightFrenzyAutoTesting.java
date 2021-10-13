
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
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark; <- this is why I need to go through imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.teamcode.FFHardwareMap;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Autonomous(name= "UltGoal_Test", group="14174")
@Disabled  //comment out this line before using
public class FreightFrenzyAutoTesting extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //Declare Sensors
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    OpenCvWebcam webcam;
    StageSwitchingPipeline StageSwitchingPipeline; //used to be lowercase s, idk why so i changed it

    FFHardwareMap robot = new FFHardwareMap();

    //USER GENERATED VALUES//
    double headingResetValue;
    double revolutions;

    //localization
    double[] position = {0, 0}; //x, y
    double[] encoderVals = {0, 0}; //l, r
    double[] lastEncoderVals = {0, 0}; //l, r
    double[] encoderAndAngleDif = {0, 0, 0}; //delta l, delta r, delta a
    double lastAngle = 0;
    double angle = 0;
    double lastAccumulatedAngle = 0;
    double accumulatedAngle = 0;

    double[] target = new double[3]; //x, y, a

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new StageSwitchingPipeline());

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        //CODE FOR SETTING UP AND INITIALIZING IMU
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        robot.init(hardwareMap);

        //Reset Encoders
        idle();

        //Set the Run Mode For The Motors

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
        while (opModeIsActive()) {
            telemetry.addData("Num contours found", StageSwitchingPipeline.getNumContoursFound()); //used to be lowercase s
            telemetry.update();

            localize();
        }
    }

    //FUNCTIONS
    public void localize() {
        //variable updates
        encoderVals[0] = robot.bl.getCurrentPosition();
        encoderVals[1] = robot.br.getCurrentPosition();
        angle = getAbsoluteHeading();
        if (angle - lastAngle < -300) { revolutions++; }
        else if (angle - lastAngle > 300) {revolutions--;}
        accumulatedAngle = getAccumulatedHeading(angle);

        //actual localization
        encoderAndAngleDif[0] = encoderVals[0] - lastEncoderVals[0];
        encoderAndAngleDif[1] = encoderVals[1] - lastEncoderVals[1];
        encoderAndAngleDif[2] = accumulatedAngle - lastAccumulatedAngle;

    }

    public void setMotorPower(double l, double r) {
        robot.fl.setPower(l);
        robot.fr.setPower(r);
        robot.bl.setPower(l);
        robot.br.setPower(r);
    }

    //why does the hub normalize angles by default it is honestly a nightmare istg
    public double getAccumulatedHeading(double angle) {
        return (revolutions*360) + angle;
    }

    //OPENCV SUFFERING
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat50 = new Mat();
        Mat thresholdMat100 = new Mat();
        Mat thresholdMat150 = new Mat();
        Mat thresholdMat200 = new Mat();
        Mat contoursOnFrameMat = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        int numContoursFound;

        enum Stage
        {
            YCbCr_CHAN2,
            THRESHOLD50,
            THRESHOLD100,
            THRESHOLD150,
            THRESHOLD200,
            CONTOURS_OVERLAYED_ON_FRAME,
            RAW_IMAGE,
        }

        private Stage stageToRenderToViewport = Stage.YCbCr_CHAN2;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();

            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat50, 50, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat100, 100, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat150, 150, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat200, 200, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.findContours(thresholdMat100, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            numContoursFound = contoursList.size();
            input.copyTo(contoursOnFrameMat);
            Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 3, 8);

            switch (stageToRenderToViewport)
            {
                case YCbCr_CHAN2:
                {
                    return yCbCrChan2Mat;
                }

                case THRESHOLD50:
                {
                    return thresholdMat50;
                }

                case THRESHOLD100:
                {
                    return thresholdMat100;
                }

                case THRESHOLD150:
                {
                    return thresholdMat150;
                }

                case THRESHOLD200:
                {
                    return thresholdMat200;
                }

                case CONTOURS_OVERLAYED_ON_FRAME:
                {
                    return contoursOnFrameMat;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

        public int getNumContoursFound()
        {
            return numContoursFound;
        }
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
