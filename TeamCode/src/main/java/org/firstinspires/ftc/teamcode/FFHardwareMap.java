package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FFHardwareMap
{
    /* Public OpMode members. */
    //DEFINE MOTORS

    public DcMotor fr;
    public DcMotor fl;
    public DcMotor br;
    public DcMotor bl;

    public DcMotor arm;
    public DcMotor lift;
    public DcMotor carousel;

    //DEFINE SERVOS
    public Servo deposit;
    public Servo colCamPivot;
    public Servo depoCamPivot;
    public CRServo collect;


    //CONSTANTS

    public final double colCamStraightOut = 0.78;
    public final double colCamSideways = 0.26;
    public final double colCamLim[] = {0.16, 1};
    public final double depoCamStraightOut = 0.26;
    public final double depoCamDetectPos[] = {0, 0, 0, 0.175}; //Blue Warehouse, Blue Carousel, Red Warehouse, Red Carousel


    public final double depoUp = 0.223;
    public final double depoLevel = 0.74;
    public final double depoDown = 0.9;
    public final double depoDownAuto = 0.8;
    public final double liftUp = 1900;
    public final int liftTop = 1500;
    public final int liftMid = 775; //825
    public final int liftBot = 220;
    public final double collectUp = 0;
    public final double collectDown = -2500;
//0.17
    public final double carouselSpeed = 0.75;

    public final double minSpeed = 0.15;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public FFHardwareMap(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        fl = hwMap.get(DcMotor.class, "fl");            //CH0
        fr = hwMap.get(DcMotor.class, "fr");            //CH1
        bl = hwMap.get(DcMotor.class, "bl");            //CH2
        br = hwMap.get(DcMotor.class, "br");            //CH3

        arm = hwMap.get(DcMotor.class, "collectArm");   //EH0
        lift = hwMap.get(DcMotor.class, "lift");        //EH1
        carousel = hwMap.get(DcMotor.class, "carousel");//EH2

        collect = hwMap.get(CRServo.class, "collect");  //CH0
        deposit = hwMap.get(Servo.class, "deposit");    //CH3

        colCamPivot = hwMap.get(Servo.class, "colCam"); //EH0
        depoCamPivot = hwMap.get(Servo.class,"depoCam" );//EH2

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);

        //robot.init(hardwareMap);

        deposit.setPosition(depoUp);
        colCamPivot.setPosition(colCamSideways);

    }
}