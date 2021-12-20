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
    public Servo cap;
    public Servo capArm;
    public Servo deposit;
    public Servo gate;
    public CRServo collect;


    public Servo cameraServo;



    //CONSTANTS

    public final double[] cameraServoLimits = {0.17, 1};
    public final double cameraServoCentered = 0.5;

    public final double depoUp = 0.223;
    public final double depoLevel = 0.74;
    public final double depoDown = 0.95;
    public final double liftUp = 2000;
    public final double liftTop = 0;
    public final double liftMid = 0;
    public final double liftBot = 150;
    public final double collectUp = 0;
    public final double collectDown = -2500;
    public final double gateOpen = 0.5;
    public final double gateClosed = 0.94;

    public final double carouselSpeed = 1;

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
        gate = hwMap.get(Servo.class, "gate");          //CH4

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

        gate.setPosition(gateClosed);
        deposit.setPosition(depoUp);

    }
}