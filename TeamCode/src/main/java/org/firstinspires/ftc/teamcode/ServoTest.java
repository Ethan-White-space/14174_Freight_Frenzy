package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Servo Test", group="14174")
//@Disabled
public class ServoTest extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Servo Servo1;
    Servo Servo2;
    Servo Servo3;
    Servo Servo4;
    //CRServo CRServo1;
    //Servo kicker;
    //DcMotor Flywheel;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Servo1 = hardwareMap.servo.get("deposit");
        Servo2 = hardwareMap.servo.get("capArm");
        Servo3 = hardwareMap.servo.get("cap");
        Servo4 = hardwareMap.servo.get("gate");
        //CRServo1 = hardwareMap.crservo.get("slide");

        //Servo1.setPosition(0.7);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //Servo2.setDirection(Servo.Direction.FORWARD);
        Servo1.setDirection(Servo.Direction.FORWARD);
        //Servo1.setPosition(0.5);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.a) {
                Servo1.setPosition(Servo1.getPosition() + .0001);
                telemetry.addData("Servo1 Position", Servo1.getPosition());
                telemetry.addData("Direction", Servo1.getDirection());
                telemetry.update();
            }
            if (gamepad1.b) {
                Servo1.setPosition(Servo1.getPosition() - .0001);
                telemetry.addData("Servo1 Position", Servo1.getPosition());
                telemetry.addData("Direction", Servo1.getDirection());
                telemetry.update();
            }
            if (gamepad1.x) {
                Servo2.setPosition(Servo2.getPosition() + .0001);
                telemetry.addData("Servo2 Position", Servo2.getPosition());
                telemetry.addData("Direction", Servo2.getDirection());
                telemetry.update();
            }
            if (gamepad1.y) {
                Servo2.setPosition(Servo2.getPosition() - .0001);
                telemetry.addData("Servo2 Position", Servo2.getPosition());
                telemetry.addData("Direction", Servo2.getDirection());
                telemetry.update();
            }
            if (gamepad1.dpad_up) {
                Servo3.setPosition(Servo3.getPosition() + .0001);
                telemetry.addData("Servo3 Position", Servo3.getPosition());
                telemetry.addData("Direction", Servo3.getDirection());
                telemetry.update();
            }
            if (gamepad1.dpad_down) {
                Servo3.setPosition(Servo3.getPosition() - .0001);
                telemetry.addData("Servo3 Position", Servo3.getPosition());
                telemetry.addData("Direction", Servo3.getDirection());
                telemetry.update();
            }
            if (gamepad1.dpad_left) {
                Servo4.setPosition(Servo4.getPosition() + .0001);
                telemetry.addData("Servo4 Position", Servo4.getPosition());
                telemetry.addData("Direction", Servo4.getDirection());
                telemetry.update();
            }
            if (gamepad1.dpad_right) {
                Servo4.setPosition(Servo4.getPosition() - .0001);
                telemetry.addData("Servo4 Position", Servo4.getPosition());
                telemetry.addData("Direction", Servo4.getDirection());
                telemetry.update();
            }
        }


            /*if(gamepad1.x) {
                Servo2.setPosition(Servo2.getPosition() + .001);
                telemetry.addData("Servo2 Position", Servo2.getPosition());
                telemetry.addData("Direction", Servo2.getDirection());
                telemetry.update();
            }
            if(gamepad1.y) {
                Servo2.setPosition(Servo2.getPosition() - .001);
                telemetry.addData("Servo2 Position", Servo2.getPosition());
                telemetry.addData("Direction", Servo2.getDirection());
                telemetry.update();
            }*/

            /*
            if(gamepad1.a) {
                CRServo1.setPower(1);
            } else if(gamepad1.b) {
                CRServo1.setPower(-1);
            } else {
                CRServo1.setPower(0);
            }
             */
        }
    }
