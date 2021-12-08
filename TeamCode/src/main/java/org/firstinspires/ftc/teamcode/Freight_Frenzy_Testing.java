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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="TeleOp", group="Linear Opmode")
//@Disabled
public class Freight_Frenzy_Testing extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    FFHardwareMap robot = new FFHardwareMap();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        double forward = 0;
        double turning = 0;
        double lSpeed = 0;
        double rSpeed = 0;

        //buttons
        boolean testMode = false;
        boolean leftBumperLastState = false;

        boolean dpadUpLS = false;
        boolean dpadRightLS = false;
        boolean dpadDownLS = false;

        boolean rightBumperLS = false;
        boolean rightTriggerLS = false;

        double liftTargetIndex = 0;
        boolean capArmTarget = true;
        boolean clawState = false; //false is closed true is open

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad2.left_bumper && !leftBumperLastState) {
                testMode = !testMode;
                leftBumperLastState = true;
            } else if (!gamepad2.left_bumper) {
                leftBumperLastState = false;
            }

            if (!testMode) {
                //NORMAL RUN MODE
                //LIFT
                if (gamepad2.dpad_up && !dpadUpLS) {
                    liftTargetIndex = 2;
                    dpadUpLS = true;
                } else if (!gamepad2.dpad_up) {
                    dpadUpLS = false;
                }

                if (gamepad2.dpad_right && !dpadRightLS) {
                    liftTargetIndex = 1;
                    dpadRightLS = true;
                } else if (!gamepad2.dpad_right) {
                    dpadRightLS = false;
                }

                if (gamepad2.dpad_down && !dpadDownLS) {
                    liftTargetIndex = 0;
                    dpadDownLS = true;
                } else if (!gamepad2.dpad_down) {
                    dpadDownLS = false;
                }

                if (liftTargetIndex == 2) {
                    liftPowerControl(robot.liftUp);
                } else if (liftTargetIndex == 1) {
                    liftPowerControl(robot.liftMid);
                } else {
                    liftPowerControl(robot.liftDown);
                }

            } else {
                //ALTERNATE RUN MODE
                telemetry.addData("RUN MODE: ", "ALTERNATE");
                //LIFT
                if (gamepad2.right_stick_y > 0.05 && robot.lift.getCurrentPosition() >= robot.liftDown && robot.lift.getCurrentPosition() <= robot.liftUp) {
                    robot.lift.setPower(-gamepad2.right_stick_y);
                } else {
                    robot.lift.setPower(0);
                }

                //COLLECTION ARM
                if (gamepad2.left_stick_y > 0.05 && robot.arm.getCurrentPosition() >= robot.collectDown && robot.arm.getCurrentPosition() <= robot.collectUp) {
                    robot.arm.setPower(-gamepad2.left_stick_y);
                } else {
                    robot.arm.setPower(0);
                }

            }

            //COLLECTION CR SERVO
            if (gamepad2.a) {
                robot.collect.setPower(0.8);
            } else if (gamepad2.b) {
                robot.collect.setPower(-0.8);
            } else {
                robot.collect.setPower(0);
            }

            //CAP ARM
            if (gamepad2.right_bumper && !rightBumperLS) {
                //true is up, false is down
                capArmTarget = !capArmTarget;
                rightBumperLS = true;
            } else if (!gamepad2.right_bumper) {
                rightBumperLS = false;
            }

            //CAP CLAW
            if (gamepad2.right_trigger > 0.1 && !rightTriggerLS) {
                clawState = !clawState;
                rightTriggerLS = true;
            } else if (gamepad2.right_trigger <= 0.1) {
                rightTriggerLS = false;
            }

            if (clawState) {
                robot.cap.setPosition(robot.clawOpen);
            } else {
                robot.cap.setPosition(robot.clawClosed);
            }

            //deposit
            if (gamepad2.y) {
                robot.deposit.setPosition(robot.depoDown);
            } else {
                robot.deposit.setPosition(robot.depoUp);
            }

            //CAROSEL
            if (gamepad1.dpad_right) {
                robot.carousel.setPower(robot.carouselSpeed);
            } else if (gamepad1.dpad_left) {
                robot.carousel.setPower(-robot.carouselSpeed);
            } else {
                robot.carousel.setPower(0);
            }

            //DRIVING
            if (Math.abs(gamepad1.left_stick_y) > 0.05 && gamepad1.right_trigger > 0.1) {
                if (gamepad1.left_stick_y >= 0) {
                    forward = -0.2*gamepad1.left_stick_y*gamepad1.left_stick_y;
                } else {
                    forward = 0.2*gamepad1.left_stick_y*gamepad1.left_stick_y;
                }
            } else if (Math.abs(gamepad1.left_stick_y) > 0.05) {
                if (gamepad1.left_stick_y >= 0) {
                    forward = -gamepad1.left_stick_y*gamepad1.left_stick_y;
                } else {
                    forward = gamepad1.left_stick_y*gamepad1.left_stick_y;
                }
            } else {
                forward = 0;
            }

            if (Math.abs(gamepad1.right_stick_x) > 0.05) {
                if (gamepad1.right_trigger > 0.1) {
                    turning = 02*gamepad1.right_stick_x;
                } else {
                    turning = gamepad1.right_stick_x;
                }
            } else {turning = 0;}

            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                forward = -forward;
            }

            lSpeed = Range.clip(forward + turning, -1, 1);
            rSpeed = Range.clip(forward - turning, -1, 1);

            setMotorSpeed(lSpeed, rSpeed);

            capArmCorrection(capArmTarget);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("r: ", rSpeed);
            telemetry.addData("l: ", lSpeed);

            telemetry.update();
        }
    }
    public void capArmCorrection(boolean target) {
        if (target) {
            robot.capArm.setPosition(Range.clip(robot.capUp + (robot.deposit.getPosition() - 0.5), 0, 1));
        } else {
            robot.capArm.setPosition(Range.clip(robot.capDown + (robot.deposit.getPosition() - 0.5), 0, 1));
        }
    }

    public void liftPowerControl(double target) {
        double distance = target - robot.lift.getCurrentPosition();
        double holdingPower = robot.lift.getCurrentPosition()/(robot.liftUp*6);
        double power = (distance/(robot.liftUp*5)) + holdingPower;

        robot.lift.setPower(power);
    }

    public void setMotorSpeed(double l, double r) {
        robot.bl.setPower(l);
        robot.br.setPower(r);
        robot.fl.setPower(l);
        robot.fr.setPower(r);
    }
}

/*
KEY ASSIGNMENT LIST

GAMEPAD 1:
left stick:             translation
right stick:            rotation
left and right bumper:  reverse
right trigger:          slow mode
dpad left:              carousel left
dpad right:             carousel right

GAMEPAD 2:              NORMAL:             ALTERNATE:
a:                                  collect
b:                                  spit out
x:
y:                                  deposit
left stick:             arm to pos          collection arm
right stick:                                deposit lift
left bumper:                     toggles alternate
right bumper:                       cap arm
left trigger:
right trigger:                      cap claw
dpad up:                 lift to top
dpad down:               lift to bottom
dpad left:               lift to mid
dpad right:
 */