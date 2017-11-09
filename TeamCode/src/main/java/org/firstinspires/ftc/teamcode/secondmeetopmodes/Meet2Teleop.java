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

package org.firstinspires.ftc.teamcode.secondmeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import eaglerobotics.library.drivetrain.Holonomic;
import eaglerobotics.library.encoder.EncoderMotor;
import eaglerobotics.library.functions.MathOperations;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Teleop Meet 2", group = "Meet 2")
//@Disabled
public class Meet2Teleop extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();

  // Holonomic System
  DcMotor leftFrontMotor;
  DcMotor leftRearMotor;
  DcMotor rightFrontMotor;
  DcMotor rightRearMotor;

  Holonomic holonomic;

  // Threaded rod lift
  DcMotor leftThreadedRodLift;
  DcMotor rightThreadedRodLift;
  EncoderMotor leftLift;
  EncoderMotor rightLift;
  int position;

  // Intake
  Servo leftIntake;
  Servo rightIntake;

  // Jewel Manipulator
  Servo jewelManipulator;

  //ColorSensor colorSensorLeft;
  //ColorSensor colorSensorRight;

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");

    // Get motors from map
    leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
    leftRearMotor = hardwareMap.dcMotor.get("leftRearMotor");
    rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
    rightRearMotor = hardwareMap.dcMotor.get("rightRearMotor");

    holonomic = new Holonomic(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

    leftThreadedRodLift = hardwareMap.dcMotor.get("leftThreadedRodLift");
    rightThreadedRodLift = hardwareMap.dcMotor.get("rightThreadedRodLift");

    leftThreadedRodLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightThreadedRodLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftThreadedRodLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightThreadedRodLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    leftLift = new EncoderMotor(leftThreadedRodLift, 1);
    rightLift = new EncoderMotor(rightThreadedRodLift, 1);

    leftIntake = hardwareMap.servo.get("leftIntake");
    rightIntake = hardwareMap.servo.get("rightIntake");


    jewelManipulator = hardwareMap.servo.get("jewelManipulator");

    //colorSensorLeft = hardwareMap.colorSensor.get("colorSensorLeft");
    //colorSensorRight = hardwareMap.colorSensor.get("colorSensorRight");
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {

  // Set all servo positions in here...
    jewelManipulator.setPosition(.3);

    leftIntake.setPosition(1);
    rightIntake.setPosition(0);

  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    runtime.reset();
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop(){
    telemetry.addData("Status", "Run Time: " + runtime.toString());
    // Run using cubic and Y reversed
    holonomic.run(MathOperations.pow(-gamepad1.left_stick_y, 3), MathOperations.pow(gamepad1.left_stick_x, 3),
            MathOperations.pow(gamepad1.right_stick_x, 3));

    // Run the Threaded Rod Lift

    //Run to Position Threaded Rod Lift
    int upperTarget = 17000;
    int midTarget = upperTarget/2;
    int lowerTarget = 7;
    if (gamepad2.dpad_up){
      position = upperTarget;
      leftLift.runToPosition(position);
      rightLift.runToPosition(position);
    } else if (gamepad2.dpad_right || gamepad2.dpad_left){
      position = midTarget;
    } else if (gamepad2.dpad_down){
      position = lowerTarget;
      leftLift.runToPosition(position);
      rightLift.runToPosition(position);
    } else {
      leftThreadedRodLift.setPower(gamepad2.left_stick_y);
      rightThreadedRodLift.setPower(gamepad2.left_stick_y);
    }


    // Run Threaded Rod Lift
    //leftThreadedRodLift.setPower(-gamepad2.left_stick_y);
    //rightThreadedRodLift.setPower(-gamepad2.left_stick_y);


    telemetry.addData("Left E Val: ", leftThreadedRodLift.getCurrentPosition());
    telemetry.addData("Right E Val: ", rightThreadedRodLift.getCurrentPosition());

    // Run the Intake
    if(gamepad2.right_trigger > 0){
      leftIntake.setPosition(.45);
      rightIntake.setPosition(.55);
    } else if(gamepad2.left_trigger > 0){
      leftIntake.setPosition(0);
      rightIntake.setPosition(1);
    } else if(gamepad2.right_bumper){
      leftIntake.setPosition(.5);
      rightIntake.setPosition(.5);
    } else if(gamepad2.left_bumper){
      leftIntake.setPosition(.3);
      rightIntake.setPosition(.7);
    }

    // Run the jewel manipulator
    if(gamepad2.y){
      jewelManipulator.setPosition(.7);
    } else if(gamepad2.a){
      jewelManipulator.setPosition(.3);
    }
  }
}
