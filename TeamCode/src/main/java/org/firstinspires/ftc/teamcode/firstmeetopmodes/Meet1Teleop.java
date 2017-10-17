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

package org.firstinspires.ftc.teamcode.firstmeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import eaglerobotics.library.drivetrain.Holonomic;
import eaglerobotics.library.functions.MathOperations;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Concept: Holonomic", group = "Concept")
//@Disabled
public class Meet1Teleop extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();

  // Holonomic System
  DcMotor leftFrontMotor;
  DcMotor leftRearMotor;
  DcMotor rightFrontMotor;
  DcMotor rightRearMotor;

  Holonomic holonomic;

  // Lift System
  DcMotor threadedRodLift;

  // Intake/Scorer System
  Servo leftIntakeSpinner;
  Servo rightIntakeSpinner;

  Servo tensionWheelFront;
  Servo tensionWheelRear;

  Servo conveyorLeft;
  Servo conveyorRight;

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

    threadedRodLift = hardwareMap.dcMotor.get("threadedRodLift");

    leftIntakeSpinner = hardwareMap.servo.get("leftIntakeSpinner");
    rightIntakeSpinner = hardwareMap.servo.get("rightIntakeSpinner");

    tensionWheelFront = hardwareMap.servo.get("tensionWheelFront");
    tensionWheelRear = hardwareMap.servo.get("tensionWheelRear");

    conveyorLeft = hardwareMap.servo.get("conveyorLeft");
    conveyorRight = hardwareMap.servo.get("conveyorRight");

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
    jewelManipulator.setPosition(0);
    conveyorRight.setPosition(.5);
    conveyorLeft.setPosition(.5);
    tensionWheelRear.setPosition(0);
    tensionWheelFront.setPosition(0);
    leftIntakeSpinner.setPosition(.5);
    rightIntakeSpinner.setPosition(.5);
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
    double tempPowerLift = (double)gamepad2.right_stick_y;
    tempPowerLift = Range.clip(tempPowerLift, -1, 1);
    threadedRodLift.setPower(tempPowerLift);

    // Run the Intake Spinner
    if (gamepad2.right_trigger > 0){
      leftIntakeSpinner.setPosition(1);
      rightIntakeSpinner.setPosition(0);
    } else if(gamepad2.right_bumper){
      leftIntakeSpinner.setPosition(0);
      rightIntakeSpinner.setPosition(1);
    } else {
      leftIntakeSpinner.setPosition(.5);
      rightIntakeSpinner.setPosition(.5);
    }

    // Run the Tension Wheels
    if(gamepad2.right_bumper){
      tensionWheelFront.setPosition(1);
      tensionWheelRear.setPosition(0);
    } else if(gamepad2.dpad_left){
      tensionWheelFront.setPosition(1);
      tensionWheelRear.setPosition(0);
    }

    // Run the Conveyor
    if (gamepad2.dpad_up){
      conveyorLeft.setPosition(1);
      conveyorRight.setPosition(0);
    } else if (gamepad2.dpad_down){
      conveyorLeft.setPosition(0);
      conveyorRight.setPosition(1);
    } else {
      conveyorLeft.setPosition(.5);
      conveyorRight.setPosition(.5);
    }

    // Run the jewel manipulator
    if(gamepad2.y){
      jewelManipulator.setPosition(1);
    } else if(gamepad2.a){
      jewelManipulator.setPosition(0);
    }


  }
}
