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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

import eaglerobotics.library.drivetrain.Holonomic;
import eaglerobotics.library.functions.MathOperations;

import java.lang.InterruptedException;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Autonomous", group = "Meet 1")
//@Disabled
public class Meet1AutoRed extends LinearOpMode {

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

  ColorSensor colorSensorLeft;
  ColorSensor colorSensorRight;

  // Team Color and Starting Position
  Color teamColor = Color.NULL;
  StartingPosition startingPosition = StartingPosition.NULL;

  @Override
  public void runOpMode() {
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

    colorSensorLeft = hardwareMap.colorSensor.get("colorSensorLeft");
    colorSensorRight = hardwareMap.colorSensor.get("colorSensorRight");

    // Set all servo positions here...

    while(!isStarted()) {
      // Set Red or Blue
      if (gamepad1.x)
        teamColor = Color.BLUE;
      else if (gamepad1.b)
        teamColor = Color.RED;

      telemetry.addData("Alliance Color: ", teamColor.toString());
      String setIt = "SET THE DANG ALLIANCE COLOR";
      if (teamColor == Color.NULL)
        telemetry.addData("Boiiii: ", setIt);

      // Set Postion 1 or 2
      if(gamepad2.right_bumper)
        startingPosition = StartingPosition.RIGHT;
      else if(gamepad2.left_bumper)
        startingPosition = StartingPosition.LEFT;
      String setItPlease = "SET THE DANG POSITION";
      if(startingPosition == startingPosition.NULL)
        telemetry.addData("Bruhhhhh: ", setItPlease);

      telemetry.addData("Starting Position: ", startingPosition.toString());
      telemetry.update();
    }

    // Lower the Jewel Manipulator
    jewelManipulator.setPosition(1);

    // Go to each case for each color
    // Check the Color
    // Drive Forward or Backwards based on color
    String temp = "Stupid";
    switch (teamColor){
      case BLUE:
        if(colorSensorLeft.red() > colorSensorLeft.blue() && colorSensorRight.blue() > colorSensorRight.red()){
          // Drive Left
          holonomic.run(0, 1, 0);
        } else if(colorSensorLeft.red() < colorSensorLeft.blue() && colorSensorRight.blue() < colorSensorRight.red()){
          // Drive Right
          holonomic.run(0,-1,0);
        }
        sleep(1000);
        break;
      case RED:
        if(colorSensorLeft.red() < colorSensorLeft.blue() && colorSensorRight.blue() < colorSensorRight.red()){
          // Drive Left
          holonomic.run(0, 1, 0);
        } else if(colorSensorLeft.red() > colorSensorLeft.blue() && colorSensorRight.blue() > colorSensorRight.red()){
          // Drive Right
          holonomic.run(0,-1,0);
        }
        sleep(1000);
        break;
      case NULL:
        telemetry.addData("YOU ARE A ", temp);
        break;
      default:
        telemetry.addData("YOU ARE A ", temp);
        break;
    }

    // Raise Arm
    jewelManipulator.setPosition(0);

    // Drive Forward To Get Off Platform

  }
}

enum Color{
  BLUE,RED,NULL
}

enum StartingPosition{
  LEFT, RIGHT, NULL
}
