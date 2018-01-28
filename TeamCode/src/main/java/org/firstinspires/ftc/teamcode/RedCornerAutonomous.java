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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="RedCornerAutonomous", group="Linear Opmode")
//@Disabled

public class RedCornerAutonomous extends LinearOpMode {
    HardwareDrive robot = new HardwareDrive();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //--------------------------------------------------------------------------------------------
    //Color Sensor Pre-code
    com.qualcomm.robotcore.hardware.ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    //-------------------------------------------------------------------------------------------
    //---------------------------------------------------------------------------------------------

    public void moveForward(double power, long time) {
        robot.motor1.setPower(power);
        robot.motor2.setPower(-power);
        robot.motor3.setPower(power);
        robot.motor4.setPower(-power);
        sleep(time);
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
    }

    public void moveBackward(double power, long time) {
        robot.motor1.setPower(-power);
        robot.motor2.setPower(power);
        robot.motor3.setPower(-power);
        robot.motor4.setPower(power);
        sleep(time);
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
    }

    public void moveRight(double power, long time) {
        robot.motor1.setPower(power);
        robot.motor2.setPower(power);
        robot.motor3.setPower(-power);
        robot.motor4.setPower(-power);
        sleep(time);
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
    }

    public void moveLeft(double power, long time) {
        robot.motor1.setPower(-power);
        robot.motor2.setPower(-power);
        robot.motor3.setPower(power);
        robot.motor4.setPower(power);
        sleep(time);
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
    }
    public void turnClockwise(double power, long time) {
        robot.motor1.setPower(power);
        robot.motor2.setPower(power);
        robot.motor3.setPower(power);
        robot.motor4.setPower(power);
        sleep(time);
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
    }
    public void turnCounterClockwise(double power, long time) {
        robot.motor1.setPower(-power);
        robot.motor2.setPower(-power);
        robot.motor3.setPower(-power);
        robot.motor4.setPower(-power);
        sleep(time);
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
    }
    public void closeClaw() {
        robot.servo1.setPosition(0);
        robot.servo2.setPosition(1);
    }
    public void openClaw() {
        robot.servo1.setPosition(0.5);
        robot.servo2.setPosition(0.5);
    }
    public void angleOpenClaw() {
        robot.servo1.setPosition(0.10);
        robot.servo2.setPosition(0.90);
    }
    //-------------------------------------------------------------------------------------------
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        runtime.reset();
        /*Define motors*/
        robot.motor1 = hardwareMap.dcMotor.get("motor1");
        robot.motor2 = hardwareMap.dcMotor.get("motor2");
        robot.motor3 = hardwareMap.dcMotor.get("motor3");
        robot.motor4 = hardwareMap.dcMotor.get("motor4");
        robot.clawMotor = hardwareMap.dcMotor.get("clawMotor");
        robot.servo1 = hardwareMap.servo.get("servo1");
        robot.servo2 = hardwareMap.servo.get("servo2");
        robot.jewelServo = hardwareMap.servo.get("jewelServo");
         /*--------------------------------------------------------------------------------------------------------*/
        /*--------------------------------Color Sensor Autonomous For Jewels------------------------------------------*/
        /*---------------------------------------------------------------------------------------------------------*/
        /**
         * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
         * It has a light/distance (range) sensor.  It also has an RGB color sensor.
         * The light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
         * or closer will display the same value for distance/light detected.
         *
         * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
         * you can treat the sensor as two separate sensors that share the same name in your op mode.
         *
         * In this example, we represent the detected color by a hue, saturation, and value color
         * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
         * color of the screen to match the detected color.
         *
         * In this example, we  also use the distance sensor to display the distance
         * to the target object.  Note that the distance sensor saturates at around 2" (5 cm).
         *
         */


        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.


        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
        sleep(1000);
        closeClaw();
        sleep(1000);
        robot.clawMotor.setPower(1);
        sleep(1500);
        robot.clawMotor.setPower(0);
        sleep(1000);
        robot.jewelServo.setPosition(1);
        sleep(1000);
        if (sensorColor.red() > sensorColor.blue() && sensorColor.red() > sensorColor.green()) {
            sleep(1000);
            turnClockwise(0.35, 500);
            sleep(1000);
            robot.jewelServo.setPosition(0);
            sleep(1000);
            turnCounterClockwise(0.35, 500);
            sleep(1000);
        } else {
            sleep(1000);
            turnCounterClockwise(0.35, 500);
            sleep(1000);
            robot.jewelServo.setPosition(0);
            sleep(1000);
            turnClockwise(0.35, 500);
            sleep(1000);
        }
        sleep(2000);
        //add code here for crypt and safezone placers
        //------------------------------------------------------------------------------------------
        while (opModeIsActive() && runtime.milliseconds() < 20000) {
            sleep(40);
            //----------------------------------------------------------------------------------------
            //Whileopmodeisactive color sensor code
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
          //  telemetry.addData("Distance (cm)",
          //          String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
           // telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
          //  telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        //---------------------------------------------------------------------------------------------



                }

            }

        }

        // Set the panel back to the default color



        //--------------------------------------------0---------------------------------------------











/*TO-DO*/
/*List:
 * Write code for identifying crypt; ideally a barcode-style scan algo
 * Time complexity is about O(
 */