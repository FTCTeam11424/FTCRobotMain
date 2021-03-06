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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.sun.tools.javac.util.Constants.format;

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

@Autonomous(name="VuforiaTest", group="Linear Opmode")
@Disabled

public class VuforiaTest extends LinearOpMode {
    HardwareDrive robot = new HardwareDrive();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private int iterationNumber = 0;
    VuforiaLocalizer vuforia;

    //--------------------------------------------------------------------------------------------
    //Color Sensor Pre-code
    com.qualcomm.robotcore.hardware.ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    //-------------------------------------------------------------------------------------------
    //---------------------------------------------------------------------------------------------


    //------------------------------------------------------------------------------------------

    public String getVuMark() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AfdO5vj/////AAAAGSpM5tOflkvMvW4RzPkR14sF7ZtBXS06d04V0BL1s3kqEDkbvcN9uoHhoUg+hPC5pKqRAuhHfpPvv6sNrQgXO6gJaL3kzjIOlcOhx35mONJDaQ4lu3cYAxeNISUTaUkmlTajAcqhGeCLj+m+0lNjg2lF3UmfzocsFnwl8Oi6117s9MDLo3/HFTmYw/QLVnSsvdUW6GRg7jnDG1sJJmTXtOkgmbHAGrvqUSevnxjnEw9w2ME69SsbZof7/J3Xyl38xE1ekM8qn3/nC4CsQF5xJFJkbnI4h9aATJx5szNP1Zu1CSON4+WSzynZrd7H4zcVA3rQZvqEuMsQ5OlKsOlsIWdLctOLXSHTcXh7+1iXU+DS";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();

        //WARNING: This WILL block until it finds a valid instance of the pictogram.
        //Run it concurrently or implement a call to turn it while it looks.
        while (true) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                return vuMark.name();
            }
        }
    }

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
        robot.servo1.setPosition(0.075);
        robot.servo2.setPosition(0.925);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        robot.motor1 = hardwareMap.dcMotor.get("motor1");
        robot.motor2 = hardwareMap.dcMotor.get("motor2");
        robot.motor3 = hardwareMap.dcMotor.get("motor3");
        robot.motor4 = hardwareMap.dcMotor.get("motor4");
        robot.clawMotor = hardwareMap.dcMotor.get("clawMotor");
        robot.servo1 = hardwareMap.servo.get("servo1");
        robot.servo2 = hardwareMap.servo.get("servo2");
        robot.jewelServo = hardwareMap.servo.get("jewelServo");

        runtime.reset();


        /*Define motors*/
        /*robot.motor1 = hardwareMap.dcMotor.get("motor1");
        robot.motor2 = hardwareMap.dcMotor.get("motor2");
        robot.motor3 = hardwareMap.dcMotor.get("motor3");
        robot.motor4 = hardwareMap.dcMotor.get("motor4");
        robot.clawMotor = hardwareMap.dcMotor.get("clawMotor");
        robot.servo1 = hardwareMap.servo.get("servo1");
        robot.servo2 = hardwareMap.servo.get("servo2");
        robot.jewelServo = hardwareMap.servo.get("jewelServo");*/
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
        //------------------------------------------------------------------------------------------
        //Vuforia Code

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AfdO5vj/////AAAAGSpM5tOflkvMvW4RzPkR14sF7ZtBXS06d04V0BL1s3kqEDkbvcN9uoHhoUg+hPC5pKqRAuhHfpPvv6sNrQgXO6gJaL3kzjIOlcOhx35mONJDaQ4lu3cYAxeNISUTaUkmlTajAcqhGeCLj+m+0lNjg2lF3UmfzocsFnwl8Oi6117s9MDLo3/HFTmYw/QLVnSsvdUW6GRg7jnDG1sJJmTXtOkgmbHAGrvqUSevnxjnEw9w2ME69SsbZof7/J3Xyl38xE1ekM8qn3/nC4CsQF5xJFJkbnI4h9aATJx5szNP1Zu1CSON4+WSzynZrd7H4zcVA3rQZvqEuMsQ5OlKsOlsIWdLctOLXSHTcXh7+1iXU+DS";


        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();



        telemetry.addData(">", "Press Play to start");
        telemetry.update();
       //-------------------------------------------------------------------------------------------
       //Starting OpMode
        waitForStart();

        relicTrackables.activate();


 /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */

        //---------------------------------------------0-------------------------------------------

        while (opModeIsActive() && runtime.milliseconds() < 30000) {


                /**
                 * See if any of the instances of {@link relicTemplate} are currently visible.
                 * {@link RelicRecoveryVuMark} is an enum which can have the following values:
                 * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
                 * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
                 */
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                    telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                    if (pose != null) {
                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        // Extract the X, Y, and Z components of the offset of the target relative to the robot
                        double tX = trans.get(0);
                        double tY = trans.get(1);
                        double tZ = trans.get(2);

                        // Extract the rotational components of the target relative to the robot
                        double rX = rot.firstAngle;
                        double rY = rot.secondAngle;
                        double rZ = rot.thirdAngle;
                    }
                } else {
                    telemetry.addData("VuMark", "not visible");
                }
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    telemetry.update();
                    sleep(10000);
                    robot.motor1.setPower(0);
                    robot.motor2.setPower(0);
                    robot.motor3.setPower(0);
                    robot.motor4.setPower(0);
                    sleep(1000);
                    closeClaw();
                    sleep(500);
                    robot.clawMotor.setPower(1);
                    sleep(1500);
                    robot.clawMotor.setPower(0);
                    sleep(2000);
                    robot.jewelServo.setPosition(1);
                    sleep(2000);

                    telemetry.addData("JewelServoPosition", robot.jewelServo.getPosition());
                    if (sensorColor.blue() > sensorColor.red() && sensorColor.blue() > sensorColor.green()) {
                        sleep(1000);
                        turnClockwise(0.20, 1000);
                        sleep(1000);
                        robot.jewelServo.setPosition(0);
                        sleep(1000);
                        turnCounterClockwise(0.20, 1000);
                        sleep(1000);
                    } else {
                        sleep(1000);
                        turnCounterClockwise(0.20, 1000);
                        sleep(1000);
                        robot.jewelServo.setPosition(0);
                        sleep(1000);
                        turnClockwise(0.20, 1000);
                        sleep(1000);
                    }
                    moveForward(0.35, 750);
                    sleep(100);
                    turnCounterClockwise(0.35, 500);
                    sleep(100);
                    moveForward(0.35, 1000);
                    sleep(100);
                    angleOpenClaw();
                }

                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    telemetry.update();
                    sleep(10000);
                    robot.motor1.setPower(0);
                    robot.motor2.setPower(0);
                    robot.motor3.setPower(0);
                    robot.motor4.setPower(0);
                    sleep(1000);
                    closeClaw();
                    sleep(500);
                    robot.clawMotor.setPower(1);
                    sleep(1500);
                    robot.clawMotor.setPower(0);
                    sleep(2000);
                    robot.jewelServo.setPosition(1);
                    sleep(2000);

                    telemetry.addData("JewelServoPosition", robot.jewelServo.getPosition());
                    if (sensorColor.blue() > sensorColor.red() && sensorColor.blue() > sensorColor.green()) {
                        sleep(1000);
                        turnClockwise(0.20, 1000);
                        sleep(1000);
                        robot.jewelServo.setPosition(0);
                        sleep(1000);
                        turnCounterClockwise(0.20, 1000);
                        sleep(1000);
                    } else {
                        sleep(1000);
                        turnCounterClockwise(0.20, 1000);
                        sleep(1000);
                        robot.jewelServo.setPosition(0);
                        sleep(1000);
                        turnClockwise(0.20, 1000);
                        sleep(1000);
                    }
                    moveForward(0.35, 250);
                    sleep(100);
                    turnCounterClockwise(0.35, 500);
                    sleep(100);
                    moveForward(0.35, 1000);
                    sleep(100);
                    angleOpenClaw();
                }

                if (vuMark == RelicRecoveryVuMark.CENTER) {
                    telemetry.update();
                    sleep(10000);
                    robot.motor1.setPower(0);
                    robot.motor2.setPower(0);
                    robot.motor3.setPower(0);
                    robot.motor4.setPower(0);
                    sleep(1000);
                    closeClaw();
                    sleep(500);
                    robot.clawMotor.setPower(1);
                    sleep(1500);
                    robot.clawMotor.setPower(0);
                    sleep(2000);
                    robot.jewelServo.setPosition(1);
                    sleep(2000);

                    telemetry.addData("JewelServoPosition", robot.jewelServo.getPosition());
                    if (sensorColor.blue() > sensorColor.red() && sensorColor.blue() > sensorColor.green()) {
                        sleep(1000);
                        turnClockwise(0.20, 1000);
                        sleep(1000);
                        robot.jewelServo.setPosition(0);
                        sleep(1000);
                        turnCounterClockwise(0.20, 1000);
                        sleep(1000);
                    } else {
                        sleep(1000);
                        turnCounterClockwise(0.20, 1000);
                        sleep(1000);
                        robot.jewelServo.setPosition(0);
                        sleep(1000);
                        turnClockwise(0.20, 1000);
                        sleep(1000);
                    }
                    moveForward(0.35, 500);
                    sleep(100);
                    turnCounterClockwise(0.35, 500);
                    sleep(100);
                    moveForward(0.35, 1000);
                    sleep(100);
                    angleOpenClaw();

                }
                if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
                    if (sensorColor.blue() > sensorColor.red() && sensorColor.blue() > sensorColor.green()) {
                        sleep(1000);
                        turnClockwise(0.25, 1000);
                        sleep(1000);
                        robot.jewelServo.setPosition(0);
                        sleep(1000);
                        turnCounterClockwise(0.25, 1000);
                        sleep(1000);
                    } else {
                        sleep(1000);
                        turnCounterClockwise(0.25, 1000);
                        sleep(1000);
                        robot.jewelServo.setPosition(0);
                        sleep(1000);
                        turnClockwise(0.25, 1000);
                        sleep(1000);
                    }
                }

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
            //telemetry.addData("Distance (cm)",
             //       String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
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
            //Move Left Function


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
