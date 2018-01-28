package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.concurrent.Callable;

/**
 * Created by dcrenshaw on 1/27/18.
 *
 * The Metal Drive Kit is a framework for connecting directly to the robot hardware for driving.
 * Metal is intended to be a more feature-rich replacement for existing drive proxy functions.
 */

public class MetalDriveKit extends Object {
    //Define servo and motor variables and set them to null
    public DcMotor motor1 = null;
    public DcMotor motor2 = null;
    public DcMotor motor3 = null;
    public DcMotor motor4 = null;
    public DcMotor clawMotor = null;
    public Servo servo1 = null;
    public Servo servo2 = null;
    public Servo jewelServo = null;
    //Reference to mapped servo/motor controller
    private HardwareMap hwMap = null;

    private double prevailingSpeed = 0.35;

    private HashMap<String, Callable> intents = new HashMap<String, Callable>();

    private ElapsedTime period = new ElapsedTime();

    public void init (HardwareMap ahwMap) {
        hwMap = ahwMap;

        //Define and connect variables to their matching motors on the robot
        motor1 = hwMap.dcMotor.get("motor1");
        motor2 = hwMap.dcMotor.get("motor2");
        motor3 = hwMap.dcMotor.get("motor3");
        motor4 = hwMap.dcMotor.get("motor4");
        clawMotor = hwMap.dcMotor.get("clawMotor");
        servo1 = hwMap.servo.get("servo1");
        servo2 = hwMap.servo.get("servo2");
        jewelServo = hwMap.servo.get("jewelServo");

        //Sets the motors to appropriate direction, FORWARD=Clockwise, REVERSE=CounterClockwise
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);
        //Rest all motors
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        //Set all motors to run without encoders
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Define and initialize ALL installed servos. Here is an example snippet:
     /* leftClaw = hwMap.servo.get("left_hand");
        rightClaw = hwMap.servo.get("right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);
    */
    }
    public void setMotors(double npower) {
        motor1.setPower(npower);
        motor2.setPower(npower);
        motor3.setPower(npower);
        motor4.setPower(npower);
    }
    public void waitForTick(long periodMs) {
        long remaining = periodMs - (long) period.milliseconds();
        //Sleep for the remaining portion of the regular cycle period
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        //Reset the cycle clock for the next pass
        period.reset();
    }
    public void moveForward() {
        setMotors(prevailingSpeed);
    }
    public void rest() {
        setMotors(0);
    }
    public void moveBackward() {
        setMotors(-prevailingSpeed);
    }
    public void moveLeft() {
        motor1.setPower(-prevailingSpeed);
        motor2.setPower(prevailingSpeed);
        motor3.setPower(-prevailingSpeed);
        motor4.setPower(prevailingSpeed);
    }
    public void moveRight() {
        motor1.setPower(prevailingSpeed);
        motor2.setPower(-prevailingSpeed);
        motor3.setPower(prevailingSpeed);
        motor4.setPower(-prevailingSpeed);
    }
    public void turnLeft() { //counterclockwise
        motor1.setPower(-prevailingSpeed);
        motor2.setPower(prevailingSpeed);
        motor3.setPower(prevailingSpeed);
        motor4.setPower(-prevailingSpeed);
    }
    public void turnRight() { //clockwise
        motor1.setPower(prevailingSpeed);
        motor2.setPower(-prevailingSpeed);
        motor3.setPower(-prevailingSpeed);
        motor4.setPower(prevailingSpeed);
    }
    public void reduceSpeed() {
        if (prevailingSpeed == 0.5) {
            prevailingSpeed = 0.35;
        } else {
            prevailingSpeed = 0.25;
        }
    }
    public void increaseSpeed() {
        if (prevailingSpeed == 0.25) {
            prevailingSpeed = 0.35;
        } else {
            prevailingSpeed = 0.5;
        }
    }
    public void sendMessage(char msg) {
        /* Needs to be fixed to...work
        <Callable> intent = new <Callable>();
        */
    }
    public void toTop() {
        //Requires testing of the robot to implement.
        //Needs to know how far the claw moves in 100 iterations
    }
}
