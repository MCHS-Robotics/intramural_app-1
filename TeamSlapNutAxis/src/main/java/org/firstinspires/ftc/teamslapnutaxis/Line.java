public class Line{

}
/*
Copyright (c) 2016 Robert Atkinson
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

        //package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.AnalogInput;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
        import com.qualcomm.robotcore.hardware.DigitalChannelController;
        import java.util.*;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.GyroSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;

*/
/**
 * OpMode for testing correct connection and direction of drive motors.
 * Spins each motor starting with the FL and going clockwise.
 *//*

*/
/*
@Autonomous(name = "Launch 1 Beacon", group = "Commands")
//@Disabled
public class Line LinearOpMode {

    //private ElapsedTime runtime = new ElapsedTime();


    DcMotor FL, FR, BL, BR, launcher, scoop;
    public double threshold = 3.05;
    public AnalogInput lightSensor;
    static ColorSensor sensorRGB;
    static DeviceInterfaceModule cdim;
    int numInAvg = 6;
    double[] valueArray;
    int current = 0;
    static final int LED_CHANNEL = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        valueArray = new double[numInAvg];
        for (int i = 0; i < numInAvg; i++) {
            valueArray[i] = 4;
        }
        lightSensor = hardwareMap.analogInput.get("lS");
        FL = hardwareMap.dcMotor.get("fl");
        FR = hardwareMap.dcMotor.get("fr");
        BL = hardwareMap.dcMotor.get("bl");
        BR = hardwareMap.dcMotor.get("br");
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        launcher = hardwareMap.dcMotor.get("launch");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scoop = hardwareMap.dcMotor.get("scoop");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        sensorRGB = hardwareMap.colorSensor.get("sensor_color");
        cdim.setDigitalChannelState(LED_CHANNEL, false);
        //lightSensorWorker.calibrate();

        idle();

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();
        */
/*sleep(2000);
        moveLauncher(2, .8);
        moveScoop(3,.37);
        moveLauncher(2,.8);
        sleep(1000);*//*

        */
/*
        forward(15);
        turnLeft(100);
        backwards(40);
        turnLeft(45);
        moveTillWhite(-.15);
        turnRight(45);
        *//*

        moveLauncher(2, .8);
        forward(7);
        counterClockWise(80);
        diagonalTilWhite(.15);
        sleep(100);
        backwards(10);
        forward(2);
        hitBlue();
        telemetry.addData("Status", "Complete");
        telemetry.update();
    }

    public void doSomethingInteresting() {
        //FL.setMode(DcMotor)
    }

    public void moveTillWhite(double power) {
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //////////
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
        //////////
        while (!isWhite()) {
            telemetry.addData("Status", lightSensor.getVoltage());
            telemetry.update();
        }
        //////////
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        //////////
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);
        if (!isWhite()) {
            moveTillWhite(-(power / Math.abs(power)) * .05);
        }
    }

    public void addToArray(double val) {
        valueArray[current] = val;
        current++;
        if (current == numInAvg) {
            current = 0;
        }
    }

    public double average() {
        double total = 0;
        for (int i = 0; i < numInAvg; i++) {
            total += valueArray[i];
        }
        return total / (numInAvg);
    }

    public double[] mode()
    {
        //List<Double> hayes = new ArrayList<>();
        double most = 0;
        double sample = 0;
        for (int i = 0; i < numInAvg; i++) {
            int highest;
            int lowest;
            int[] positive;
            int[] negative;
            for (int i = 0; i < numInAvg; i++) {
                if (i == 0) {
                    highest = valueArray[i];
                    lowest = highest;
                } else {
                    if (valueArray[i] > highest) {
                        highest = valueArray[i];
                    } else if (valueArray[i] < lowest) {
                        lowest = valueArray[i];
                    }
                }
            }
            int lav = lowest * -1;
            positive = new int[highest];
            negative = new int[lav];
            for (int x = 0; x < numInAvg; x++) {
                int value = valueArray[x];
                if (value >= 0) {
                    positive[value]++;
                } else {
                    negative[value * -1]++;
                }
            }
            //double highest;
            //double lowest;
            double mode;
            //double nmode;
            ArrayList<Double> pos = new ArrayList<Double>();
            ArrayList<Double> neg = new ArrayList<Double>();
            for (int i = 0; i < positive.length; i++) {
                if (i == 0) {
                    mode = positive[i];
                } else {
                    if (positive[i] > mode) {
                        mode = positive[i];
                    }
                }
            }
            for (int i = 0; i < positive.length; i++) {
                if (positive[i] == mode) {
                    pos.add(i);
                }
            }
            for (int i = 0; i < negative.length; i++) {
                if (negative[i] > mode) {
                    neg.add(i * -1);
                } else if (negative[i] == mode) {
                    pos.add(i * -1);
                }
            }
            if (neg.size() > 0) {
                return neg;
            } else {
                return pos;
            }
        }
    }

    public boolean isWhite() {
        addToArray(lightSensor.getVoltage());
        return average() < threshold;
    }

    public void hitBlue() {
        if (isRed()) {

            try {

                Thread.sleep(100);
            } catch (Exception e) {
            }
        } else {

            try {
                forward(3);
                sleep(3000);
                backwards(9);
                forward(5);
                Thread.sleep(100);
            } catch (Exception e) {
            }

        }
        //forward(10);
    }

    public boolean isRed() {
        //soundPlayer.play(hardwareMap.appContext,0);
        cdim.setDigitalChannelState(LED_CHANNEL, true);
        try {
            Thread.sleep(100);
        } catch (Exception e) {

        }
        boolean isR = sensorRGB.red() > sensorRGB.blue();
        cdim.setDigitalChannelState(LED_CHANNEL, false);
        return isR;
    }

    public void forward(double inches) {
        double target = 560 * inches / (4 * Math.PI * Math.sqrt(2));
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        powerAll(.20);
        while (FR.getCurrentPosition() < target) {
        }
        powerAll(0);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void left(double inches) {
        double target = 560 * inches / (4 * Math.PI * Math.sqrt(2));
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setPower(-.25);
        FR.setPower(.25);
        BL.setPower(.25);
        BR.setPower(-.25);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void right(double inches) {
        double target = 560 * inches / (4 * Math.PI * Math.sqrt(2));
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setPower(.25);
        FR.setPower(-.25);
        BL.setPower(-.25);
        BR.setPower(.25);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void powerAll(double power) {
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
    }

    public void counterClockWise(int degrees) {
        int target = (int) (700 / 90.0 * degrees);
        FL.setTargetPosition(-(target));
        FR.setTargetPosition((target));
        BL.setTargetPosition(-(target));
        BR.setTargetPosition((target));

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(-.25);
        FR.setPower(.25);
        BL.setPower(-.25);
        BR.setPower(.25);

        while (FL.isBusy() && FR.isBusy() && BR.isBusy() && BL.isBusy()) {
            telemetry.addData("FL", "" + FL.getCurrentPosition());
            telemetry.update();
            telemetry.addData("FR", "" + FR.getCurrentPosition());
            telemetry.update();
            telemetry.addData("BL", "" + BL.getCurrentPosition());
            telemetry.update();
            telemetry.addData("BR", "" + BR.getCurrentPosition());
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void clockWise(int degrees) {
        int target = (int) (700 / 90.0 * degrees);
        FL.setTargetPosition((target));
        FR.setTargetPosition(-(target));
        BL.setTargetPosition((target));
        BR.setTargetPosition(-(target));

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(-.25);
        FR.setPower(.25);
        BL.setPower(-.25);
        BR.setPower(.25);

        while (FL.isBusy() && FR.isBusy() && BR.isBusy() && BL.isBusy()) {
            telemetry.addData("FL", "" + FL.getCurrentPosition());
            telemetry.update();
            telemetry.addData("FR", "" + FR.getCurrentPosition());
            telemetry.update();
            telemetry.addData("BL", "" + BL.getCurrentPosition());
            telemetry.update();
            telemetry.addData("BR", "" + BR.getCurrentPosition());
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void backwards(double inches) {
        double target = 560 * inches / (4 * Math.PI * Math.sqrt(2));
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        powerAll(-.25);
        while (-FR.getCurrentPosition() < target) {

        }
        powerAll(0);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void diagonal(double inches) {
        double target = 560 * inches / (4 * Math.PI * Math.sqrt(2));
        //FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        powerAll(.25);
        while (FR.getCurrentPosition() < target) {
        }
        powerAll(0);
        //FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void moveScoop(double seconds, double power) {
        scoop.setPower(power);
        sleep((int) (seconds * 1000));
        scoop.setPower(0);

    }

    public void diagonalTilWhite(double power) {
        //FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //////////
        //FL.setPower(power);
        FR.setPower(-power);
        BL.setPower(-power);
        //BR.setPower(power);
        //////////
        while (!isWhite()) {
            telemetry.addData("Status", lightSensor.getVoltage());
            telemetry.update();
        }
        //////////
        //FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        //BR.setPower(0);
        //////////
        //FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);
        if (!isWhite()) {
            moveTillWhite(-(power / Math.abs(power)) * .05);
        }


    }

    public void hitRed() {
        if (isRed()) {
            //forward
            try {

                Thread.sleep(100);
            } catch (Exception e) {
            }
        } else {
            try {
                Thread.sleep(100);
            } catch (Exception e) {
            }
        }
    }
    public void moveLauncher(double turns, double speed) {
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcher.setTargetPosition((int) (turns * 1680));
        launcher.setPower(speed);
        while (launcher.isBusy() && opModeIsActive()) {
            telemetry.addData("Status", "" + launcher.getCurrentPosition());
            telemetry.update();
        }
        launcher.setPower(0);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
*/
