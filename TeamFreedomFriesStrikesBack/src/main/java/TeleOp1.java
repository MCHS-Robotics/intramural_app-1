/**
 * Created by student on 4/18/17.
 */
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@TeleOp
public class TeleOp1 extends LinearOpMode {
    DcMotor fLeft, fRight, bLeft, bRight, sweep, lift;
    Servo gateOpen, tilt;
    //double mPower;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        fLeft = hardwareMap.dcMotor.get("fL");
        fRight = hardwareMap.dcMotor.get("fR");
        bLeft = hardwareMap.dcMotor.get("bL");
        bRight = hardwareMap.dcMotor.get("bR");
        sweep = hardwareMap.dcMotor.get("sweep");
        lift = hardwareMap.dcMotor.get("lift");
        gateOpen = hardwareMap.servo.get("gate");
        tilt = hardwareMap.servo.get("tilt");

        fRight.setDirection(DcMotor.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");

        waitForStart();

        while(opModeIsActive()){
            forward(gamepad1.left_stick_y);
            turn(gamepad1.right_stick_x);

            /*if (gamepad1.a)
                sweep.setPower(0.20);
            */
        }
    }

    public void forward(double power) {
        fLeft.setPower(power);
        fRight.setPower(power);
        bLeft.setPower(power);
        bRight.setPower(power);
    }

    public void turn(double power) {
        fLeft.setPower(-power);
        fRight.setPower(power);
        bLeft.setPower(-power);
        bRight.setPower(power);
    }
}
