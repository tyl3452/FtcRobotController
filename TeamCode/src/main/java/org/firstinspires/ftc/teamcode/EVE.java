/**********************************************
 * @author MrLiu
 * EVE
 * A fun-loving robot who wants to help
 * Dual Auto and TeleOp Linear OpMode
 * 3 Wheel Odometry using RoadRunner Library
 * Features:
 * 4 Motor Mecanum/Omni Drive
 * 1 Servo - gobilda is 300 degrees
 * 1 arm using a DCMotor w/encoder
 * 1 Camera using TF
 * 1 Color Sensor
 * 3 Odometry
 **********************************************/

package org.firstinspires.ftc.teamcode;

// Basic imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// IMU imports
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//Camera Imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;

// Color Sensor Imports
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// RoadRunner library
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

@TeleOp(name="EVE TeleOp", group="Linear Opmode")
public class EVE extends LinearOpMode {
    /* Declare Global variables */

    // Define 2 states, drive control or automatic control
    enum Mode {
        TELEOP,
        AUTONOMOUS
    }

    Mode currentMode = Mode.TELEOP;         // Default Mode TeleOp

    //*********************************************************************************
    // (0,0) is Top left is near blue1 by backdrop
    // (6,6) bottom right is near Red2 location
    // y is positive when you go down
    // * means out of bounds
    // X is on field but no Robot
    // R is the Robot
    // P is preferred path
    // B is backdrop
    //*********************************************************************************

    // Field location - Blue 1 next to  backdrop
    private char[][] fieldBlue1 = {
            {'*', '*', '*', '*', '*', '*', '*','*'},
            {'*', 'X', 'B', 'X', 'X', 'B', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'R', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', '*', '*', '*', '*', '*', '*','*'} };

    // Field location - Blue 2 away from backdrop
    private char[][] fieldBlue2 = {
            {'*', '*', '*', '*', '*', '*', '*','*'},
            {'*', 'X', 'B', 'X', 'X', 'B', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'R', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', '*', '*', '*', '*', '*', '*','*'} };

    // Field location - Red 1 next to backdrop
    private char[][] fieldRed1 = {
            {'*', '*', '*', '*', '*', '*', '*','*'},
            {'*', 'X', 'B', 'X', 'X', 'B', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'R','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', '*', '*', '*', '*', '*', '*','*'} };

    // Field location - Red 2 away from backdrop
    private char[][] fieldRed2 = {
            {'*', '*', '*', '*', '*', '*', '*','*'},
            {'*', 'X', 'B', 'X', 'X', 'B', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'R','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', '*', '*', '*', '*', '*', '*','*'} };

    // Blue 1 preferred path, follow the P's for autonomous
    private char[][] blue1Path = {
            {'*', '*', '*', '*', '*', '*', '*','*'},
            {'*', 'X', 'B', 'X', 'X', 'B', 'X','*'},
            {'*', 'X', 'P', 'X', 'X', 'X', 'X','*'},
            {'*', 'R', 'P', 'X', 'X', 'X', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', 'X', 'X', 'X', 'X', 'X', 'X','*'},
            {'*', '*', '*', '*', '*', '*', '*','*'} };


    // Set default starting field position
    private char[][] field = fieldBlue1;
    private String alliance = "blue";
    private double[] coordinatesOdometry = new double[2];       // current odometry coordinates in inches

    //*********************************************************************************
    // 4 Motor Drive
    //*********************************************************************************
    private DcMotor leftFrontDrive   = null;    // Motors
    private DcMotor leftRearDrive    = null;
    private DcMotor rightRearDrive   = null;
    private DcMotor rightFrontDrive  = null;

    private double  leftFrontSpeed    = 0;      // Motor Power/Speed
    private double  leftRearSpeed     = 0;
    private double  rightRearSpeed    = 0;
    private double  rightFrontSpeed   = 0;

    private int     leftFrontTarget   = 0;      // Use motors with encoders
    private int     leftRearTarget    = 0;
    private int     rightRearTarget   = 0;
    private int     rightFrontTarget  = 0;


    //*********************************************************************************
    // Servo and CRServo
    //*********************************************************************************
    private Servo servo = null;
    private CRServo crServo = null;             // CR Servo, use setPower() instead of setPosition()
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    = 2;        // period of each cycle
    static final double SERVO_MAX_POS     = 1.0;      // Maximum rotational position
    static final double SERVO_MIN_POS     = 0.0;      // Minimum rotational position
    private boolean isServoOpen;                      // servo Open State, Open is at MAX
    private boolean rotateServo;                      // CR Servo state
    private double servoPosition;

    //*********************************************************************************
    // Wrist and Claw
    //*********************************************************************************
    private Servo wrist = null;
    private Servo claw = null;
    static final double WRIST_MAX_POS    = 1.0;        // Maximum rotational position
    static final double WRIST_MIN_POS    = 0.0;        // Minimum rotational position
    static final double CLAW_MAX_POS     = 1.0;         // Maximum rotational position
    static final double CLAW_MIN_POS     = 0.0;         // Minimum rotational position
    private boolean isWristUp;
    private boolean isClawOpen;
    private double wristPosition;
    private double clawPosition;


    //*********************************************************************************
    // 1 Motor Control Arm
    //*********************************************************************************
    private DcMotor arm   = null;
    static final double ARM_DEFAULT_POWER = 1.0;// Default ARM Power if needed
    static final int ARM_MAX_POS = 1500;        // Max encoder arm position
    static final int ARM_MIN_POS = 50;          // Min encoder arm position
    private boolean isArmExtended;              // Global Arm Position State
    private int armPosition;                    // Start position at default

    //*********************************************************************************
    // Dual Motor Control
    //*********************************************************************************
    private DcMotor hang1   = null;
    private DcMotor hang2   = null;
    static final double HANG_DEFAULT_POWER = 1.0;// Default Dual Power if needed
    static final int HANG_MAX_POS = 1500;        // Max encoder dual position
    static final int HANG_MIN_POS = 50;          // Min encoder dual position
    static final int HANG_INCREMENT = 10;          // INCREMENT
    private boolean isHangExtended;              // Global Dual State
    private int hangPosition;
    private double hangPower;


    //*********************************************************************************
    // IMU (Gyro)
    //*********************************************************************************
    /* Declare OpMode members. */
    private IMU             imu         = null;      // Control/Expansion Hub IMU
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // GoBilda 96mm wheels
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 1.0;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.8;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    //*********************************************************************************
    // Camera
    //*********************************************************************************
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    //private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Pixel",
    };
    private TfodProcessor tfod;     // The variable to store our instance of the TensorFlow Object Detection processor.
    private VisionPortal visionPortal;      // The variable to store our instance of the vision portal.


    //*********************************************************************************
    // Color Sensor
    //*********************************************************************************
    /** The colorSensor field will contain a reference to our color sensor hardware object */
    NormalizedColorSensor colorSensor;

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
    // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
    // can give very low values (depending on the lighting conditions), which only use a small part
    // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
    // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
    // colors will report at or near 1, and you won't be able to determine what color you are
    // actually looking at. For this reason, it's better to err on the side of a lower gain
    // (but always greater than  or equal to 1).
    // increase/decrease by 0.005
    float colorSensorGain = 2;   // Color Sensor GAIN, inc/dec by 0.005

    //*********************************************************************************
    // Odometry
    //*********************************************************************************
    static final double     ODOM_COUNTS_PER_REV     = 2000 ;    // 2000 for gobilda odom pod
    static final double     ODOM_DIAMETER           = 1.89 ;    // 48mm diameter
    static final double     ODOM_COUNTS_PER_INCH    = ODOM_COUNTS_PER_REV / (ODOM_DIAMETER * 3.1415);

    private DcMotor odometryX1   = null;    // Odometry Pods using encoders
    private DcMotor odometryY    = null;

    // Elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize robot
        initializeRobot();

        //Check current Mode, change mode to AUTONOMOUS if start and left bumper are pressed in init
        if(gamepad1.start && gamepad1.left_bumper) {
            currentMode = Mode.AUTONOMOUS;
            telemetry.addLine("SWITCHING TO AUTONOMOUS MODE");
        }

        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Waiting for Play...");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            // Check which mode, and run appropriate methods
            switch(currentMode) {
                case AUTONOMOUS:
                    boolean pathComplete = false;

                    // Run autonomous path
                    if(gamepad1.a) {
                        pathComplete = autoRed1();          // Autonomous path 1
                    }
                    // Run autonomous path
                    if(gamepad1.y) {
                        //pathComplete = autoRed2();        // Autonomous path 2
                    }
                    // return back to Red1 starting position
                    if(gamepad1.x) {
                        //pathComplete = autoRed1Start();      //Return back to start
                    }

                    //When path is done, switch back to TELE-OP, good for testing and resetting
                    if(pathComplete) {
                        currentMode = Mode.TELEOP;
                    }

                case TELEOP:

                    //Check current Mode, change mode to AUTONOMOUS if start and left bumper are pressed
                    if(gamepad1.start && gamepad1.left_bumper) {
                        currentMode = Mode.AUTONOMOUS;
                        telemetry.addLine("SWITCHING TO AUTONOMOUS MODE");
                    }

                    // check Color Sensor
                    /*
                    try {
                        checkColorSensor();
                    } finally {
                        // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
                        // as pure white, but it's too much work to dig out what actually was used, and this is good
                        // enough to at least make the screen reasonable again.
                        // Set the panel back to the default color
                        relativeLayout.post(new Runnable() {
                            public void run() { relativeLayout.setBackgroundColor(Color.WHITE); }
                        });
                    }

                     */

                    // Check camera
                    telemetryTfod();

                    // Check wrist controls - Y
                    checkWristControlsOneButtonState();

                    // Check claw controls - B
                    checkClawControlsOneButtonState();

                    // Check arm controls - left bumper
                    controlArmOneButtonState();

                    //Check Dual Motor Hang controls - X and A
                    controlDualTwoButtons();

                    // Check Mecanum drive controls - Left Joystick + Right Joystick
                    checkDriveControls3();

                    // Check Gyro Drive Controls - dpad up,down,left,right
                    checkGyroDriveControls();

                    break;

            }//end switch(currentMode)

            // Update telemetry output to driver hub
            sendTelemetry();
            //printField(field);
            telemetry.update();

        }//end while - main loop

    }//end runOpMode()

    /**
     * Initialize
     * Motors
     * Servo
     * Arm (using 1 DCMotor)
     */
    public void initializeRobot() {
        // Initialize components
        telemetry.addLine("Status: Initialization of Robot Started...");
        initializeMotors();
        initializeWrist();
        initializeClaw();
        initializeArm();
        initializeDualMotors();
        //initializeColorSensor();
        initTfod();
        initializeGyro();
        //initializeOdometry2Pods();
        telemetry.addLine("Status: Initialization of Robot Completed.");
        telemetry.update();     // update telemetry output
        sleep(3000);            // Sleep just to see telemetry initialization data
    }//end initializeRobot()

    /**
     * Initialize the Motors
     */
    public void initializeMotors() {
        // 4 Motor Drive

        // Initialize the drive motor variables.                                // Control Hub Ports
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");     // 0
        leftRearDrive  = hardwareMap.get(DcMotor.class, "leftRearDrive");       // 1
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");      // 2
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");    // 3

        // Set drive direction
        // CCW is positive by default, so change accordingly
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); //left side are FORWARD - CCW
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE); // right side are REVERSE - CW

        // Reset encoder at initialization
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Setup to run using encoder
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake motor MODE with zero power
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message with current position to indicate successful Encoder reset
        // Show the elapsed game time and wheel power.
        telemetry.addLine("Initialized Motors Position");
        telemetry.addData("LF:RF", "%7d:%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition() );
        telemetry.addData("LR:RR", "%7d:%7d", leftRearDrive.getCurrentPosition(), rightRearDrive.getCurrentPosition() );

    }//end initializeMotors()

    /**
     * Initialize the servo
     */
    public void initializeServo() {
        servo = hardwareMap.get(Servo.class, "servo");
        isServoOpen = true;
        servo.setPosition(SERVO_MIN_POS);       // Set position to default position

        // Display the current position
        telemetry.addData("Initializing Servo Position", "%4.2f", servoPosition);
    }//end initializeServo()

    /**
     * Initialize the CR servo
     */
    public void initializeCRServo() {
        crServo = hardwareMap.get(CRServo.class, "crServo");
        rotateServo = false;
        crServo.setPower(0);
        crServo.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("Initializing CRServo Power", "%4.2f", crServo.getPower());
    }//end initializeCRServo()


    /**
     * Initialize the wrist
     */
    public void initializeWrist() {
        wrist = hardwareMap.get(Servo.class, "wrist");
        isWristUp = false;
        wrist.setPosition(WRIST_MIN_POS);       // Set position to default position

        // Display the current position
        telemetry.addData("Initializing Wrist Position", "%4.2f", wristPosition);
    }//end initializeWrist()

    /**
     * Initialize the claw
     */
    public void initializeClaw() {
        claw = hardwareMap.get(Servo.class, "claw");
        isClawOpen = false;
        claw.setPosition(CLAW_MIN_POS);       // Set position to default position

        // Display the current position
        telemetry.addData("Initializing Claw Position", "%4.2f", clawPosition);
    }//end initializeClaw()

    /**
     * Initialize the arm
     */
    public void initializeArm() {
        // Initialize the drive motor variables
        arm  = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    // Reset encoder values
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // hold position with zero power
        // Initialize arm to starting point, buffer so it doesn't crash into motor or robot
        armPosition = ARM_MIN_POS;
        isArmExtended = false;
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(armPosition);
        arm.setPower(0.2);          // Slow speed to initial position, motor will try to keep position

        // Send telemetry message with current position to indicate successful Encoder reset
        telemetry.addData("Initializing Arm", "%7d", arm.getCurrentPosition());

    }//end initializeArm()

    /**
     * Initialize the dual motors for viper slides to hang
     */
    public void initializeDualMotors() {
        // Initialize the dual motor variables
        hang1  = hardwareMap.get(DcMotor.class, "hang1");
        hang2  = hardwareMap.get(DcMotor.class, "hang2");
        hang1.setDirection(DcMotor.Direction.FORWARD);
        hang2.setDirection(DcMotor.Direction.FORWARD);
        hang1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);    // Reset encoder values
        hang2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hang2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hang1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // hold position with zero power
        hang2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize arm to starting point, buffer so it doesn't crash into motor or robot
        hangPosition = HANG_MIN_POS;
        isHangExtended = false;
        hang1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang1.setTargetPosition(hangPosition);
        hang2.setTargetPosition(hangPosition);
        hang1.setPower(0.2);          // Slow speed to initial position, motor will try to keep position
        hang2.setPower(0.2);


        // Send telemetry message with current position to indicate successful Encoder reset
        telemetry.addData("Initializing Hang", "%7d", hang1.getCurrentPosition());

    }//end initializeDualMotors()

    /*
     * Initialize the color sensor
     */
    public void initializeColorSensor() {
        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        colorSensor.setGain(colorSensorGain);

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // Display the current value
        telemetry.addData("Initializing Color Sensor, Gain:", "%4.2f", colorSensorGain);
    }//end initializeServo()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

        telemetry.addLine("Initializing Camera and TFOD...");

    } // end method initTfod()


    /*
     * Initialize the imu / gyro using new IM SDK
     * Use with motor encoders enabled
     * Hub is Vertical so y is now yaw.
     */
    public void initializeGyro() {
        // Retrieve and initialize the IMU.
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");

        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
         *
         * Two input parameters are required to fully specify the Orientation.
         * The first parameter specifies the direction the printed logo on the Hub is pointing.
         * The second parameter specifies the direction the USB connector on the Hub is pointing.
         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        // Add to telemetry that Gyro is initialized after heading is reset
        imu.resetYaw();
        telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);
        telemetry.addData(">", "Initialized IMU/Gyro - Robot Heading = %4.0f", getHeading());
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));

    }//end initializeGyro()

    /*********************************************
     * Initialize the Odometry 2 Pods
     *********************************************/
    public void initializeOdometry2Pods() {

        // Initialize the drive motor variables.                        // Expansion Hub Ports
        // Check orientation of odometry pods and placement
        odometryX1 = hardwareMap.get(DcMotor.class, "odometryX1");      // 2
        odometryY = hardwareMap.get(DcMotor.class, "odometryY");        // 3

        // Reset encoder at initialization
        odometryX1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometryY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Setup to run using encoder
        odometryX1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odometryY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set starting position
        int[] coordinates = getCurrentRobotLocation(field);
        coordinatesOdometry[0] = 24 * (coordinates[0]-1);
        coordinatesOdometry[1] = 24 * (coordinates[1]-1);

        // Send telemetry message with current position to indicate successful Encoder reset
        telemetry.addLine("Initialized 2 Pod Odometry Positions in inches ");
        telemetry.addData("x:y", "%5.2f:%5.2f", coordinatesOdometry[0], coordinatesOdometry[0]);

    }// end initializeOdometry2Pods()

    /**
     *  Display the various control parameters while driving
     */
    private void sendTelemetry() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double[] currentCoordinates = getRobotLocationOdometry2PodsInches();

        telemetry.addData("Pos LF:RF -- Target LF:RF",  "%7d:%7d -- %7d:%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), leftFrontTarget, rightFrontTarget);
        telemetry.addData("Pos LR:RR -- Target LR:RR",  "%7d:%7d -- %7d:%7d", leftRearDrive.getCurrentPosition(), rightRearDrive.getCurrentPosition(), leftRearTarget, rightRearTarget);
        telemetry.addData("Odometry", "%5.2f : %5.2f", currentCoordinates[0], currentCoordinates[1]);
        telemetry.addData("Wheel Speeds LF:RF.", "%5.2f : %5.2f", leftFrontSpeed, rightFrontSpeed);
        telemetry.addData("Wheel Speeds LR:RR.", "%5.2f : %5.2f", leftRearSpeed, rightRearSpeed);
        telemetry.addLine("Yaw Current:Target  --  Roll  --  Pitch");
        telemetry.addData("      ", "%5.2f:%5.2f  --  %5.2f  --  %5.2f", orientation.getYaw(AngleUnit.DEGREES), targetHeading, orientation.getRoll(AngleUnit.DEGREES), orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, TURN_SPEED);
        telemetry.addData("RunTime:", "seconds: " + runtime.toString());

    }//end sendTelemetry()


    /*
     * Test motor control
     * Might have to Comment out other code in runOpMode to make other controls don't interfere
     */
    public void testMotors() {
        //Test motor direction based on two buttons pressed
        if(gamepad1.left_bumper && gamepad1.x) {        // X is left front
            leftFrontDrive.setPower(0.5);
        }
        else if(gamepad1.left_bumper && gamepad1.y) {   // Y is right front
            rightFrontDrive.setPower(0.5);
        }
        else if(gamepad1.left_bumper && gamepad1.a) {   // A is left rear
            leftRearDrive.setPower(0.5);
        }
        else if(gamepad1.left_bumper && gamepad1.b) {   // B is right rear
            rightRearDrive.setPower(0.5);
        }

    }//end testMotors()



    /*
     * Check controls for driving
     * From BasicOmniMode
     */
    public void checkDriveControls() {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw     = gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        leftFrontSpeed  = axial - lateral + yaw;
        leftRearSpeed   = axial + lateral + yaw;
        rightRearSpeed  = axial - lateral - yaw;
        rightFrontSpeed = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed));
        max = Math.max(max, Math.abs(leftRearSpeed));
        max = Math.max(max, Math.abs(rightRearSpeed));
        if (max > 1.0) {
            leftFrontSpeed  /= max;
            leftRearSpeed   /= max;
            rightFrontSpeed /= max;
            rightRearSpeed  /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontSpeed);
        leftRearDrive.setPower(leftRearSpeed);
        rightRearDrive.setPower(rightRearSpeed);
        rightFrontDrive.setPower(rightFrontSpeed);

        // Show the elapsed game time and wheel power/speed
        /* data outputted in sendTelemetry
        telemetry.addLine("*******Manual Driving******");
        telemetry.addData("LF,RF Power", "%4.2f, %4.2f", leftFrontSpeed, rightFrontSpeed);
        telemetry.addData("LR,RR Power", "%4.2f, %4.2f", leftRearSpeed, rightRearSpeed);
        */

    }//end checkDriveControls()

    /*
     * Check controls for driving
     * Version 2 - use trig to smooth it out
     * Need to test out values computed, may not work but not as intended
     * Watch out for inverted value of y - test it out first
     */
    public void checkDriveControls2() {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double turn = gamepad1.right_stick_x;
        telemetry.addData("r : angle : turn", "%7.4f : %7.4f : %7.4f", r,angle,turn);   // debug output to see values
        leftFrontSpeed = r * Math.cos(angle) + turn;
        leftRearSpeed = r * Math.sin(angle) + turn;
        rightRearSpeed = r * Math.cos(angle) - turn;
        rightFrontSpeed = r * Math.sin(angle) - turn;

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontSpeed);
        leftRearDrive.setPower(leftRearSpeed);
        rightRearDrive.setPower(rightRearSpeed);
        rightFrontDrive.setPower(rightFrontSpeed);

        // Show wheel power.
        /* Handled in sendTelemetry
        telemetry.addData("LF,RF Power", "%4.2f, %4.2f", leftFrontSpeed, rightFrontSpeed);
        telemetry.addData("LR,RR Power", "%4.2f, %4.2f", leftRearSpeed, rightRearSpeed);
        */

    }//end checkDriveControls2()

    /*
     * Check controls for driving
     * Version 3 - use squared value to have better slow speed precision
     */
    public void checkDriveControls3() {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw     = gamepad1.right_stick_x;

        int axial_sign = 1;
        int lateral_sign = 1;
        int yaw_sign = 1;

        // Check and save the sign of the original joystick input value
        if(axial < 0) {
            axial_sign = -1;
        }
        if(lateral < 0) {
            lateral_sign = -1;
        }
        if(yaw < 0) {
            yaw_sign = -1;
        }

        // Square the values and multiply by saved sign
        axial = axial_sign * axial * axial;
        lateral = lateral_sign * lateral * lateral;
        yaw = yaw_sign * yaw * yaw;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        leftFrontSpeed  = axial - lateral + yaw;
        leftRearSpeed   = axial + lateral + yaw;
        rightRearSpeed  = axial - lateral - yaw;
        rightFrontSpeed = axial + lateral - yaw;


        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        // May not be needed especially if joystick is -1 - 0 - 1.
        max = Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed));
        max = Math.max(max, Math.abs(leftRearSpeed));
        max = Math.max(max, Math.abs(rightRearSpeed));
        if (max > 1.0) {
            leftFrontSpeed  /= max;
            leftRearSpeed   /= max;
            rightRearSpeed  /= max;
            rightFrontSpeed /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontSpeed);
        leftRearDrive.setPower(leftRearSpeed);
        rightFrontDrive.setPower(rightFrontSpeed);
        rightRearDrive.setPower(rightRearSpeed);

        // Show wheel power.
        /* Handled in sendTelemetry
        telemetry.addData("LF,RF Power", "%4.2f, %4.2f", leftFrontSpeed, rightFrontSpeed);
        telemetry.addData("LR,RR Power", "%4.2f, %4.2f", leftRearSpeed, rightRearSpeed);
        */

    }//end checkDriveControls3()






    /**********************************************
     * rotate servo based on gamepad
     * y is to max
     * a is to min
     * Increments up to max position, Hold down button to move, off stops
     **********************************************/
    public void checkServoControls() {
        // Keep stepping up until we hit the max value.
        if (gamepad1.y) {
            servoPosition += INCREMENT;
            if(servoPosition >= SERVO_MAX_POS) {  // Don't increment past max
                servoPosition = SERVO_MAX_POS;
            }
        } else if (gamepad1.a) {
            servoPosition -= INCREMENT;
            if(servoPosition <= SERVO_MIN_POS) {  // Don't decrement past min
                servoPosition = SERVO_MIN_POS;
            }
        }

        // set servoPosition with the new increment;
        servo.setPosition(servoPosition);
        sleep(CYCLE_MS);            // maybe can comment out depending on time to get to position

        // IMPORTANT NOTE: getPosition() gets last servo position set
        // NOT the current real position, if servo gets stuck, value will not be correct
        telemetry.addData("Servo Target Position", "%4.2f", servo.getPosition());

    }//end checkServoControls()

    /**********************************************
     * rotate servo based on gamepad
     * y is to max
     * a is to min
     * Does not increment, Press button once to go to specific max and min
     * However, if servo gets stuck, servo may burn out trying to get to position
     **********************************************/
    public void checkServoControls2() {
        // if y is pressed, go to max position
        if (gamepad1.y) {
            servoPosition = SERVO_MAX_POS;
        } else if (gamepad1.a) {
            servoPosition = SERVO_MIN_POS;
        }

        // set servoPosition with the new target
        servo.setPosition(servoPosition);
        sleep(CYCLE_MS);            // maybe can comment out depending on time to get to position

        // IMPORTANT NOTE: getPosition() gets last servo position set
        // NOT the current real position, if servo gets stuck, value will not be correct
        telemetry.addData("Servo Target Position", "%4.2f", servo.getPosition());

    }//end checkServoControls2()

    /**********************************************
     * rotate servo based on gamepad
     * Y alternates between two positions only
     * Does not increment, Press only ONE button
     * However, if servo gets stuck, servo may burn out trying to get to position
     **********************************************/
    public void checkServoControlsOneButton() {
        // if y is pressed, set servoPosition to opposite of current MIN or MAX
        if (gamepad1.y) {
            if(servo.getPosition() >= SERVO_MAX_POS) {
                servoPosition = SERVO_MIN_POS;
            }
            else if(servo.getPosition() <= SERVO_MIN_POS) {
                servoPosition = SERVO_MAX_POS;
            }
        }

        // set servoPosition with the new target
        servo.setPosition(servoPosition);
        sleep(CYCLE_MS);            // maybe can comment out depending on time to get to position

        // IMPORTANT NOTE: getPosition() gets last servo position set
        // NOT the current real position, if servo gets stuck, value will not be correct
        telemetry.addData("Servo Target Position", "%4.2f", servo.getPosition());

    }//end checkServoControlsOneButton2()

    /**********************************************
     * rotate servo based on gamepad
     * Y alternates between two states using global state variable isOpen
     * Does not increment, Press only ONE button
     * However, if servo gets stuck, servo may burn out trying to get to position
     **********************************************/
    public void checkServoControlsOneButtonState() {
        // if y is pressed, set servoPosition to opposite saved state.
        if (gamepad1.y) {
            if(isServoOpen) {
                servoPosition = SERVO_MIN_POS;
            }
            else {
                servoPosition = SERVO_MAX_POS;
            }
            isServoOpen = !isServoOpen;
        }

        // set servoPosition with the new target
        servo.setPosition(servoPosition);
        sleep(CYCLE_MS);            // maybe can comment out depending on time to get to position

        // IMPORTANT NOTE: getPosition() gets last servo position set
        // NOT the current real position, if servo gets stuck, value will not be correct
        telemetry.addData("Servo Target Position", "%4.2f", servo.getPosition());

    }//end checkServoControlsOneButtonState()

    /**********************************************
     * wrist rotate servo based on gamepad
     * Y alternates between two states using global state variable isWristUp
     * Does not increment, Press only ONE button
     * However, if servo gets stuck, servo may burn out trying to get to position
     **********************************************/
    public void checkWristControlsOneButtonState() {
        // if y is pressed, set servoPosition to opposite saved state.
        if (gamepad1.y) {
            if(isWristUp) {
                wristPosition = WRIST_MIN_POS;
            }
            else {
                wristPosition = WRIST_MAX_POS;
            }
            isWristUp = !isWristUp;
        }

        // set servoPosition with the new target
        wrist.setPosition(wristPosition);
        sleep(CYCLE_MS);            // maybe can comment out depending on time to get to position

        // IMPORTANT NOTE: getPosition() gets last servo position set
        // NOT the current real position, if servo gets stuck, value will not be correct
        telemetry.addData("Wrist Target Position", "%4.2f", wrist.getPosition());

    }//end checkWristControlsOneButtonState()

    /**********************************************
     * claw rotate servo based on gamepad
     * B alternates between two states using global state variable isOpen
     * Does not increment, Press only ONE button
     * However, if servo gets stuck, servo may burn out trying to get to position
     **********************************************/
    public void checkClawControlsOneButtonState() {
        // if b is pressed, set clawPosition to opposite saved state.
        if (gamepad1.b) {
            if(isClawOpen) {
                clawPosition = CLAW_MIN_POS;
            }
            else {
                clawPosition = CLAW_MAX_POS;
            }
            isClawOpen = !isClawOpen;
        }

        // set servoPosition with the new target
        claw.setPosition(clawPosition);
        sleep(CYCLE_MS);            // maybe can comment out depending on time to get to position

        // IMPORTANT NOTE: getPosition() gets last servo position set
        // NOT the current real position, if servo gets stuck, value will not be correct
        telemetry.addData("Claw Target Position", "%4.2f", claw.getPosition());

    }//end checkClawControlsOneButtonState()


    /**********************************************
     * rotate a continuous revolution servo
     * Y alternates between on/off
     * However, if servo gets stuck, servo may burn out trying to get to position
     **********************************************/
    public void checkCRServoControlsOneButtonState() {
        // if y is pressed, rotate CRServo
        if (gamepad1.y) {
            if(rotateServo) {                   // rotateServo is true, it's rotating, so stop it
                crServo.setPower(0);
            }
            else {                              // rotateServo is false, so start rotating
                crServo.setPower(1);
            }
        }

        sleep(CYCLE_MS);            // maybe can comment out depending on time to get to position

        telemetry.addData("CRServo Power", "%4.2f", crServo.getPower());

    }//end checkCRServoControlsOneButtonState()







    /**********************************************
     * control Arm
     * using 1 DCMotor with just power
     * Using triggers as they have values from 0 to 1.
     **********************************************/
    public void controlArm() {
        // Using right trigger to go "up/open"
        // left trigger is to go down/close"
        double armPower = gamepad1.right_trigger - gamepad1.left_trigger;

        // Check to make sure arm isn't past max allowed position
        if(arm.getCurrentPosition() >= ARM_MAX_POS) {
            arm.setPower(0);
        }
        else if(arm.getCurrentPosition() <= ARM_MIN_POS) {
            arm.setPower(0);
        }
        else {
            arm.setPower(armPower);
        }

        telemetry.addData("Arm Power, Pos", "%4.2f, %7d", armPower, arm.getCurrentPosition());

    }//end controlArm()

    /**********************************************
     * control Arm
     * Using left and right bumpers to go to position
     * using 1 DCMotor using ENCODER with RUN_TO_POSITION to ARM_MAX_POS
     * if weight is too much for motor to hold, use zero power brake hold in init
     **********************************************/
    public void controlArm2() {
        //Use encoder RUN_TO_POSITION to run motor to maintain position in init

        if(gamepad1.right_bumper) {       // up sets target to max
            armPosition = ARM_MAX_POS;
        }
        else if(gamepad1.left_bumper) {   // down sets target to min
            armPosition = ARM_MIN_POS;
        }

        // Go to position using ENCODER RUN TO POSITION
        arm.setTargetPosition(armPosition);
        double armPower = ARM_DEFAULT_POWER;
        arm.setPower(armPower);

        telemetry.addData("Arm Power, Pos, Target", "%4.2f, %7d, %7d", armPower, arm.getCurrentPosition(), armPosition);


    }//end controlArm2()

    /**********************************************
     * control Arm
     * Using one button only, right bumper
     * using 1 DCMotor using ENCODER with RUN_TO_POSITION to alternate between ARM_MAX_POS or ARM_MIN_POS
     * if weight is too much for motor to hold, use zero power brake hold in init
     * Downside that you can't alternate direction unless arm gets to other position, if arm is stuck, button won't work
     **********************************************/
    public void controlArmOneButton() {
        //Confirm encoder in mode RUN_TO_POSITION to run motor to maintain position in init

        if(gamepad1.right_bumper) {
            if(arm.getCurrentPosition() >= ARM_MAX_POS) {   // if at MAX, switch to MIN
                armPosition = ARM_MIN_POS;
            }
            else if(arm.getCurrentPosition() <= ARM_MIN_POS) {  // if at MIN, switch to MAX
                armPosition = ARM_MAX_POS;
            }
        }

        // Go to position using ENCODER RUN TO POSITION
        arm.setTargetPosition(armPosition);
        double armPower = ARM_DEFAULT_POWER;
        arm.setPower(armPower);

        telemetry.addData("Arm Power, Pos, Target", "%4.2f, %7d, %7d", armPower, arm.getCurrentPosition(), armPosition);

    }//end controlArmOneButton()

    /**********************************************
     * control Arm
     * Using one button only, right bumper and keeping isArmExtended state.
     * using 1 DCMotor using ENCODER with RUN_TO_POSITION to alternate between ARM_MAX_POS or ARM_MIN_POS
     * if weight is too much for motor to hold, use zero power brake hold in init
     * Allows user to tap button to reverse direction even if arm didn't get to target position yet
     **********************************************/
    public void controlArmOneButtonState() {
        //Confirm encoder in mode RUN_TO_POSITION to run motor to maintain position in init

        if(gamepad1.right_bumper) {
            if(isArmExtended) {             // check current Arm state
                armPosition = ARM_MIN_POS;
            }
            else {
                armPosition = ARM_MAX_POS;
            }
            isArmExtended = !isArmExtended;
        }

        // Go to position using ENCODER RUN TO POSITION
        arm.setTargetPosition(armPosition);
        double armPower = ARM_DEFAULT_POWER;
        arm.setPower(armPower);

        telemetry.addData("Arm Power, Pos, Target", "%4.2f, %7d, %7d", armPower, arm.getCurrentPosition(), armPosition);


    }//end controlArmOneButtonState()




    /**********************************************
     * control 2 Motors in sync for hanging with viper slides
     * Using two buttons, x to go up, a to go down
     * uses RUN_TO_POSITION to hold position
     * if weight is too much for motor to hold, use zero power brake hold in init
     * Allows user to tap button to reverse direction even if arm didn't get to position yet
     **********************************************/
    public void controlDualTwoButtons() {

        if(gamepad1.x) {                                        // go up
            hangPosition += HANG_INCREMENT;
            if(hang1.getCurrentPosition() >= HANG_MAX_POS) {    // make sure not past max
                hangPosition = HANG_MAX_POS;
            }
        }
        else if(gamepad1.a) {                                   //go down
            hangPosition -= HANG_INCREMENT;
            if(hang1.getCurrentPosition() <= HANG_MIN_POS) {   // make sure not past min
                hangPosition = HANG_MIN_POS;
            }
        }

        // Go to position using ENCODER RUN TO POSITION
        hang1.setTargetPosition(hangPosition);
        hang2.setTargetPosition(hangPosition);
        double hangPower = HANG_DEFAULT_POWER;
        hang1.setPower(hangPower);
        hang2.setPower(hangPower);

        telemetry.addData("Hang Power, Pos", "%4.2f, %7d", hangPower, hang1.getCurrentPosition());

    }//end controlDualTwoButtons()

    /**********************************************
     * control 2 Motors in sync for hanging with viper slides
     * Using one button only, right bumper and keeping isHangExtended state.
     * using 2 DCMotor using ENCODER with RUN_TO_POSITION to alternate between min and max
     * if weight is too much for motor to hold, use zero power brake hold in init
     * Allows user to tap button to reverse direction even if arm didn't get to position yet
     **********************************************/
    public void controlDualOneButtonState() {
        //Confirm encoder in mode RUN_TO_POSITION to run motor to maintain position in init

        if(gamepad1.right_bumper) {
            if(isHangExtended) {                // Check current Motor state
                hangPosition = HANG_MIN_POS;
            }
            else {
                hangPosition = HANG_MAX_POS;
            }
            isHangExtended = !isHangExtended;
        }

        // Go to position using ENCODER RUN TO POSITION
        hang1.setTargetPosition(hangPosition);
        hang2.setTargetPosition(hangPosition);
        double hangPower = HANG_DEFAULT_POWER;
        hang1.setPower(hangPower);
        hang2.setPower(hangPower);

        telemetry.addData("Dual Power, Pos, Target", "%4.2f, %7d, %7d", hangPower, hang1.getCurrentPosition(), hangPosition);

    }//end controlDualOneButtonState()

    /*********************************************
     * Color Sensor just reports the color values it sees
     *********************************************/
    protected void checkColorSensor() {
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */

        // In checkColorSensor, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        final float[] hsvValues = new float[3];

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addData("R:G:B", "%.3f:%.3f:%.3f", colors.red, colors.green, colors.blue);
        telemetry.addData("Hue:Sat:Value:Alpha", "%.3f:%.3f:%.3f:%.3f", hsvValues[0],hsvValues[1],hsvValues[2],colors.alpha);

        /* If this color sensor also has a distance sensor, display the measured distance.
         * Note that the reported distance is only useful at very close range, and is impacted by
         * ambient light and surface reflectivity. */
        /*
        if (colorSensor instanceof DistanceSensor) {
            telemetry.addData("Distance (inches)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.INCH));
        }

         */

        // Change the Robot Controller's background color to match the color detected by the color sensor.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
            }
        });

    }//end checkColorSensor

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected:", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("Pos (x,y) -- Size w x l", "(%.0f,%.0f) -- %.0fx%.0f", x, y, recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()


    /*
     * Check controls for driving
     * Initially limit to turns at 0,90,-180, -90
     * Only allow turning in 90 degrees increments so only 4 headings
     * Accounts for Euler angles
     * Go forward/backward one tile 24 inches for each press up/down
     * Reset heading
     */
    public void checkGyroDriveControls() {

        // Reset heading/Yaw by pressing both up and left at same time
        if(gamepad1.dpad_up && gamepad1.dpad_left) {
            imu.resetYaw();
        }

        double newHeading = getHeading();   // get currentHeading

        // IMU is in euler angles so 0 to 179, then jumps to -180
        // Test IMU angles
        // use dpad left and right to rotate left and right 90 degrees
        // Turning
        if(gamepad1.dpad_left && !gamepad1.dpad_up) {
            telemetry.addLine("TEST GYRO TURNING LEFT 90");
            newHeading += 90;
            if(newHeading >= 180) {
                newHeading -= 360;
            }
            turnToHeading(TURN_SPEED, newHeading);
            holdHeading(TURN_SPEED,newHeading,0.25);     // Hold for quarter sec, reduce if steady
        }
        else if(gamepad1.dpad_right) {
            telemetry.addLine("TEST GYRO TURNING RIGHT 90");
            newHeading -= 90;
            if(newHeading <= -180) {
                newHeading += 360;
            }
            turnToHeading(TURN_SPEED, newHeading);
            holdHeading(TURN_SPEED,newHeading,0.25);     // Hold for quarter sec, reduce if steady
        }

        // Check heading to determine which way to update robot location
        // 1 degree leeway, test and change angles based on imu
        int deltaX = 0;
        int deltaY = 0;
        if(getHeading() > -1.0 && getHeading() < 1.0 ) {
            if(alliance.equals("blue")) {       // if starting orientation is blue
                deltaX = 1;
                deltaY = 0;
            }
            else {
                deltaX = -1;
                deltaY = 0;
            }
        }
        else if(getHeading() > 89.0 && getHeading() < 91.0 ) {
            if(alliance.equals("blue")) {
                deltaX = 0;
                deltaY = -1;
            }
            else {
                deltaX = 0;
                deltaY = 1;
            }

        }
        else if(getHeading() > -181.0 && getHeading() < -179.0 ) {
            if(alliance.equals("blue")) {
                deltaX = -1;
                deltaY = 0;
            }
            else {
                deltaX = 1;
                deltaY = 0;
            }
        }
        else if(getHeading() < -89 && getHeading() > -91.0 ) {
            if(alliance.equals("blue")) {
                deltaX = 0;
                deltaY = 1;
            }
            else {
                deltaX = 0;
                deltaY = -1;
            }
        }

        // Drive straight one tile 24 inches
        if(gamepad1.dpad_up && !gamepad1.dpad_left) {
            telemetry.addLine("TEST GYRO DRIVE STRAIGHT FORWARD");
            driveStraight(DRIVE_SPEED, 24, newHeading);

        }
        else if(gamepad1.dpad_down) {
            telemetry.addLine("TEST GYRO DRIVE STRAIGHT BACKWARD");
            driveStraight(DRIVE_SPEED, -24, newHeading);
        }

        field = updateRobotLocation(field,deltaX,deltaY);   //update robot location

    }//end checkGyroDriveControls()

    /*
     * Check dpad controls for gyro and odometry 2 pod driving
     * Initially limit to turns at 0,90,-180, -90
     * Only allow turning in 90 degrees increments so only 4 headings
     * Accounts for Euler angles
     */
    public void checkGyroOdometry2PodsDriveControls() {

        // Reset heading/Yaw by pressing both up and left at same time
        if(gamepad1.dpad_up && gamepad1.dpad_left) {
            imu.resetYaw();
        }

        double newHeading = getHeading();   // get currentHeading

        // IMU is in euler angles so 0 to 179, then jumps to -180
        // Test IMU angles
        // use dpad left and right to rotate left and right 90 degrees
        // Turning
        if(gamepad1.dpad_left && !gamepad1.dpad_up) {
            telemetry.addLine("TEST GYRO/2POD TURNING LEFT 90");
            newHeading += 90;
            if(newHeading >= 180) {
                newHeading -= 360;
            }
            turnToHeading(TURN_SPEED, newHeading);
            holdHeading(TURN_SPEED, newHeading,0.25);     // Hold for quarter sec, reduce if steady
        }
        else if(gamepad1.dpad_right) {
            telemetry.addLine("TEST GYRO/2POD TURNING RIGHT 90");
            newHeading -= 90;
            if(newHeading <= -180) {
                newHeading += 360;
            }
            turnToHeading(TURN_SPEED, newHeading);
            holdHeading(TURN_SPEED, newHeading,0.25);     // Hold for quarter sec, reduce if steady
        }

        // Check heading to determine which way to update robot location
        // 1 degree leeway, test and change angles based on imu
        int deltaX = 0;
        int deltaY = 0;
        if(getHeading() > -1.0 && getHeading() < 1.0 ) {
            if(alliance.equals("blue")) {       // if starting orientation is blue
                deltaX = 1;
                deltaY = 0;
            }
            else {
                deltaX = -1;
                deltaY = 0;
            }
        }
        else if(getHeading() > 89.0 && getHeading() < 91.0 ) {
            if(alliance.equals("blue")) {
                deltaX = 0;
                deltaY = -1;
            }
            else {
                deltaX = 0;
                deltaY = 1;
            }

        }
        else if(getHeading() > -181.0 && getHeading() < -179.0 ) {
            if(alliance.equals("blue")) {
                deltaX = -1;
                deltaY = 0;
            }
            else {
                deltaX = 1;
                deltaY = 0;
            }
        }
        else if(getHeading() < -89 && getHeading() > -91.0 ) {
            if(alliance.equals("blue")) {
                deltaX = 0;
                deltaY = 1;
            }
            else {
                deltaX = 0;
                deltaY = -1;
            }
        }

        // Drive straight one tile 24 inches
        if(gamepad1.dpad_up && !gamepad1.dpad_left) {
            telemetry.addLine("TEST GYRO/2POD FORWARD");
            forward(24);

        }
        else if(gamepad1.dpad_down) {
            telemetry.addLine("TEST GYRO/2POD BACKWARD");
            backward(24);
        }

        field = updateRobotLocation(field,deltaX,deltaY);   //update robot location

    }//end checkGyroOdometry2PodsDriveControls()


    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Linear, have to wait until drive is complete
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed, double distance, double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftFrontTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            leftRearTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            rightRearTarget = rightFrontDrive.getCurrentPosition() + moveCounts;
            rightFrontTarget = rightFrontDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            leftRearDrive.setTargetPosition(leftRearTarget);
            rightRearDrive.setTargetPosition(rightRearTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);

            // IMPORTANT REMINDER
            // RUN_TO_POSITION ignores motor direction and power
            // It will rotate motor to get to correct encoder target
            // If it's reversed, test each motor rotation direction and encoder value individually
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() &&
                    (leftFrontDrive.isBusy() && leftRearDrive.isBusy() &&
                            rightFrontDrive.isBusy() && rightRearDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                //sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }//end driveStraight()

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // Encoder targets not used in turning but setting back to 0 if turning
        leftFrontTarget = 0;
        leftRearTarget = 0;
        rightRearTarget = 0;
        rightFrontTarget = 0;

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            //sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);

    }//end turnToHeading()

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            //sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }//end holdHeading()

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);

    }//end getSteeringCorrection()

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftFrontSpeed = drive - turn;
        leftRearSpeed  = drive - turn;
        rightRearSpeed = drive + turn;
        rightFrontSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed));
        max = Math.max(max, Math.abs(leftRearSpeed));
        max = Math.max(max, Math.abs(rightRearSpeed));
        if (max > 1.0)
        {
            leftFrontSpeed /= max;
            leftRearSpeed /= max;
            rightRearSpeed /= max;
            rightFrontSpeed /= max;
        }

        leftFrontDrive.setPower(leftFrontSpeed);
        leftRearDrive.setPower(leftRearSpeed);
        rightRearDrive.setPower(rightRearSpeed);
        rightFrontDrive.setPower(rightFrontSpeed);

    }//end moveRobot()

    /**
     * Modify powers to handle strafing with odometry
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobotStrafe(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftFrontSpeed = -drive - turn;
        leftRearSpeed  = drive - turn;
        rightRearSpeed = -drive + turn;
        rightFrontSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed));
        max = Math.max(max, Math.abs(leftRearSpeed));
        max = Math.max(max, Math.abs(rightRearSpeed));
        if (max > 1.0)
        {
            leftFrontSpeed /= max;
            leftRearSpeed /= max;
            rightRearSpeed /= max;
            rightFrontSpeed /= max;
        }

        leftFrontDrive.setPower(leftFrontSpeed);
        leftRearDrive.setPower(leftRearSpeed);
        rightRearDrive.setPower(rightRearSpeed);
        rightFrontDrive.setPower(rightFrontSpeed);

    }//end moveRobotStrafe()




    /**
     * read the Robot heading directly from the IMU (in degrees)
     * @return          Current Heading in Degrees
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }//end getHeading()

    /**
     * return current location in int array x,y
     * Field is 6x6 but oversized 8x8 field to avoid array out of bounds error
     * Print starting at indices 1 to 6 (inclusive)
     * Potential error if there are two R's in field
     * @return                      int coordinate[] for x,y of Robot, -1,-1 if not found
     */
    public int[] getCurrentRobotLocation(char[][] gameField) {
        int[] coordinates = new int[2];

        for(int i = 1; i <= 6; i++) {
            for(int j = 1; j <= 6; j++) {
                if(gameField[i][j] == 'R') {
                    coordinates[0] = i;
                    coordinates[1] = j;
                    return coordinates;
                }
            }
        }

        // Otherwise not found
        coordinates[0] = -1;
        coordinates[1] = -1;
        return coordinates;

    }//end getCurrentRobotLocation()

    /**
     * update Robot location in custom coordinate system
     * only accurate with Gyro Movement dpad controls checkGyroControls()
     * Field is 6x6 but oversized 8x8 field to avoid array out of bounds error
     * Print starting at indices 1 to 6 (inclusive)
     * @param gameField char [][] gamefield current field
     * @param changeX delta in x movement
     * @param changeY delta in y movement
     * @return new field with updated location
     */
    public char[][] updateRobotLocation(char[][] gameField, int changeX, int changeY) {

        // Find and save current Location
        int[] currentLocation = getCurrentRobotLocation(gameField);

        // check if robot not found, exit method and return current field
        if(currentLocation[0] == -1 && currentLocation[1] == -1) {
            return gameField;
        }

        // Check gamefield setting new array
        // check this code.... scope...

        // Update current Location with changeX, changeY
        // If out of bounds, output error line
        /*
        if(currentLocation[0]+changeX < 1 || currentLocation[0]+changeX > 6 ||
                currentLocation[1]+changeY < 1 || currentLocation[1]+changeY > 6 )
        {
            telemetry.addData("Cannot update Robot location to", "%5d:%5d", currentLocation[0]+changeX, currentLocation[1]+changeY);
        }
        else {
            field[currentLocation[0]+changeX][currentLocation[1]+changeY] = 'R';    // update current location
        }
         */

        // Update current Location with changeX, changeY
        // check maze code value, * out of bounds, B is the board
        if(gameField[currentLocation[0]][currentLocation[1]] == '*' || gameField[currentLocation[0]][currentLocation[1]] == 'B' )
        {
            telemetry.addData("Cannot update Robot location to", "(%2d:%2d)", currentLocation[0]+changeX, currentLocation[1]+changeY);
        }
        else {
            gameField[currentLocation[0]][currentLocation[1]] = 'X';                    // Reset current location
            gameField[currentLocation[0]+changeX][currentLocation[1]+changeY] = 'R';    // update current location
        }

        return gameField;

    }//end updateRobotLocation()

    /**
     * Autonomous - Follow preferred path
     * Can go only forward/backward, turn left right for now, no diagonal movement
     *
     * @param gameField     2D char array of current field
     */
    public void followPath(char[][] gameField) {
        double direction = findNextPath(gameField);

        // 0 is up, 1 is right, 2 is down, 3 is left,-1 if not found
        if(direction == 90.0) {            // up
            turnToHeading(TURN_SPEED,direction);
            driveStraight(DRIVE_SPEED,24,direction);
        }
        else if(direction == 0.0) {       // right
            turnToHeading(TURN_SPEED,direction);
            driveStraight(DRIVE_SPEED,24,direction);
        }
        else if(direction == 270.0) {       // down
            turnToHeading(TURN_SPEED,direction-360);
            driveStraight(DRIVE_SPEED,24,direction-360);
        }
        else if(direction == 180.0) {       // left
            turnToHeading(TURN_SPEED,direction-360);
            driveStraight(DRIVE_SPEED,24,direction-180);
        }
        else if(direction == -1) {       // no P found
            telemetry.addLine("No more P found. Path ended.");
        }

    }//end followPath()

    /**
     * Find the next P surrounding the robot
     * @param   gameField       2D char array of current field
     * @return                  angle of next path, 0 is right, 90 is up, 180 is left, 270 is right
     */
    public double findNextPath(char[][] gameField) {
        // delta[0] is delta x
        // delta[1] is delta y
        int[] delta = new int[2];

        int[] robotLocation = getCurrentRobotLocation(gameField);

        //Check for P in 4 locations
        if(gameField[robotLocation[0]][robotLocation[1]-1] == 'P' ) {       // above/up
            return 90.0;
        }
        else if(gameField[robotLocation[0]+1][robotLocation[1]] == 'P' ) {   // right
            return 0.0;
        }
        else if(gameField[robotLocation[0]][robotLocation[1]+1] == 'P' ) {   // below
            return 270.0;
        }
        else if(gameField[robotLocation[0]-1][robotLocation[1]] == 'P' ) {   // left
            return 180.0;
        }

        return -1.0;  // If no path found, return -1

    }//end findNextPath()



    /**
     * print the field
     * Field is 6x6 but oversized 8x8 field to avoid array out of bounds error
     * Print starting at indices 1 to 6 (inclusive)
     * @param gameField     2D char array of current field
     */
    public void printField(char[][] gameField) {
        // Can't print per char so print per row
        for(int i = 1; i <= 6; i++) {
            telemetry.addLine(  String.valueOf(gameField[i][1]) + " " +
                    String.valueOf(gameField[i][2]) + " " +
                    String.valueOf(gameField[i][3]) + " " +
                    String.valueOf(gameField[i][4]) + " " +
                    String.valueOf(gameField[i][5]) + " " +
                    String.valueOf(gameField[i][6]));
        }

    }//end printField()












    /**
     * getRobotLocationOdometry2PodsInches
     * return current location in double array x,y in inches
     * @return              double coordinate[] for x,y of Robot in inches
     */
    public double[] getRobotLocationOdometry2PodsInches() {

        double[] coordinates = new double[2];

        // Get the current odometery readings in inches
        double deltaX1 = odometryX1.getCurrentPosition() / ODOM_COUNTS_PER_INCH;        // x
        double deltaY = odometryY.getCurrentPosition() / ODOM_COUNTS_PER_INCH;;        // y

        // If starting on red alliance side, starting orientation is opposite.
        if(alliance.equals("red")) {
            deltaX1 *= -1;
            deltaY *= -1;
        }

        // Calc current odometry readings adjusting for starting position
        coordinates[0] = coordinatesOdometry[0] + deltaX1;
        coordinates[1] = coordinatesOdometry[1] + deltaY;

        return coordinates;

    }//end getRobotLocationOdometry2PodsInches()

    /**
     * forward
     * move forward specific number of inches - method needs odmetry setup
     * Try to finalize coordinate system of field
     * @param   distance        inches to move forward in same heading
     */
    public void forward(double distance) {
        // modification of existing driveStraight method, reuse existing moveRobot method
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position
            double moveCounts = (distance * ODOM_COUNTS_PER_INCH);  // in ticks

            // Get current heading, depending on heading update target in x or y direction
            double currentHeading = getHeading();
            double targetX1 = odometryX1.getCurrentPosition();
            double targetY = odometryY.getCurrentPosition();

            // Fix starting orientation by multiplying encoder with negative later
            if(currentHeading == 0.0) {
                if(alliance.equals("blue")) {
                    targetX1 += moveCounts;
                }
                else {
                    targetX1 -= moveCounts;
                }
            }
            else if(currentHeading == 90.0) {
                if(alliance.equals("blue")) {
                    targetY -= moveCounts;
                }
                else {
                    targetY += moveCounts;
                }

            }
            else if(currentHeading == 180.0) {
                if(alliance.equals("blue")) {
                    targetX1 -= moveCounts;
                }
                else {
                    targetX1 += moveCounts;
                }
            }
            else if(currentHeading == -90) {
                if(alliance.equals("blue")) {
                    targetY += moveCounts;
                }
                else {
                    targetY -= moveCounts;
                }
            }

            // Start driving straight and then enter the control loop
            driveSpeed = DRIVE_SPEED;
            moveRobot(driveSpeed, 0);

            // keep looping while we are still active and not reached target
            while (opModeIsActive() && !checkReachTarget(targetX1,targetY)) {

                // Determine required steering to keep on current heading
                turnSpeed = getSteeringCorrection(currentHeading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

            }

            // Stop all motion
            moveRobot(0, 0);
        }

    }//end forward()

    /**
     * back
     * move backward specific number of inches - method needs odmetry setup
     * @param   distance        inches(positive) to move backward in same heading, positive to move backwards
     */
    public void backward(double distance) {
        // just call forward with negative distance
        forward(-distance);
    }//end back()


    /**
     * strafeLeft
     * strafe left specific number of inches - method need odmetry setup
     * @param   distance        inches(positive) to strafe left
     */
    public void strafeLeft(double distance) {
        //Modified and combined driveStraight code and use new moveRobotStrafe
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position
            double moveCounts = (distance * ODOM_COUNTS_PER_INCH);  // in ticks

            // Get current heading, depending on heading update target in x or y direction
            double currentHeading = getHeading();
            double targetX1 = odometryX1.getCurrentPosition();
            double targetY = odometryY.getCurrentPosition();

            if(currentHeading == 0.0) {
                targetX1 += moveCounts;
            }
            else if(currentHeading == 90.0) {
                targetY += moveCounts;
            }
            else if(currentHeading == 180.0) {
                targetX1 -= moveCounts;
            }
            else if(currentHeading == -90) {
                targetY -= moveCounts;
            }

            // Start driving straight and then enter the control loop
            driveSpeed = DRIVE_SPEED;
            moveRobotStrafe(driveSpeed, 0);

            // keep looping while we are still active and not reached target
            while (opModeIsActive() && !checkReachTarget(targetX1,targetY)) {

                // Determine required steering to keep on current heading
                turnSpeed = getSteeringCorrection(currentHeading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobotStrafe(driveSpeed, turnSpeed);

            }

            // Stop all motion
            moveRobotStrafe(0, 0);

        }

    }//end strafeLeft()

    /**
     * strafeRight
     * strafe right specific number of inches - method need odmetry setup
     * @param   distance        inches(positive) to strafe left
     */
    public void strafeRight(double distance) {
        //just call strafeLeft with negative distance
        strafeLeft(-distance);
    }//end strafeRight()

    /**
     * checkReachTarget
     * checks if robot reached it's target coordinates in inches
     * @param   targetX1        targetX coordinate to check if reached
     * @param   targetY         targetY coordinate to check if recahed
     * @return                  true if reached target within buffer, false if not yet at target
     */
    public boolean checkReachTarget(double targetX1, double targetY) {
        // check if current encoder values in are within a small range
        if(targetX1 > odometryX1.getCurrentPosition() - 5 &&
                targetX1 <= odometryY.getCurrentPosition() + 5) {

            if(targetY > odometryY.getCurrentPosition() - 5 &&
                    targetY <= odometryY.getCurrentPosition() + 5) {
                return true;
            }

        }

        return false;   // did not reach target yet

    }//end checkReachTarget()


    /**
     * autoRed1
     * Runs autonomous path for Red1
     * @return          when path is complete, return true
     */
    public boolean autoRed1() {
        driveStraight(DRIVE_SPEED,24.0,getHeading());
        turnToHeading(TURN_SPEED,90);
        driveStraight(DRIVE_SPEED,24.0,getHeading());
        return true;
    }//end autoRed1()


}//end OptimusPrime class

