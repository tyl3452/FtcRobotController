/**********************************************
 * @author MrLiu
 * WALL-E
 * A fun-loving robot who wants to help
 * Dual Auto and Tele-Op Linear OpMode
 * Gyro for angles and wheel encoders
 * Features:
 * 4 Motor Mecanum/Omni Drive
 * 2 Servos, left hand is continuous, right hand is regular
 * 1 DCMotor w/encoder, can be used for arm or anything else
 * IMU (Gyro) on Expansion Hub
 * 1 Camera using TF
 * 2 Color Sensors
 **********************************************/

package org.firstinspires.ftc.teamcode;

// Basic imports
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp(name="WALL-E TeleOp", group="Linear Opmode")
public class WallETeleOp extends LinearOpMode {
    /* Declare Global variables */

    // Control the current Mode
    boolean teleOpMode = true;          // set TeleOp as default mode

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
    private Servo leftHand = null;              // Regular servo
    private CRServo rightHand = null;           // CR Servo, use setPower() instead of setPosition()
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    = 2;        // period of each cycle
    static final double LEFT_HAND_MAX_POS     = 1.0;      // Maximum rotational position
    static final double LEFT_HAND_MIN_POS     = 0.0;      // Minimum rotational position
    private boolean isLeftHandUp;                         // servo Open State, Open is at MAX
    private boolean rotateRightHand;                      // CR Servo state
    private double leftHandPosition;

    //*********************************************************************************
    // 1 Motor Control using encoder
    //*********************************************************************************
    private DcMotor motor   = null;
    static final double MOTOR_DEFAULT_POWER = 1.0;// Default motor Power if needed
    static final int MOTOR_MAX_POS = 1500;        // Max encoder motor position
    static final int MOTOR_MIN_POS = 50;          // Min encoder motor position
    private boolean isMotorTargetPositionReached; // Global motor Position State
    private int motorPosition;                      // Start position at default

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

    // Elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize robot
        initializeRobot();

        // Check current Mode, change mode to AUTONOMOUS if start and left bumper are pressed
        // and held during init
        if(gamepad1.left_bumper && gamepad1.start) {
            teleOpMode = false;
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
            if(!teleOpMode) {       // if autonomous
                initTfod();         // Init camera again, maybe change logic to leave cam on always later
                boolean pathComplete = false;

                // Check camera - don't need ot check camera in manual mode
                telemetryTfod();

                // Run autonomous path
                // call / add autonomous code here
                sleep(1000);            //  remove this sleep and pathComplete later
                pathComplete = true;

                //When path is done, switch back to TELE-OP, good for testing and resetting
                if (pathComplete) {
                    teleOpMode = true;
                }

                // Save more CPU resources when camera is no longer needed in teleOpMode
                visionPortal.close();   // maybe don't close and leave always on, think change later
            }
            else {
                //Check current Mode, change mode to AUTONOMOUS if start and left bumper are pressed
                if (gamepad1.left_bumper && gamepad1.start) {
                    teleOpMode = false;
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

                // Check left hand controls - X
                checkLeftHandControlsOneButtonState();

                // Check right hand controls - B
                checkRightHandControlsOneButtonState();

                // Check Mecanum drive controls - Left Joystick + Right Joystick
                checkDriveControls3();

                // Check Gyro Drive Controls - dpad up,down,left,right
                //checkGyroDriveControls();

            }

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
        initializeWheels();
        initializeLeftHand();
        initializeRightHand();
        //initializeColorSensor();
        initTfod();
        initializeGyro();
        telemetry.addLine("Status: Initialization of Robot Completed.");
        telemetry.update();     // update telemetry output
        sleep(2000);            // Sleep just to see telemetry initialization data
    }//end initializeRobot()

    /**
     * Initialize the Motors
     */
    public void initializeWheels() {
        // 4 Motor Drive

        // Initialize the drive motor variables.                                // Control Hub Ports
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");     // 0
        leftRearDrive  = hardwareMap.get(DcMotor.class, "leftRearDrive");       // 1
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");      // 2
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");    // 3

        // Set drive direction
        // CCW is positive by default, so change accordingly
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); //left side are REVERSE
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE); // right side are REVERSE

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
     * Initialize the left hand servo
     */
    public void initializeLeftHand() {
        leftHand = hardwareMap.get(Servo.class, "leftHand");
        isLeftHandUp = true;
        leftHand.setPosition(LEFT_HAND_MIN_POS);       // Set position to default position

        // Display the current position
        telemetry.addData("Initializing Left Hand Servo Position", "%4.2f", leftHandPosition);
    }//end initializeLeftHand()

    /**
     * Initialize the right hand CR servo
     */
    public void initializeRightHand() {
        rightHand = hardwareMap.get(CRServo.class, "rightHand");
        rotateRightHand  = false;
        rightHand.setDirection(DcMotorSimple.Direction.FORWARD);
        rightHand.setPower(0);
        telemetry.addData("Initializing CRServo RightHand Power", "%4.2f", rightHand.getPower());
    }//end initializeRightHand()

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
                .setIsModelTensorFlow2(true)
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
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
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

    /**
     *  Display the various control parameters while driving
     */
    private void sendTelemetry() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Pos LF:RF -- Target LF:RF",  "%7d:%7d -- %7d:%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), leftFrontTarget, rightFrontTarget);
        telemetry.addData("Pos LR:RR -- Target LR:RR",  "%7d:%7d -- %7d:%7d", leftRearDrive.getCurrentPosition(), rightRearDrive.getCurrentPosition(), leftRearTarget, rightRearTarget);
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
        if(gamepad1.x) {        // X is left front
            leftFrontDrive.setPower(0.5);
        }
        else if(gamepad1.y) {   // Y is right front
            rightFrontDrive.setPower(0.5);
        }
        else if(gamepad1.a) {   // A is left rear
            leftRearDrive.setPower(0.5);
        }
        else if(gamepad1.b) {   // B is right rear
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
     * Version 3 - use squared values to have better slow speed precision
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

        // Telemetry Data
        telemetry.addData("Pos LF:RF", "%7d:%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
        telemetry.addData("Pos LR:RR", "%7d:%7d", leftRearDrive.getCurrentPosition(), rightRearDrive.getCurrentPosition());
        telemetry.addData("Wheel Speeds LF:RF.", "%5.2f : %5.2f", leftFrontSpeed, rightFrontSpeed);
        telemetry.addData("Wheel Speeds LR:RR.", "%5.2f : %5.2f", leftRearSpeed, rightRearSpeed);

    }//end checkDriveControls3()

    /**********************************************
     * wrist rotate servo based on gamepad
     * X alternates between two states using global state variable isLEftHandUp
     * Does not increment, Press only ONE button
     * However, if servo gets stuck, servo may burn out trying to get to position
     **********************************************/
    public void checkLeftHandControlsOneButtonState() {
        // if y is pressed, set servoPosition to opposite saved state.
        if (gamepad1.x) {
            if(isLeftHandUp) {
                leftHandPosition = LEFT_HAND_MIN_POS;
            }
            else {
                leftHandPosition = LEFT_HAND_MAX_POS;
            }
            isLeftHandUp = !isLeftHandUp;
        }

        // set servoPosition with the new target
        leftHand.setPosition(leftHandPosition);
        sleep(CYCLE_MS);            // maybe can comment out depending on time to get to position

        // IMPORTANT NOTE: getPosition() gets last servo position set
        // NOT the current real position, if servo gets stuck, value will not be correct
        telemetry.addData("Left Hand Target Position", "%4.2f", leftHand.getPosition());

    }//end checkLeftHandControlsOneButtonState()

    /**********************************************
     * rotate right hand CRServo based on gamepad
     * B alternates between two states using global state variable
     * Does not increment, Press only ONE button to start and stop
     * However, if servo gets stuck, servo may burn out trying to get to position
     **********************************************/
    public void checkRightHandControlsOneButtonState() {
        // if b is pressed, set clawPosition to opposite saved state.
        if (gamepad1.b) {
            if(rotateRightHand) {
                rightHand.setPower(0);      // stop if rotating already
            }
            else {
                rightHand.setPower(1);      // go if not rotating
            }
            rotateRightHand = !rotateRightHand;
        }

        // IMPORTANT NOTE: getPosition() gets last servo position set
        // NOT the current real position, if servo gets stuck, value will not be correct
        telemetry.addData("Right Hand Position", "%4.2f", rightHand.getPower());

    }//end checkRightHandControlsOneButtonState()

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
    public void forward(double maxDriveSpeed, double distance, double heading) {

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
            while (opModeIsActive() && !isStopRequested() &&
                    (leftFrontDrive.isBusy() && leftRearDrive.isBusy() &&
                     rightFrontDrive.isBusy() && rightRearDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Telemetry Data
                telemetry.addData("Pos LF:RF", "%7d:%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.addData("Pos LR:RR", "%7d:%7d", leftRearDrive.getCurrentPosition(), rightRearDrive.getCurrentPosition());
                telemetry.addData("Tar LF:RF",  "%7d:%7d", leftFrontTarget, rightFrontTarget);
                telemetry.addData("Tar LR:RR",  "%7d:%7d", leftRearTarget, rightRearTarget);
                telemetry.addData("Wheel Speeds LF:RF", "%5.2f : %5.2f", leftFrontSpeed, rightFrontSpeed);
                telemetry.addData("Wheel Speeds LR:RR", "%5.2f : %5.2f", leftRearSpeed, rightRearSpeed);
                telemetry.addData("Turn Speed/Correction", "%5.2f", turnSpeed);


            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }//end forward()

    /**
     *  Method to drive in a backward in a line, on a fixed compass heading (angle), based on encoder counts.
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
    public void backward(double maxDriveSpeed, double distance, double heading) {
        forward(maxDriveSpeed, -distance, heading);
    }//end backward()

    /**
     * strafeLeft
     * strafe left specific number of inches, will be off with no odometry
     * @param   distance        inches(positive) to strafe left
     */
    public void strafeLeft(double distance) {
        //Modified and combined driveStraight code and use new moveRobotStrafe
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftFrontTarget = leftFrontDrive.getCurrentPosition() - moveCounts;
            leftRearTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            rightRearTarget = rightFrontDrive.getCurrentPosition() - moveCounts;
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

            // Get current heading
            double currentHeading = getHeading();

            // Start driving straight and then enter the control loop
            driveSpeed = DRIVE_SPEED;
            moveRobotStrafe(driveSpeed, 0);

            // keep looping while we are still active
            while (opModeIsActive() && !isStopRequested() &&
                    (leftFrontDrive.isBusy() && leftRearDrive.isBusy() &&
                     rightFrontDrive.isBusy() && rightRearDrive.isBusy())) {

                // Determine required steering to keep on current heading
                turnSpeed = getSteeringCorrection(currentHeading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobotStrafe(driveSpeed, turnSpeed);

                // Telemetry data
                telemetry.addData("Pos LF:RF", "%7d:%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.addData("Pos LR:RR", "%7d:%7d", leftRearDrive.getCurrentPosition(), rightRearDrive.getCurrentPosition());
                telemetry.addData("Tar LF:RF",  "%7d:%7d", leftFrontTarget, rightFrontTarget);
                telemetry.addData("Tar LR:RR",  "%7d:%7d", leftRearTarget, rightRearTarget);
                telemetry.addData("Wheel Speeds LF:RF.", "%5.2f : %5.2f", driveSpeed, rightFrontSpeed);
                telemetry.addData("Wheel Speeds LR:RR.", "%5.2f : %5.2f", leftRearSpeed, rightRearSpeed);
                telemetry.addData("Turn Speed/Correction", "%5.2f", turnSpeed);

            }

            // Stop all motion
            moveRobotStrafe(0, 0);

        }

    }//end strafeLeft()

    /**
     * strafeRight
     * strafe right specific number of inches - will be off without odometry
     * @param   distance        inches(positive) to strafe right
     */
    public void strafeRight(double distance) {
        //just call strafeLeft with negative distance
        strafeLeft(-distance);
    }//end strafeRight()

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
        while (opModeIsActive() && !isStopRequested() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

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
        while (opModeIsActive() && !isStopRequested() && (holdTimer.time() < holdTime)) {
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
     * Strafes Left by default with (+) drive
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



}//end WallETeleOp class
