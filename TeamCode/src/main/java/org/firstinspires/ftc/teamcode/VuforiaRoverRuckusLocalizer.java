package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * THIS IS NOT AN OPMODE
 * This is a class for Vuforia that gets the robots locaiton.
 * @author Lorenzo Pedroza
 * Version history
 * ======= ======
 * v 0.1  11/24/18 @author Lorenzo Pedroza. Created file. Has license key. //TODO test this.
 */


public class VuforiaRoverRuckusLocalizer implements FTCModularizableSystems{

    private static final String VUFORIA_KEY = "AZj3x0D/////AAABmUJIr5DbgE3Km1qA/yma4sZwYhAr5k8hMCDClJgACoA4rsUX/eplcfQcyMMY5j5UH0ASZY5vNNjUTl1IAUA/YmS87xIyvXMGsLH9M2GNrhofohDKXpqCUpCiYnxCMfQ/HDqyTonde4wAupmH+rHFPO85DOYRBzZkcPrXb+4cTeSeG2SJXbOm3YJ4DaEQZOPOP2r2FWC3w6shySp5RsqH5M8RSLxdA2RzriAbi+dQEKQf6LpNkZN2sh2W5Bpbbpq6PXvqC93tZ7sO6Iho/+6H92dwKorWRnsswHINimiPgV7TcN8eqmeSXLhRNuWkFWoc4MzMDKtWGwBcXphphdFFEn+dhR1n/7/aFDnAQQrr/kdF";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    //private final VuforiaLocalizer.CameraDirection CAMERA_CHOICE;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private VuforiaLocalizer vuforia;
    private int cameraMonitorViewId;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables targetsRoverRuckus;
    private VuforiaTrackable blueRover; //image resources
    private VuforiaTrackable redFootprint;
    private VuforiaTrackable frontCraters;
    private VuforiaTrackable backSpace;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private OpenGLMatrix blueRoverLocationOnField;
    private OpenGLMatrix redFootprintLocationOnField;
    private OpenGLMatrix frontCratersLocationOnField;
    private OpenGLMatrix backSpaceLocationOnField;

    private OpenGLMatrix phoneLocationOnRobot;

    private final int CAMERA_FORWARD_DISPLACEMENT;
    private final int CAMERA_VERTICAL_DISPLACEMENT;
    private final int CAMERA_LEFT_DISPLACEMENT;

    OpenGLMatrix robotLocationTransform = null;

    VectorF translation = null;

    Orientation rotation = null;


    VuforiaRoverRuckusLocalizer(final VuforiaLocalizer.CameraDirection CAMERA_CHOICE, final int CAMERA_FORWARD_DISPLCACEMENTInmm,
                                final int CAMERA_VERTICAL_DISPLACEMENTinmm, final int CAMERA_LEFT_DISPLACEMENTinmm){
        parameters.cameraDirection = CAMERA_CHOICE;
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        this.CAMERA_FORWARD_DISPLACEMENT = CAMERA_FORWARD_DISPLCACEMENTInmm;
        this.CAMERA_VERTICAL_DISPLACEMENT = CAMERA_VERTICAL_DISPLACEMENTinmm;
        this.CAMERA_LEFT_DISPLACEMENT = CAMERA_LEFT_DISPLACEMENTinmm;
    }

    public void initHardware(HardwareMap ahwMap) {
        cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
        blueRover.setName("Blue-Rover");
        redFootprint.setName("Red-Footprint");
        frontCraters.setName("Front-Craters");
        backSpace.setName("Back-Space");

        allTrackables.addAll(targetsRoverRuckus);

        blueRoverLocationOnField = OpenGLMatrix.translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ,
                        DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        parameters.cameraDirection == VuforiaLocalizer.CameraDirection.FRONT ? 90 : -90,
                        0, 0));

        for (VuforiaTrackable trackable : allTrackables){
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        targetsRoverRuckus.activate();

    }

    public String currentVisibleRoverRuckusVuforiaTrackable(){
        String currentVisibleTraclableName = "none visible";
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                currentVisibleTraclableName = trackable.getName();
                break;
            }
        }
        return currentVisibleTraclableName;
    }

    public float getRobotX(){
        currentVisibleRoverRuckusVuforiaTrackable(); //no need to collect output. Just to run it to get visible vumark
        if (targetVisible){
            translation = lastLocation.getTranslation();
            return translation.get(0)/mmPerInch;
        }
        return -1;
    }

    public float getRobotY(){
        currentVisibleRoverRuckusVuforiaTrackable();
        if (targetVisible){
            translation = lastLocation.getTranslation();
            return translation.get(1)/mmPerInch;
        }
        return -1;
    }

    public float getRobotZ(){
        currentVisibleRoverRuckusVuforiaTrackable();
        if (targetVisible){
            translation = lastLocation.getTranslation();
            return translation.get(2)/mmPerInch;
        }
        return -1;
    }

    public float getRobotRoll(){
        currentVisibleRoverRuckusVuforiaTrackable();
        if (targetVisible){
            rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            return rotation.firstAngle;
        }
        return -1;
    }

    public float getRobotPitch(){
        currentVisibleRoverRuckusVuforiaTrackable();
        if (targetVisible){
            rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            return rotation.secondAngle;
        }
        return -1;
    }

    public float getRobotHeading(){
        currentVisibleRoverRuckusVuforiaTrackable();
        if (targetVisible){
            rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            return rotation.thirdAngle;
        }
        return -1;
    }
}
