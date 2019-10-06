package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Victo on 9/17/2018.
 *
 * This is NOT an opmode. It is a software service for detecting Gold Minerals. It can give a Pos, or goldXPos
 * that can be used to get a robot to align itself with a gold mineral and push it off. Thus this code is being
 * used in the Rover Ruckus Season
 *
 * Version History
 * ======= =======
 * v 0.1   09/17/18 Created by Victo
 * v 0.2   11/02/18 added this Javadoc
 */

public class GoldMineralDetector_2 extends DogeCVDetector implements FTCModularizableSystems, Runnable{

    //Position enum

    private Pos pos = Pos.MID;

    //TelemetryPositionText
    private String TelemetryPosText = "??";

    //Thresholds to correlate screen points to position enums
    private final static int MIDPOINT = 320;  // screen midpoint
    private final static int LEFTPOINT = -106;
    private final static int RIGHTPOINT = 106;

    // Defining Mats to be used.
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)
    private Mat workingMat = new Mat(); // Used for preprocessing and working with (blurring as an example)
    private Mat maskYellow = new Mat(); // Yellow Mask returned by color filter
    private Mat hierarchy  = new Mat(); // hierarchy used by coutnours

    // Results of the detector
    private boolean found    = false; // Is the gold mineral found
    private boolean aligned  = false; // Is the gold mineral aligned
    private double  goldXPos = 0;     // X Position (in pixels) of the gold element
    private double  goldYPos = 0;     // Y Position (in pixels) of the gold element
    private double  goldArea = 0;     // Area
    private double  goldRatio = 0;         // width / high ratio

    private double  goldScore = 0;         // width / high ratio

    // Detector settings
    public boolean debugAlignment = true; // Show debug lines to show alignment settings
    public double alignPosOffset  = 0;    // How far from center frame is aligned
    public double alignSize       = 640;  // Used to be 100. How wide is the margin of error for alignment

    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Setting to decide to use MaxAreaScorer or PerfectAreaScorer
    // Can also be PERFECT_AREA
    //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring


    //Create the default filters and scorers
    public DogeCVColorFilter yellowFilter      = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW); //Default Yellow filter

    public RatioScorer       ratioScorer       = new RatioScorer(1.25, 50000);          // Used to find perfect squares
    public MaxAreaScorer     maxAreaScorer     = new MaxAreaScorer( 0.001);                    // Used to find largest objects
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(5000,0.05); // Used to find objects near a tuned area value

    /**
     * Simple constructor
     */
    public GoldMineralDetector_2() {
        super();
        detectorName = "Gold Align Detector"; // Set the detector name
        downscale = 0.4; //edit downscale of DogeCVDetector

    }

    public void initHardware(HardwareMap ahwMap){
        super.init(ahwMap.appContext, CameraViewDisplay.getInstance());
        useDefaults();
    }

    public void tune(){ //change values as needed.

    }

    @Override
    public void run() {

    }

    @Override
    public Mat process(Mat input) {

        // Copy the input mat to our working mats, then release it for memory
        input.copyTo(displayMat);
        input.copyTo(workingMat);
        input.release();


        //Preprocess the working Mat (blur it then apply a yellow filter)
        Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        yellowFilter.process(workingMat.clone(),maskYellow);

        //Find contours of the yellow mask and draw them to the display mat for viewing

        List<MatOfPoint> contoursYellow = new ArrayList<>();
        Imgproc.findContours(maskYellow, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursYellow,-1,new Scalar(230,70,70),2);

        // Current result
        Rect bestRect = null;
        double bestDiffrence = Double.MAX_VALUE; // MAX_VALUE since less diffrence = better

        // Loop through the contours and score them, searching for the best result
        for(MatOfPoint cont : contoursYellow){
            double score = calculateScore(cont); // Get the diffrence score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0,0,255),2); // Draw rect

            // If the result is better then the previously tracked one, set this rect as the new best
            if(score < bestDiffrence){
                bestDiffrence = score;
                bestRect = rect;
                goldScore = score;
            }
        }

        // Vars to calculate the alignment logic.
        double alignX    = (getAdjustedSize().width / 2) + alignPosOffset; // Center point in X Pixels
        double alignXMin = alignX - (alignSize / 2); // Min X Pos in pixels
        double alignXMax = alignX +(alignSize / 2); // Max X pos in pixels
        double xPos; // Current Gold X Pos
        double yPos; // Current Godl Y Pos

        if(bestRect != null){
            // Show chosen result
            Imgproc.rectangle(displayMat, bestRect.tl(), bestRect.br(), new Scalar(255,0,0),4);
            Imgproc.putText(displayMat, "Chosen", bestRect.tl(),0,1,new Scalar(255,255,255));

            // Set align X pos
            xPos = bestRect.x + (bestRect.width / 2);
            goldXPos = xPos;

            // Set align Y pos
            yPos = bestRect.y + (bestRect.height / 2);
            goldYPos = yPos;

            // Set Area
            goldArea = bestRect.area();

            // Set width / high ratio

            goldRatio = (double)bestRect.width / (double)bestRect.height;

            // Draw center point
            Imgproc.circle(displayMat, new Point( xPos, bestRect.y + (bestRect.height / 2)), 5, new Scalar(0,255,0),2);

            // Check if the mineral is aligned
            if(xPos < alignXMax && xPos > alignXMin){
                aligned = true;
            }else{
                aligned = false;
            }

            // Draw Current X
            Imgproc.putText(displayMat,"Current X: " + bestRect.x,new Point(10,getAdjustedSize().height - 10),0,0.5, new Scalar(255,255,255),1);
            found = true;
        }else{
            found = false;
            aligned = false;
        }
        if(debugAlignment){

            //Draw debug alignment info
            if(isFound()){
                Imgproc.line(displayMat,new Point(goldXPos, getAdjustedSize().height), new Point(goldXPos, getAdjustedSize().height - 30),new Scalar(255,255,0), 2);
            }

            Imgproc.line(displayMat,new Point(alignXMin, getAdjustedSize().height), new Point(alignXMin, getAdjustedSize().height - 40),new Scalar(0,255,0), 2);
            Imgproc.line(displayMat,new Point(alignXMax, getAdjustedSize().height), new Point(alignXMax,getAdjustedSize().height - 40),new Scalar(0,255,0), 2);
        }

        //Print result
        Imgproc.putText(displayMat,"Result: " + aligned,new Point(10,getAdjustedSize().height - 30),0,1, new Scalar(255,255,0),1);


        return displayMat;

    }

    @Override
    public void useDefaults() {
        addScorer(ratioScorer);

        // Add diffrent scoreres depending on the selected mode
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }

    }

    /**
     * Set the alignment settings for GoldAlign
     * @param offset - How far from center frame (in pixels)
     * @param width - How wide the margin is (in pixels, on each side of offset)
     */
    public void setAlignSettings(int offset, int width){
        alignPosOffset = offset;
        alignSize = width;
    }

    /**
     * Returns if the gold element is aligned
     * @return if the gold element is alined
     */
    public boolean getAligned(){
        return aligned;
    }

    /**
     * Returns gold element last x-position
     * @return last x-position in screen pixels of gold element
     */
    public double getXPosition(){
        return goldXPos-MIDPOINT;
    } //account for midpoint

    public double getYPosition(){
        return goldYPos;
    } //account for midpoint

    public double getArea(){
        return goldArea;
    }

    public double getRatio(){
        return goldRatio;
    }

    public double getScore(){
        return Math.abs(goldScore);
    }
    /**
     * Returns if a gold mineral is being tracked/detected
     * @return if a gold mineral is being tracked/detected
     */
    public boolean isFound() {
        return found;
    }

    public void updateGoldPos(Pos newPos){
        pos = newPos;

        /*
        double x = getXPosition();
        if(isFound()){
            if (x < LEFTPOINT) {
                pos = Pos.LEFT;
                TelemetryPosText = "LEFT";
            }

            else if ((x >= LEFTPOINT) && (x <= RIGHTPOINT)) {
                pos = Pos.MID;
                TelemetryPosText = "Mid";
            }

            else if (x >= RIGHTPOINT) {
                pos = Pos.RIGHT;
                TelemetryPosText = "Right";
            }

        }
        else{ //mineral not in FOV
            pos = Pos.UNKNOWN;
            TelemetryPosText = "Unknown";
        }
        */

    }

    public Pos getGoldPos(){
        updateGoldPos(pos);
        return pos;
    }

    public String getGoldPosForTelemetry(){
        updateGoldPos(pos);
        return TelemetryPosText;
    }
}
