package org.deceivers.drivers;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightDriver {
    // Limelight Network Table
    private NetworkTable limelightTable;

    //Network Table Entry Names
    private static final String LIMELIGHT_TABLE = "limelight";
    //inputs
    private static final String TARGET_VALID = "tv";
    private static final String TARGET_X_ANGLE = "tx";
    private static final String TARGET_Y_ANGLE = "ty";
    private static final String TARGET_AREA = "ta";
    private static final String TARGET_SKEW = "ts";
    private static final String PIPELINE_LATENCY = "tl";
    private static final String TARGET_SHORT_SIDE = "tshort";
    private static final String TARGET_LONG_SIDE = "tlong";
    private static final String TARGET_HORIZONTAL = "thor";
    private static final String TARGET_VERTICAL = "tvert";
    private static final String GET_PIPELINE = "getpipe";
    private static final String CAMERA_TRANSLATION = "camtran";
    private static final String CROSSHAIR_COLOR = "tc";

    //outputs
    private static final String LED_MODE = "ledMode";
    private static final String CAM_MODE = "camMode";
    private static final String SET_PIPELINE = "pipeline";
    private static final String STREAM_MODE = "stream";
    private static final String SNAPSHOT = "snapshot";
    private static final String CROP = "crop";

    //Cross hair data
    private static final String CROSS_HAIR_X = "cx";
    private static final String CROSS_HAIR_Y = "cy";

    //corners
    private static final String CONTOUR_CORNERS = "tcornxy";

    //python
    private static final String PYTHON_DATA = "llpython";
    private static final String ROBOT_DATA = "llrobot";

    //Enums for setting modes
    public enum CamMode {
        VisionMode(0),
        DriverMode(1);

        private final int value;
        private static Map<Integer, CamMode> map = new HashMap<>();

        private CamMode(int value){
            this.value = value;
        }

        static {
            for (CamMode camMode : CamMode.values()){
                map.put(camMode.value, camMode);
            }
        }

        public static CamMode valueOf(int cameMode){
            return (CamMode) map.get(cameMode);
        }
      }
    
      public enum LedMode {
        Off(1),
        Blink(2),
        On(3),
        Auto(0);
        
        private final int value;
        private static Map<Integer, LedMode> map = new HashMap<>();

        private LedMode(int value){
            this.value = value;
        }

        static {
            for (LedMode ledMode : LedMode.values()){
                map.put(ledMode.value, ledMode);
            }
        }

        public static LedMode valueOf(int ledMode){
            return (LedMode) map.get(ledMode);
        }
      }

      public enum StreamMode{
          Standard(0),
          PiPMain(1),
          PiPSecondary(2);

          private final int value;
          private static Map<Integer, StreamMode> map = new HashMap<>();

          private StreamMode(int value){
              this.value = value;
          }

          static{
              for (StreamMode streamMode : StreamMode.values()) {
                  map.put(streamMode.value, streamMode);
              }
          }

          public static StreamMode valueOf(int streamMode){
              return (StreamMode) map.get(streamMode);
          }
      }

      public LimelightDriver(){
        limelightTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE);
    }

    //Are there any valid targets
    public boolean hasTarget(){
        return limelightTable.getEntry(TARGET_VALID).getBoolean(false);
    }

    //Horizontal offset from crosshair to target
    public double getXAngle(){
        return limelightTable.getEntry(TARGET_X_ANGLE).getDouble(0.0);
    }

    //Vertical offset from crosshair to target
    public double getYAngle(){
        return limelightTable.getEntry(TARGET_Y_ANGLE).getDouble(0.0);
    }

    //Percentage of image filled with target
    public double getTargetArea(){
        return limelightTable.getEntry(TARGET_AREA).getDouble(0.0);
    }

    //Skew/Rotation of the the target
    public double getTargetSkew(){
        return limelightTable.getEntry(TARGET_SKEW).getDouble(0.0);
    }

    //Pipelines latency contribution, add 11ms for image capture latency
    public double getPipelineLatency(){
        return limelightTable.getEntry(PIPELINE_LATENCY).getDouble(0.0);
    }

    //Length in pixels of the short side of the bounding box
    public double getSideLengthShort(){
        return limelightTable.getEntry(TARGET_SHORT_SIDE).getDouble(0.0);
    }

    //Length in pixels of the long side of the bounding box
    public double getSideLengthLong(){
        return limelightTable.getEntry(TARGET_LONG_SIDE).getDouble(0.0);
    }

    //Horizontal side length in pixels of the bounding box
    public double getHorizontalSideLength(){
        return limelightTable.getEntry(TARGET_HORIZONTAL).getDouble(0.0);
    }

    //Verticle side length in pixels of the bounding box
    public double getVerticleSideLength(){
        return limelightTable.getEntry(TARGET_VERTICAL).getDouble(0.0);
    }

    //Get the currently selected pipeline between 0 and 9
    public double getActivePipeline(){
        return limelightTable.getEntry(GET_PIPELINE).getDouble(0.0);
    }

    //get pose of robot relative to target if 3D position solution is active
    public Number[] getCameraPose(){
        return limelightTable.getEntry(CAMERA_TRANSLATION).getNumberArray(null);
    }

    //get the HSV color under the crosshair region
    public Number[] getColorAtCrosshair(){
        return limelightTable.getEntry(CROSSHAIR_COLOR).getNumberArray(null);
    }

    //get the current setting of the LED
    public LedMode getLedMode(){
        return LedMode.valueOf(limelightTable.getEntry(LED_MODE).getNumber(0).intValue());
    }

    //set LED mode
    public void setLedMode(LedMode mode){
        limelightTable.getEntry(LED_MODE).setNumber(mode.value);
    }

    //get the current setting of the camera
    public CamMode getCamMode(){
        return CamMode.valueOf(limelightTable.getEntry(CAM_MODE).getNumber(0).intValue());
    }

    //Set the camera mode
    public void setCamMode(CamMode mode){
        limelightTable.getEntry(CAM_MODE).setNumber(mode.value);
    }

    //set the vision pipeline to use
    public void setPipeline(int pipeline){
        limelightTable.getEntry(SET_PIPELINE).setNumber(pipeline);
    }

    //get the current stream mode
    public StreamMode getStreamMode(){
        return StreamMode.valueOf(limelightTable.getEntry(STREAM_MODE).getNumber(0).intValue());
    }

    //take a snapshot of the camera image
    public void takeSnapshot(){
        limelightTable.getEntry(SNAPSHOT).setNumber(1);
    }

    //set the crop rectangle, must use the crop setting in the web interface
    public void setCropRectangle(double x0, double x1, double y0, double y1){
        Number[] rectangle = {x0, x1, y0, y1};
        limelightTable.getEntry(CROP).setNumberArray(rectangle);
    }

    //get data from limelight pythin script
    public Number[] getPythonData(){
        return limelightTable.getEntry(PYTHON_DATA).getNumberArray(null);
    }

    //Set the robot data to be used by limelight python script
    public void setRobotData(Number[] data){
        limelightTable.getEntry(ROBOT_DATA).setNumberArray(data);
    }

    //get contour corners
    //TODO: make corners object
    public Number[] getContourCorners(){
        return limelightTable.getEntry(CONTOUR_CORNERS).getNumberArray(null);
    }

    //TODO: make crosshair object
    //get y location of crosshair in normalized space
    public double getCrosshairY(int crosshairNumber){
        return limelightTable.getEntry(CROSS_HAIR_Y + crosshairNumber).getDouble(0.0);
    }

    //get x location of crosshair in normalized space
    public double getCrosshairX(int crosshairNumber){
        return limelightTable.getEntry(CROSS_HAIR_X + crosshairNumber).getDouble(0.0);
    }

    //TOD: add raw contours
}
