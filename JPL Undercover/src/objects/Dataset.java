package objects;

import utils.Utility;

/**
 * Class responsible for storing all the information about our image.
 * @author Joshua Aurich
 */
public class Dataset {
    //The optical dataset (not currently utilized in our algorithm)
    private int[][] imageData;
    //The lidar dataset
    private float[][] lidarData;
    //If a solution was provided, it will be stored here
    private int[][] solutionData;
    //The preliminary mask which is used to ignore many pixels already deemed unsafe and increase runtimes
    private int[][] preliminaryMask;
    //Boolean to check if we were provided a solution
    private boolean haveSolution = false;
    
    /**
     * Class constructor
     * @param dataset
     * @param optical
     * @param lidar
     * @param solution 
     */
    public Dataset(int dataset, String lidar){
        //optical == null means that we are running a predefined dataset and dont need to load anything new.
        if(lidar == null){
            switch (dataset) {
                case (1):
                    imageData = Utility.readPGM("terrainS0C0R10_100.pgm");
                    solutionData = Utility.readPGM("terrainS0C0R10_100.invHazard.pgm");
                    lidarData = Utility.readLIDARData("terrainS0C0R10_100_500by500_dem.raw");
                    break;//easy dataset1
                case (2):
                    imageData = Utility.readPGM("terrainS4C0R10_100.pgm");
                    solutionData = Utility.readPGM("terrainS4C0R10_100.invHazard.pgm");
                    lidarData = Utility.readLIDARData("terrainS4C0R10_100_500by500_dem.raw");
                    break; //medium dataset2
                case (3):
                    imageData = Utility.readPGM("terrainS4C4R10_100.pgm");
                    solutionData = Utility.readPGM("terrainS4C4R10_100.invHazard.pgm");
                    lidarData = Utility.readLIDARData("terrainS4C4R10_100-500by500_dem.raw");
                    break; //medium dataset3
                case (4):
                    imageData = Utility.readPGM("terrainS4C4R20_100.pgm");
                    solutionData = Utility.readPGM("terrainS4C4R20_100.invHazard.pgm");
                    lidarData = Utility.readLIDARData("terrainS4C4R20_100-500by500_dem.raw");
                    break; //hard dataset4
            }
            if(solutionData != null){
                haveSolution = true;
            }
        }else{                    
            lidarData = Utility.readLIDARData(lidar);
        }
    }

    public int[][] getImageData() {
        return imageData;
    }

    public void setImageData(int[][] imageData) {
        this.imageData = imageData;
    }

    public float[][] getLidarData() {
        return lidarData;
    }

    public void setLidarData(float[][] lidarData) {
        this.lidarData = lidarData;
    }

    public int[][] getSolutionData() {
        return solutionData;
    }

    public void setSolutionData(int[][] solutionData) {
        this.solutionData = solutionData;
    }

    public boolean isHaveSolution() {
        return haveSolution;
    }

    public void setHaveSolution(boolean haveSolution) {
        this.haveSolution = haveSolution;
    }

    public int[][] getPreliminaryMask() {
        return preliminaryMask;
    }

    public void setPreliminaryMask(int[][] preliminaryMask) {
        this.preliminaryMask = preliminaryMask;
    }
    
    
}
