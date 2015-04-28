package objects;

import java.util.concurrent.Callable;
import utils.Utility;

/**
 * This is the mulithreaded implementation of the preprocessor to quickly determine locations of angle violations.
 * @author Joshua Aurich
 */
public class PremaskProcessor implements Callable<int[][]> {       
        //The child number of this thread
        int num;
        //The total number of threads
        int totalNum;
        //The dataset of optical,lidar, and solution data
        Dataset dataset;
        
        //The locations over which this thread will iterate in the y direction of the image
        private int startY;
        private int endY;
        
        /**
         * Constructor of this callable.
         * @param dataset
         * @param num
         * @param totalNum 
         */
        public PremaskProcessor(Dataset dataset, int num, int totalNum) {
            this.num = num;
            this.totalNum = totalNum;
            this.dataset = dataset;
            startY = num*125;
            endY = (num+1)*125;
            if(num == 0){
                startY = 10;
            }else if(num == 3){
                endY = endY-9;
            }
        }

        /**
         * The function which is executed to produce the solution of this thread.
         * @return 
         */
        @Override
        public int[][] call() {
            return robustAnglePreprocessor(dataset.getLidarData(),10,0.2f); 
        }
        
        /**
         * This function iterates over every pixel in the domain to check spacecraft sized neighborhoods in the lidar
         * data in order to determine obvious potential orientations which will violate the angle constraint of 10 degrees.
         * @param lidarData
         * @param startLoc
         * @param step
         * @return 
         */
        public int[][] robustAnglePreprocessor(float[][] lidarData, int startLoc, float step) {
            int[][] mask = new int[lidarData.length][lidarData[0].length];
            mask = Utility.fill(mask,255);
            
            for (int i = startLoc; i < mask.length - startLoc; i++) {
                for (int j = startY; j < endY; j++) {
                    if (mask[i][j] == 255) {
                        boolean searching = true;
                        for (int k = -startLoc; k < startLoc && searching; k++) {
                            for (int h = -startLoc; h < startLoc; h++) {
                                float distance = (float) Math.sqrt((k * k) + (h * h));
                                if (Math.abs((distance * step) - 1.7) <= 0.25f) {
                                    float y1 = lidarData[i + h][j - k];
                                    float y2 = lidarData[i - h][j + k];
                                    float x1 = lidarData[i - h][j + k];
                                    float x2 = lidarData[i + h][j - k];
                                    float xDiff = Math.abs(x2 - x1);
                                    float yDiff = Math.abs(y2 - y1);
                                    float tan10 = (float) Math.tan(10.0f * Math.PI / 180.0f);
                                    if (3.4f * tan10 <= xDiff || 3.4f * tan10 <= yDiff) {
                                        mask[i][j] = 0;
                                        searching = false;
                                    }
                                }
                            }
                        }
                    }
                }
            }  
            //Utility.getMemoryUsage();
            return mask;
        }
    }
