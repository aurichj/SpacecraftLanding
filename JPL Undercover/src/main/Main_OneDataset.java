package main;

import objects.ImageProcessor;
import objects.PremaskProcessor;
import objects.Dataset;
import utils.Utility;
import java.util.ArrayList;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Main Executable class
 * Takes input arguments of thread count, input optical data, input lidar data, and optional solution data
 * @author Joshua Aurich
 */
public class Main_OneDataset {
    
    //the dataset which we wish to check
    static int dataset = 4;
    //how many threads to spawn
    static int processors = 8;
    
    /**
     * Main function
     * @param args 
     */
    public static void main(String[] args) {
        long start = System.currentTimeMillis();
        if(args.length == 0){
            System.out.println("Not enough input arguments.");
            System.out.println("run.jar inputOptical inputLidar");           
            System.exit(0);
        }else if(args.length == 1){
            System.out.println("Not enough input arguments.");
            System.out.println("run.jar inputOptical inputLidar");
            dataset = Integer.parseInt(args[0]);
        }
        
        //processor threads to calculate solution
        ImageProcessor[] runnable = new ImageProcessor[processors];
        //processor threads to calculate mask
        PremaskProcessor[] runnableMask = new PremaskProcessor[4];
        //the dataset object which holds the image, lidar, and solution data
        Dataset d;
        if(args.length < 2){
            d = new Dataset(dataset,null);
        }else{
            d = new Dataset(dataset,args[1]);
        }
        //executes the ImageProcessors and PremaskProcessors
        ExecutorService executor = Executors.newFixedThreadPool(processors);
        //holds the Future results from the callable processors
        ArrayList<Future<int[][]>> results = new ArrayList<Future<int[][]>>();
        //our final solution array
        int[][] estimatedSolution = new int[1000][1000];
        
        estimatedSolution = Utility.fill(estimatedSolution,255);
        
        //If a solution was provided, display the following statistics
        //This shows how accurate of a result we would have if we blindly applied all safe or all unsafe.
        float safeBlind = 0;
        float unsafeBlind = 0;
        if(d.isHaveSolution()){
            int[][] unsafeBlindSolution = new int[1000][1000];
            int[][] safeBlindSolution = new int[1000][1000];

            for (int i = 22; i < 1000 - 22; i++) {
                for (int j = 0; j < 1000 - 22; j++) {
                    safeBlindSolution[i][j] = 255;
                }
            }

            safeBlind = Utility.solutionAnalysis(safeBlindSolution, d.getSolutionData());
            unsafeBlind = Utility.solutionAnalysis(unsafeBlindSolution, d.getSolutionData());
        }

        //preprocessing phase to build the preliminary mask
        for(int i = 0; i < 4; i++){
            runnableMask[i] = new PremaskProcessor(d,i,4);
            results.add(executor.submit(runnableMask[i]));
        }
        int[][] preMaskTemp = new int[500][500];
        for(int i = 0; i <4; i++){
            int yRangeMin = i*(125);
            int yRangeMax = ((i+1)*125);
            int[][] tMask = null;
            try {
                tMask = results.get(i).get();
                results.get(i).cancel(true);
                //Utility.writeOutput(Utility.scale(tMask, false), "tMask_"+i+".jpg");
            } catch (InterruptedException ex) {
                Logger.getLogger(Main_OneDataset.class.getName()).log(Level.SEVERE, null, ex);
            } catch (ExecutionException ex) {
                Logger.getLogger(Main_OneDataset.class.getName()).log(Level.SEVERE, null, ex);
            }
            
            for(int x = 0; x < 500; x++){
                for(int y = yRangeMin; y < yRangeMax; y++){
                    preMaskTemp[x][y] = tMask[x][y];
                }
            }
        }
        d.setPreliminaryMask(preMaskTemp);
        results.clear();        
        
        //If a solution was provided, report the following statistical information
        if(d.isHaveSolution()){
            Utility.writeOutput(Utility.scale(d.getPreliminaryMask(), false), "currentMask.jpg");
            float maskMatch = Utility.solutionAnalysis(Utility.scale(d.getPreliminaryMask(), false), d.getSolutionData());        
            System.out.println("safeBlind: "+safeBlind+"%  Unsafe Blind: "+unsafeBlind+"%  Mask Match: "+maskMatch+"%");
        }
        
        //Create submasks which tell each processor what it is responsible for processing.
        int[][][] localMasks = new int[processors][d.getPreliminaryMask().length][d.getPreliminaryMask()[0].length];
        int whiteCount = 0;
        for(int i = 0; i < d.getPreliminaryMask().length; i++){
            for(int j = 0; j < d.getPreliminaryMask().length; j++){
                if(d.getPreliminaryMask()[i][j] == 255){
                    whiteCount++;
                }
            }
        }
        int localMax = whiteCount/processors;
        int localCount = 0;
        int currentProcessor = 0;
        for(int i = 0; i < d.getPreliminaryMask().length; i++){
            for(int j = 0; j < d.getPreliminaryMask().length; j++){
                if(d.getPreliminaryMask()[i][j] == 255){
                    localMasks[currentProcessor][i][j] = 255;
                    localCount++;
                    if(localCount >= localMax && currentProcessor != processors-1){
                        currentProcessor++;
                        localCount = 0;
                    }
                }
            }
        }
        
        //Spawn the individual threads to multithread the problem
        for(int i = 0; i < processors; i++){
            runnable[i] = new ImageProcessor(d,localMasks[i],i);
            results.add(executor.submit(runnable[i]));
        }

        //Collect the results
        for(int i = 0; i < processors; i++){
            int[][] localResult = null;
            try {
                localResult = results.get(i).get();
                results.get(i).cancel(true);
            } catch (InterruptedException ex) {
                Logger.getLogger(Main_OneDataset.class.getName()).log(Level.SEVERE, null, ex);
            } catch (ExecutionException ex) {
                Logger.getLogger(Main_OneDataset.class.getName()).log(Level.SEVERE, null, ex);
            }
            for(int x = 0; x < 1000; x++){
                for(int y = 0; y < 1000; y++){
                    if(d.getPreliminaryMask()[x/2][y/2] == 0){
                        estimatedSolution[x][y] = 0;
                    }else if(localMasks[i][x/2][y/2] == 255 && localResult[x][y] == 0){
                        estimatedSolution[x][y] = 0;
                    }
                }
            }
        }
        executor.shutdownNow();      
        estimatedSolution = Utility.expand(estimatedSolution);
        
        //If we are provided a solution, present the following statistics
        if(d.isHaveSolution()){
            int[][] falsePositive = new int[1000][1000];
            int[][] falseNegative = new int[1000][1000];
            float fpCount = 0, fnCount = 0;
            for (int i = 0; i < 1000; i++) {
                for (int j = 0; j < 1000; j++) {
                    if (estimatedSolution[i][j] == 255 && d.getSolutionData()[i][j] == 0) {
                        falsePositive[i][j] = 255;
                        falseNegative[i][j] = 0;
                        fnCount++;
                    } else if (estimatedSolution[i][j] == 0 && d.getSolutionData()[i][j] == 255) {
                        falsePositive[i][j] = 0;
                        falseNegative[i][j] = 255;
                        fpCount++;
                    } else {
                        falsePositive[i][j] = 255;
                        falseNegative[i][j] = 255;
                    }
                }
            }
            System.out.println("False Positive: " + (100 * fpCount / (1000 * 1000)) + "% (" + fpCount + ")  False Negative: " + (100 * fnCount / (1000 * 1000)) + "% (" + fnCount + ")");
            Utility.writeOutput(falsePositive, "falsepositive_"+dataset+".jpg");
            Utility.writeOutput(falseNegative, "falsenegative_"+dataset+".jpg");
            Utility.writeOutput(estimatedSolution, "currentSolution_"+dataset+".jpg");
            float finalResult = Utility.solutionAnalysis(estimatedSolution, d.getSolutionData());
            System.out.println("Final Result: "+finalResult+"%");
        }
        
        //If we are in a live run, output the .pgm result.
        if(args.length == 2 ){
            String fileName = args[0].replace("terrain", "solution");
            System.out.println("Writing result to: "+fileName);
            Utility.pgmOut(estimatedSolution,fileName);
        }
        
        System.out.println("Runtime: "+((System.currentTimeMillis()-start)/1000)+" seconds.");
        //Utility.getMemoryUsage();
    }       
    
}
