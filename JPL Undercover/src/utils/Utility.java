package utils;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Scanner;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.imageio.ImageIO;

/**
 * This class holds static methods which are commonly called across the program.
 *
 * @author Joshua Aurich
 */
public class Utility {

    /**
     * A quick and dirty "write to .jpg" function. Useful for visualization in
     * development.
     *
     * @param data
     * @param filePath
     */
    public static void writeOutput(int[][] data, String filePath) {
        BufferedImage output = new BufferedImage(data.length, data[0].length, BufferedImage.TYPE_3BYTE_BGR);
        Graphics2D g = output.createGraphics();
        for (int y1 = 0; y1 < data.length; y1++) {
            for (int x1 = 0; x1 < data[0].length; x1++) {
                int[] color = {data[y1][x1], data[y1][x1], data[y1][x1]};
                output.getRaster().setPixel(x1, y1, color);
            }
        }
        g.dispose();
        try {
            ImageIO.write(output, "jpg", new File(filePath));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * This function produces a .pgm output file, necessary for our final
     * solution to the competition.
     *
     * @param outputData
     * @param outputFileName
     */
    public static void pgmOut(int[][] outputData, String outputFileName) {
        PrintWriter writer = null;
        FileOutputStream out = null;

        try {
            writer = new PrintWriter(outputFileName, "UTF-8");
            writer.println("P5");
            writer.println("1000 1000");
            writer.println("255");
            writer.close();
            out = new FileOutputStream(outputFileName, true);
            for (int x = 0; x < outputData.length; x++) {
                for (int y = 0; y < outputData[0].length; y++) {
                    out.write((byte) outputData[x][y]);
                }
            }
            out.close();
        } catch (FileNotFoundException ex) {
            Logger.getLogger(Utility.class.getName()).log(Level.SEVERE, null, ex);
        } catch (UnsupportedEncodingException ex) {
            Logger.getLogger(Utility.class.getName()).log(Level.SEVERE, null, ex);
        } catch (IOException ex) {
            Logger.getLogger(Utility.class.getName()).log(Level.SEVERE, null, ex);
        } finally {
        }
    }

    /**
     * This function is used to read the input .pgm file and store it to an
     * int[][] array.
     *
     * @param filePath
     * @return
     */
    public static int[][] readPGM(String filePath) {
        FileInputStream fileInputStream = null;
        int[][] data2D = null;
        try {
            fileInputStream = new FileInputStream(filePath);
            Scanner scan = new Scanner(fileInputStream);
            // Discard the magic number
            scan.nextLine();
            // Discard the comment line
            //scan.nextLine();
            // Read pic width, height and max value
            int picWidth = scan.nextInt();
            int picHeight = scan.nextInt();
            int maxvalue = scan.nextInt();
            //fileInputStream.close();
            // Now parse the file as binary data
            fileInputStream = new FileInputStream(filePath);
            DataInputStream dis = new DataInputStream(fileInputStream);
            // look for 4 lines (i.e.: the header) and discard them
            // System.out.println("Here: "+dis.readUnsignedByte());
            int numnewlines = 3;
            while (numnewlines > 0) {
                char c;
                do {
                    c = (char) (dis.readUnsignedByte());
                } while (c != '\n');
                numnewlines--;
            }
            data2D = new int[picHeight][picWidth];
            for (int row = 0; row < picHeight; row++) {
                for (int col = 0; col < picWidth; col++) {
                    data2D[row][col] = dis.readUnsignedByte();
                    //System.out.print(data2D[row][col] + " ");
                }
                //System.out.println();
            }
        } catch (FileNotFoundException ex) {
            Logger.getLogger(Utility.class.getName()).log(Level.SEVERE, null, ex);
        } catch (IOException ex) {
            Logger.getLogger(Utility.class.getName()).log(Level.SEVERE, null, ex);
        } finally {
            try {
                fileInputStream.close();
            } catch (IOException ex) {
                Logger.getLogger(Utility.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
        return data2D;
    }

    /**
     * This function scales a 500x500 array into a 1000x1000 array for output
     * images. The augmented boolean is used when the input is already a
     * 1000x1000 array.
     *
     * @param image
     * @param augmented
     * @return
     */
    public static int[][] scale(int[][] image, boolean augmented) {
        int[][] scaled = new int[1000][1000];
        if (augmented) {
            scaled = image;
        } else {
            for (int i = 0; i < 500; i++) {
                for (int j = 0; j < 500; j++) {
                    scaled[(i * 2)][(j * 2)] = image[i][j];
                    scaled[(i * 2) + 1][(j * 2)] = image[i][j];
                    scaled[(i * 2)][(j * 2) + 1] = image[i][j];
                    scaled[(i * 2) + 1][(j * 2) + 1] = image[i][j];
                }
            }
        }
        for (int i = 0; i < 21; i++) {
            for (int j = 0; j < 1000; j++) {
                scaled[i][j] = 0;
                scaled[j][i] = 0;
                scaled[(1000) - 1 - i][j] = 0;
                scaled[j][(1000) - 1 - i] = 0;
            }
        }
        return scaled;
    }

    /**
     * This function returns the % match of dataset1 to dataset2
     *
     * @param dataset1
     * @param dataset2
     * @return
     */
    public static float solutionAnalysis(int[][] dataset1, int[][] dataset2) {
        int totalCount = 0;
        int matchCount = 0;
        for (int i = 0; i < dataset1.length; i++) {
            for (int j = 0; j < dataset1[0].length; j++) {
                totalCount++;
                if ((dataset1[i][j] > 0 && dataset2[i][j] > 0) || dataset1[i][j] == 0 && dataset2[i][j] == 0) {
                    matchCount++;
                }
            }
        }
        return (matchCount * 100.0f / totalCount);
    }

    /**
     * This function reads the lidar .dem file and stores it to float[][]
     *
     * @param path
     * @return
     */
    public static float[][] readLIDARData(String path) {
        BufferedInputStream bufferedInput = null;
        try {
            float[][] lidarData = new float[500][500];
            bufferedInput = new BufferedInputStream(new FileInputStream(path));

            for (int i = 0; i < 500 * 500; i++) {
                byte[] firstThree = new byte[4];
                bufferedInput.read(firstThree);
                float t = ByteBuffer.wrap(firstThree).order(ByteOrder.BIG_ENDIAN).getFloat();

                if (t != 0) {
                    int x = (i) % 500;
                    int y = (i) / 500;
                    lidarData[y][x] = t;
                }
            }

            return lidarData;
            //return null;
        } catch (FileNotFoundException ex) {
            Logger.getLogger(Utility.class.getName()).log(Level.SEVERE, null, ex);
        } catch (IOException ex) {
            Logger.getLogger(Utility.class.getName()).log(Level.SEVERE, null, ex);
        } finally {
            try {
                bufferedInput.close();
            } catch (IOException ex) {
                Logger.getLogger(Utility.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
        return null;
    }

    /**
     * This function fills an array with some constant value.
     *
     * @param dataset
     * @param value
     * @return
     */
    public static int[][] fill(int[][] dataset, int value) {
        for (int i = 0; i < dataset.length; i++) {
            for (int j = 0; j < dataset[0].length; j++) {
                dataset[i][j] = value;
            }
        }
        return dataset;
    }

    /**
     * Expands around the current dataset, to provide a margin of safety in our
     * solution.
     *
     * @param dataset
     * @return
     */
    public static int[][] expand(int[][] dataset) {
        int[][] newDataset = new int[dataset.length][dataset.length];
        for (int i = 11; i < dataset.length - 11; i++) {
            for (int j = 11; j < dataset.length - 11; j++) {
                if (dataset[i][j] == 0) {
                    newDataset[i][j] = 0;
                } else if (dataset[i][j] == 255) {
                    if (dataset[i + 1][j] == 0 || dataset[i - 1][j] == 0 || dataset[i][j + 1] == 0 || dataset[i][j - 1] == 0) {
                        newDataset[i][j] = 0;
                    } else {
                        newDataset[i][j] = 255;
                    }
                }
            }
        }
        return newDataset;
    }

    /**
     * Outputs the amount of bytes and megabytes used by this program in the JVM
     */
    public static void getMemoryUsage() {
        // Get the Java runtime
        Runtime runtime = Runtime.getRuntime();
        // Run the garbage collector
        runtime.gc();
        // Calculate the used memory
        long memory = runtime.totalMemory() - runtime.freeMemory();
        System.out.println("Used memory is bytes: " + memory);
        System.out.println("Used memory is megabytes: "
                + bytesToMegabytes(memory));
    }
    private static final long MEGABYTE = 1024L * 1024L;

    private static long bytesToMegabytes(long bytes) {
        return bytes / MEGABYTE;
    }
}
