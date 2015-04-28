package utils;

import Jama.Matrix;

/**
 * Interpolation function 
 * @author Vishwa Shah
 */
public class interp{
    
    /**
     * An interpolation function to improve accuracy of low resolution lidar data.
     * @param data
     * @return 
     */
    public static double[] interp(double[] data){
            // Create Vandermonde Matrix
            int l = data.length;
            double[][] A_arr = new double[l][l];
            for(int i = 0; i <l; i++){
                    for(int j = 0; j<l; j++){
                    int power_of = l-j-1;
                    A_arr[i][j] = Math.pow(i+1,power_of);
                    //System.out.print(A_arr[i][j] + " ");
                    }
                    //System.out.println();
            }
            Matrix A = new Matrix(A_arr);
            // Solve for polynomial coefficients
            Matrix b = new Matrix(data,1);
            Matrix x = A.solve(b.transpose());
            double[][] x_arr = x.getArray();

            // Evaluate Polynomial at double resolution
            double p;
            double[] sol = new double[l*2-1];
            int c = 0;
            for (double pos = 1; pos <= l; pos += 0.5){
                    p = eval(pos,x_arr);
                    sol[c] = p;
                    c++;

            }
            return sol;
    }

    public static double eval(double x, double[][] p){
            double result = 0;
            for (int i = 0; i<= p.length-1; i++){
                result = p[i][0] + (x*result);
            }
            return result;
    }
}