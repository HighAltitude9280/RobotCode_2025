package frc.robot.resources.math;

import java.util.concurrent.ThreadLocalRandom;

import edu.wpi.first.wpilibj.DriverStation;

public class Math {

    public static final double PI = 3.141592653589793238462643383279502884197;

    /**
     * @param p  input to clamp
     * @param mn minimum value of the output
     * @param mx maximum value of the output
     * @return if the input is within mn and mx, returns p. If the input is out of
     *         bounds, returns either mn or mx.
     */
    public static double clamp(double p, double mn, double mx) {
        return java.lang.Math.max(mn, java.lang.Math.min(p, mx));
    }

    /**
     * @return returns positive a mod b
     */
    public static int module(int a, int b) {
        int mod = a % b;
        if (mod < 0) {
            mod += b;
        }
        return mod;
    }

    public static double max(double a, double b) {
        return java.lang.Math.max(a, b);
    }

    public static double min(double a, double b) {
        return java.lang.Math.min(a, b);
    }

    public static double abs(double a) {
        return java.lang.Math.abs(a);
    }

    public static double sin(double a) {
        return java.lang.Math.sin(a);
    }

    public static double cos(double a) {
        return java.lang.Math.cos(a);
    }

    public static double tan(double a) {
        return java.lang.Math.tan(a);
    }

    public static double atan(double a) {
        return java.lang.Math.atan(a);
    }

    public static double hypot(double a, double b) {
        return java.lang.Math.hypot(a, b);
    }

    public static double toRadians(double a) {
        return java.lang.Math.toRadians(a);
    }

    public static double toDegrees(double a) {
        return java.lang.Math.toDegrees(a);
    }

    public static double sqrt(double a) {
        return java.lang.Math.sqrt(a);
    }

    public static double pow(double a, double b) {
        return java.lang.Math.pow(a, b);
    }

    /**
     * @param r Red value in a range from 0-1
     * @param g Green value in a range from 0-1
     * @param b Blue value in a range from 0-1
     * @return Array with three values: Hue, Saturation, and Value
     */

    public static double[] RGBtoHSV(double r, double g, double b) {
        double[] hsv = { 0f, 0f, 0f };
        double minRGB = Math.min(r, Math.min(g, b));
        double maxRGB = Math.max(r, Math.max(g, b));

        double d = (r == minRGB) ? g - b : ((b == minRGB) ? r - g : b - r);
        double h = (r == minRGB) ? 3 : ((b == minRGB) ? 1 : 5);
        hsv[0] = 60 * (h - d / (maxRGB - minRGB));
        hsv[1] = (maxRGB - minRGB) / maxRGB;
        hsv[2] = maxRGB;
        return hsv;
    }

    public static double applyDeadzone(double input, double deadzone) {
        return Math.abs(input) > deadzone ? input : 0;
    }

    public static double[] convertIntArrayToDoubleArray(int[] source) {
        double[] destinationArray = new double[source.length];
        for (int i = 0; i < source.length; i++) {
            destinationArray[i] = source[i];
        }
        return destinationArray;
    }

    /**
     * Returns the angle with the opposite direction of the given angle.
     * 
     * @param angle The angle in degrees (from -180 to 180) to generate its opposite
     *              angle.
     * @return The opposite angle of the given angle in degrees, from -180 to 180
     */
    public static double getOppositeAngle(double angle) {

        double result = angle - 180;

        return normalizeAngle(result);

    }

    /**
     * Returns an angle which is coterminal with the given one, but in a range of
     * -180 to 180.
     * 
     * @param angle The angle (in degrees) to normalize.
     * @return An angle(in degrees), coterminal with the given one, but in a range
     *         of -180 to 180.
     */
    public static double normalizeAngle(double angle) {

        double result = angle;

        if (Math.abs(result) <= 180) {
            return result;
        }

        if (result > 180) {
            result -= 360;
        }
        if (result < -180) {
            result += 360;
        }

        return normalizeAngle(result);

    }

    /**
     * The smallest possible angle between the current angle and the target.
     * 
     * @param angle  The current angle (From -180 to 180).
     * @param target The desired angle (From -180 to 180).
     * @return The smallest angle between the angle and the target, from -180 to
     *         180.
     */
    public static double deltaAngle(double angle, double target) {

        double delta = target - angle;

        return normalizeAngle(delta);

    }

    public static double distance(double x1, double x2, double y1, double y2) {

        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

    }

    public static int randomInt(int min, int max) {
        return ThreadLocalRandom.current().nextInt(min, max + 1);
    }

    public static double atan2(double y, double x) {
        return java.lang.Math.atan2(y, x);
    }

    /**
     * Deletes a certain row and column of the given matrix.
     * 
     * @param i      The column to delete.
     * @param j      The row to delete.
     * @param matrix The matrix from which the row and column will be deleted.
     * @return A new matrix, without the indicated row and column.
     */
    public static double[][] matrixDeleteRowColumn(int i, int j, double[][] matrix) {
        double[][] newMatrix = new double[matrix.length - 1][matrix[0].length - 1];

        for (int a = 0; a < matrix.length - 1; a++) {
            for (int b = 0; b < matrix.length - 1; b++) {

                int thisLine = a + (i <= a ? 1 : 0);
                int thisColumn = b + (j <= b ? 1 : 0);

                newMatrix[a][b] = matrix[thisLine][thisColumn];

            }
        }
        return (newMatrix);
    }

    /**
     * Transposes a matrix.
     * 
     * @param matrix The matrix to transpose.
     * @return The transpose of the given matrix.
     */
    public static double[][] transposeMatrix(double[][] matrix) {
        double[][] newMatrix = new double[matrix[0].length][matrix.length];

        for (int i = 0; i < matrix.length; i++) {
            for (int j = 0; j < matrix[0].length; j++) {

                newMatrix[j][i] = matrix[i][j];

            }
        }

        return newMatrix;
    }

    /**
     * Multiplies two matrices.
     * 
     * @param a The first matrix to multiply.
     * @param b The second matrix to multiply
     * @return aXb.
     */
    public static double[][] multiplyMatrices(double[][] a, double[][] b) {
        if (a[0].length != b.length) {
            DriverStation.reportError(
                    "Could not perform matrix multiplication, the number of columns in the first matrix should be equal to the number of rows in the second.",
                    true);
            return null;
        }

        double[][] result = new double[a.length][b[0].length];

        for (int i = 0; i < result.length; i++) {
            for (int j = 0; j < result[0].length; j++) {
                double total = 0;

                for (int current = 0; current < a[0].length; current++) {
                    total += a[i][current] * b[current][j];
                }

                result[i][j] = total;
            }
        }

        return result;
    }

    /**
     * Calculates the determinant of the given matrix.
     * 
     * @param matrix The matrix to calculate the determinant.
     * @return The determinant of the given matrix.
     */
  

    /**
     * Inverses a matrix.
     * 
     * @param matrix The matrix to inverse.
     * @return The inverse of the matrix.
     */
       /**
     * Converts a slope to an angle. Useful for spline movement. Note that a
     * vertical angle to slope
     * means that the axis will be swaped, resulting in a, maybe unexpected,
     * mirrored beheaviour.
     * 
     * Vertical:
     * 
     * 0°
     * x
     * |
     * ------------ y -90°
     * |
     * 
     * 
     * Normal:
     * 
     * -90°
     * y
     * |
     * ------------- x 0°
     * |
     * 
     * @param slope    The slope to convert.
     * @param vertical True to swap angles.
     *
     * @return The angle (from -180 to 180).
     */
    public static double angleFromSlope(double slope, boolean vertical) {

        double result = 0;

        if (!vertical)
            result = -toDegrees(atan(slope));

        else if (slope < 0) {
            slope = abs(slope);

            result = -toDegrees(atan(slope)) - 90;
        } else if (slope == 0)
            result = -90;

        else
            result = -toDegrees(atan(1 / slope));

        return (normalizeAngle(result));
    }

}