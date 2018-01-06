package com._604robotics.robotnik.utils;

/**
 * Utility class to perform Array arithmetic.
 * Also look at java.util.Array functionality.
 */
public class ArrayMath
{
    private ArrayMath() {}
    
    /**
     * Finds the maximum element of an array.
     * @param array the array
     * @return the maximum value
     */
    public static double ArrayMax(double[] array)
    {
        double max=Double.NEGATIVE_INFINITY;
        for (double element:array)
        {
            if (element>max)
            {
                max=element;
            }
        }
        return max;
    }
    
    /**
     * Finds the minimum element of an array.
     * @param array the array
     * @return the minimum value
     */
    public static double ArrayMin(double[] array)
    {
        double max=Double.POSITIVE_INFINITY;
        for (double element:array)
        {
            if (element<max)
            {
                max=element;
            }
        }
        return max;
    }
    
    /**
     * Finds the average value of an array.
     * @param array the array
     * @return the average value
     */
    public static double ArrayAverage(double[] array) {
        double sum = 0;
        for (double element:array) {
            sum+=element;
        }
        return sum/(double)array.length;
    }
}
