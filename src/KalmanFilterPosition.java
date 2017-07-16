import org.apache.commons.math3.filter.*;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

/**
 * @author Marinus Toman Date: 28-Jun-2017
 */
public class KalmanFilterPosition {

    // Position measurement noise (in meters)
    private final double MEAS_NOISE;
    // Process noise (in meters)
    private final double PROC_NOISE = 1d;
    // Discrete time interval between steps
    private final double dt;

    // A - state transition matrix
    private RealMatrix A;
    // B - control input matrix
    private RealMatrix B;
    // H - measurement matrix
    private RealMatrix H;
    // Q - process noise covariance matrix (process error)
    private RealMatrix Q;
    // R - measurement noise covariance matrix (measurement error)
    private RealMatrix R;
    // P - error covariance matrix
    private RealMatrix P;
    // x - state
    private RealVector x;

    // Kalman Filter
    private KalmanFilter filter;

    /**
     * Constructs a KalmanFilter that takes initial system state
     * @param measNoise Measurement covariance
     * @param time Time interval
     * @param X Initial X coordinate
     * @param Y Initial Y coordinate
     */
    public KalmanFilterPosition(double measNoise, double time, double X, double Y) {
        // Set measurement and process error constants
        this.MEAS_NOISE = measNoise;
        // Set discrete time steps
        this.dt = time;
        // A = 
        A = new Array2DRowRealMatrix(new double[][]{
            {1d, 0d, dt, 0d},
            {0d, 1d, 0d, dt},
            {0d, 0d, 1d, 0d},
            {0d, 0d, 0d, 1d}
        });
        // B = 
        B = new Array2DRowRealMatrix(new double[][]{
            {Math.pow(dt, 2d) / 2d},
            {Math.pow(dt, 2d) / 2d},
            {dt},
            {dt}
        });
        //only observe first 2 values - the position coordinates
        H = new Array2DRowRealMatrix(new double[][]{
            {1d, 0d, 0d, 0d},
            {0d, 1d, 0d, 0d},
        });
        // System state with initial state included
        x = new ArrayRealVector(new double[] {X, Y, 0, 0});
        // Measurement noise covariance matrix
        R = new Array2DRowRealMatrix(new double[][] {
            { Math.pow(this.MEAS_NOISE, 2d), 0d },
            { 0d, Math.pow(this.MEAS_NOISE, 2d) }
        });
        // Process noise covariance matrix
        Q = new Array2DRowRealMatrix(new double[][]{
            {Math.pow(dt, 4d) / 4d, 0d, Math.pow(dt, 3d) / 2d, 0d},
            {0d, Math.pow(dt, 4d) / 4d, 0d, Math.pow(dt, 3d) / 2d},
            {Math.pow(dt, 3d) / 2d, 0d, Math.pow(dt, 2d), 0d},
            {0d, Math.pow(dt, 3d) / 2d, 0d, Math.pow(dt, 2d)}
        });
        // Error covariance matrix
        P = new Array2DRowRealMatrix(new double[][] {
            {0.5d, 0d, 0d, 0d},
            {0d, 0.5d, 0d, 0d},
            {0d, 0d, 0.5d, 0d},
            {0d, 0d, 0d, 0.5d}
        });
        
        // Create process model, measurement model and kalman filter
        ProcessModel pm = new DefaultProcessModel(A, B, Q, x, P);
        MeasurementModel mm = new DefaultMeasurementModel(H, R);
        filter = new KalmanFilter(pm, mm);
    }
    
    /**
     * Constructs a KalmanFilter with no initial system state
     * @param measNoise Measurement covariance
     * @param time Time interval
     */
    public KalmanFilterPosition(double measNoise, double time) {
        // Set measurement and process error constants
        this.MEAS_NOISE = measNoise;
        // Set discrete time steps
        this.dt = time;
        // A = 
        A = new Array2DRowRealMatrix(new double[][]{
            {1d, 0d},
            {0d, 1d}
        });
        // B = 
        B = new Array2DRowRealMatrix(new double[][]{
            {Math.pow(dt, 2d) / 2d},
            {dt}
        });
        //only observe first 2 values - the position coordinates
        H = new Array2DRowRealMatrix(new double[][]{
            {1d, 0d},
            {0d, 1d}
        });
        // System state with initial state excluded
        x = new ArrayRealVector(new double[] {0, 0});
        // Measurement noise covariance matrix
        R = new Array2DRowRealMatrix(new double[][] {
            {Math.pow(this.MEAS_NOISE, 2d), 0d},
            {0d, Math.pow(this.MEAS_NOISE, 2d)}
        });
        // Process noise covariance matrix
        Q = new Array2DRowRealMatrix(new double[][]{
            {Math.pow(dt, 4d) / 4d, Math.pow(dt, 3d) / 2d},
            {Math.pow(dt, 3d) / 2d, Math.pow(dt, 2d)}
        });
        // Error covariance matrix
        P = new Array2DRowRealMatrix(new double[][] {
            {1d, 0d},
            {0d, 1d}
        });
        
        // Create process model, measurement model and kalman filter
        ProcessModel pm = new DefaultProcessModel(A, B, Q, x, P);
        MeasurementModel mm = new DefaultMeasurementModel(H, R);
        filter = new KalmanFilter(pm, mm);
    }
    
    /**
     * Method to estimate position using Kalman filter
     * @param xy measured position
     * @param xError
     * @param yError
     * @return estimated position
     */
    public double[] estimatePosition(double[] xy, double xError, double yError){
        filter.predict();
        filter.correct(xy);
        double[] points = filter.getStateEstimation();
        // correct errors
        points[0] -= xError;
        points[1] -= yError;
        return points;
    }
    
    /**
     * Method to estimate position using Kalman filter
     * @param xy measured position
     * @return estimated position
     */
    public double[] estimatePosition(double[] xy){
        filter.predict();
        filter.correct(xy);
        return filter.getStateEstimation();
    }
}
