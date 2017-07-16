import com.google.common.eventbus.Subscribe;

public class MainPosition {
    private Exporter exporter;


    // Error bounds
    private final static double MEAS_NOISE = 0.00001d;
//    private final static double MEAS_NOISE = 1000d;
    private final static double PROC_NOISE = 1d;
//    private final static double X_ERROR = 0.025d;
//    private final static double Y_ERROR = -0.053d;
    private final static double X_ERROR = 0.0d;//经度误差
    private final static double Y_ERROR = 0.0d;//维度误差

    // Discrete TIME
    private static final double TIME = 0.001d;

    // coordinate kalman filter
    private static KalmanFilterPosition filter;

    public void init(double x, double y) {
        this.filter = new KalmanFilterPosition(MainPosition.MEAS_NOISE, MainPosition.TIME, x, y);
    }

    public static double getDifference(double x1, double y1, double x2, double y2){
        return Math.sqrt(Math.pow((x2-x1), 2) + Math.pow((y2-y1), 2));
    }

    public MainPosition() {
        exporter = new Exporter();
        registerBus();
    }

    private void registerBus() {
        BusProvider.getInstance().register(this);
    }

    @Subscribe
    public void onLocationUpdate(GPSSingleData singleData) {
        if (null==filter){
            init(singleData.getLongitude(),singleData.getLatitude());
        }else {
            double [] coord={singleData.getLongitude(),singleData.getLatitude()};
            double[] lonlat = filter.estimatePosition(coord, X_ERROR, Y_ERROR);
            double distance = getDifference(lonlat[0], lonlat[1], coord[0], coord[1]);
            System.out.println(distance);

            GPSSingleData newGPSdata = new GPSSingleData(0, lonlat[0], lonlat[1], 0);
            exporter.writeData(newGPSdata.toString());

        }


    }
    /**
     * Main method
     *
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        new MainPosition();
        new GPSDataFactory();

    }
}
