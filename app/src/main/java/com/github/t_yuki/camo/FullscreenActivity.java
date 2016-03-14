package com.github.t_yuki.camo;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;

import com.github.t_yuki.camo.util.SystemUiHider;
import com.orbotix.DualStackDiscoveryAgent;
import com.orbotix.Sphero;
import com.orbotix.common.DiscoveryException;
import com.orbotix.common.Robot;
import com.orbotix.common.RobotChangedStateListener;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


/**
 * An example full-screen activity that shows and hides the system UI (i.e.
 * status bar and navigation/system bar) with user interaction.
 *
 * @see SystemUiHider
 */
public class FullscreenActivity extends Activity
        implements CameraBridgeViewBase.CvCameraViewListener2,
        RobotChangedStateListener,
        SensorEventListener {
    private SensorManager mSensorManager;
    private Sensor mAccel;
    private Sensor mMagnetic;
    private CameraBridgeViewBase mCameraView;
    // ライブラリ初期化完了後に呼ばれるコールバック (onManagerConnected)
    // public abstract class BaseLoaderCallback implements LoaderCallbackInterface
    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                // 読み込みが成功したらカメラプレビューを開始
                case LoaderCallbackInterface.SUCCESS:
                    mCameraView.enableView();
                    break;
                default:
                    super.onManagerConnected(status);
                    break;
            }
        }
    };

    private Sphero mRobot;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_fullscreen);

        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mAccel = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mMagnetic = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        // カメラビューのインスタンスを変数にバインド
        mCameraView = (CameraBridgeViewBase) findViewById(R.id.camera_view);
        // リスナーの設定 (後述)
        mCameraView.setCvCameraViewListener(this);

        DualStackDiscoveryAgent.getInstance().addRobotStateListener(this);
    }

    @Override
    public void handleRobotChangedState(Robot robot, RobotChangedStateListener.RobotChangedStateNotificationType robotChangedStateNotificationType) {
        switch (robotChangedStateNotificationType) {
            case Online:
                mRobot = new Sphero(robot);
                break;
            case Disconnected:
                mRobot = null;
                startDiscovery();
                break;
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        DualStackDiscoveryAgent.getInstance().addRobotStateListener(null);
    }

    private void startDiscovery() {
        //If the DiscoveryAgent is not already looking for robots, start discovery.
        if (!DualStackDiscoveryAgent.getInstance().isDiscovering()) {
            try {
                DualStackDiscoveryAgent.getInstance().startDiscovery(this);
            } catch (DiscoveryException e) {
                Log.e("Sphero", "DiscoveryException: " + e.getMessage());
            }
        }
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        // カメラプレビュー開始時に呼ばれる
    }

    @Override
    public void onCameraViewStopped() {
        // カメラプレビュー終了時に呼ばれる
    }

    // HSV Color range to detect: orange
    Scalar low = new Scalar(7, 170, 115);
    Scalar up = new Scalar(25, 255, 255);
    // HSV Color range to detect: green
    //  Scalar low = new Scalar(29, 86, 60);
    //  Scalar up = new Scalar(64, 255, 255);


    Mat w1;
    Mat w2;

    // CvCameraViewListener2 の場合
    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat in = inputFrame.rgba();
        if (w1 == null) {
            w1 = new Mat((int) (in.rows() * 0.25), (int) (in.cols() * 0.25), CvType.CV_8UC1);
            w2 = new Mat((int) (in.rows() * 0.25), (int) (in.cols() * 0.25), CvType.CV_8UC1);
        }
        Imgproc.resize(in, w1, w1.size(), 0, 0, Imgproc.INTER_CUBIC);

        Imgproc.cvtColor(w1, w2, Imgproc.COLOR_RGB2HSV);
        Core.inRange(w2, low, up, w1);
        Imgproc.erode(w1, w2, Mat.ones(5, 5, CvType.CV_8UC1), new Point(-1, -1), 2);
        Imgproc.dilate(w2, w1, Mat.ones(5, 5, CvType.CV_8UC1), new Point(-1, -1), 2);

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hier = new Mat();

        Imgproc.findContours(w1, contours, hier, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() == 0) {
            return in;
        }

        for (MatOfPoint area : contours) {
            double sz = Imgproc.contourArea(area);

            MatOfPoint2f area2f = new MatOfPoint2f(area.toArray());
            Point center = new Point();
            float[] rads = new float[1];
            Imgproc.minEnclosingCircle(area2f, center, rads);
            int rad = (int) rads[0];

            double csz = 2 * rads[0] * rads[0] * 3.14;
            Log.w("CAMO", "area: " + sz + " circle sz:" + csz);
            if (sz * 6 < csz) { // heuristic: not filled circle
                continue;
            }

            Core.circle(in, new Point(center.x * 4, center.y * 4), rad * 4, new Scalar(0, 0, 255));
            if (rad < 20) { // too small
                continue;
            }
            if (mRobot == null) {
                return in;
            }
            float speed = 10.0f / rad; // 0% to 100%
            if (speed > 1.0f) {
                speed = 1.0f;
            }
//            mRobot.drive(0f, speed); // TODO: which heading is front??
            return in;
        }
        return in;
    }

    @Override
    protected void onResume() {
        super.onResume();

        mSensorManager.registerListener(this, mAccel, SensorManager.SENSOR_DELAY_NORMAL);
        mSensorManager.registerListener(this, mMagnetic, SensorManager.SENSOR_DELAY_NORMAL);

        // 非同期でライブラリの読み込み/初期化を行う
        // static boolean initAsync(String Version, Context AppContext, LoaderCallbackInterface Callback)
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_9, this, mLoaderCallback);
        startDiscovery();
    }

    @Override
    protected void onPostCreate(Bundle savedInstanceState) {
        super.onPostCreate(savedInstanceState);
    }

    /**
     * Called when the user presses the back or home button
     */
    @Override
    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(this);
        // Disconnect Robot properly
        //If the DiscoveryAgent is in discovery mode, stop it.
        if (DualStackDiscoveryAgent.getInstance().isDiscovering()) {
            DualStackDiscoveryAgent.getInstance().stopDiscovery();
        }

        //If a robot is connected to the device, disconnect it
        if (mRobot != null) {
            mRobot.disconnect();
            mRobot = null;
        }
    }

    /**
     * When the user clicks "STOP", stop the Robot.
     *
     * @param v The View that had been clicked
     */
    public void onStopClick(View v) {
        if (mRobot != null) {
            // Stop robot
            mRobot.stop();
            mRobot = null;
        }
    }


    /**
     * When the user clicks a control button, roll the Robot in that direction
     *
     * @param v The View that had been clicked
     */
    public void onControlClick(View v) {
        if (mRobot == null) {
            return
        }

            // Find the heading, based on which button was clicked
        final float heading;
        switch (v.getId()) {

            case R.id.ninety_button:
                heading = 90f;
                break;

            case R.id.one_eighty_button:
                heading = 180f;
                break;

            case R.id.two_seventy_button:
                heading = 270f;
                break;

            default:
                heading = 0f;
                break;
        }

        // Set speed. 60% of full speed
        final float speed = 0.6f;

        // Roll robot
        mRobot.drive(heading, speed);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (mRobot == null) {
            return;
        }
        if(event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
onSensorChangedGyro(event);
        }
    }

    // http://developer.android.com/intl/ja/guide/topics/sensors/sensors_motion.html#sensors-motion-accel
    // Create a constant to convert nanoseconds to seconds.
    private static final float NS2S = 1.0f / 1000000000.0f;
    private final float[] deltaRotationVector = new float[4]();
    private float timestamp;

    protected void onSensorChangedGyro(SensorEvent event){
        // This timestep's delta rotation to be multiplied by the current rotation
        // after computing it from the gyro sample data.
        if (timestamp != 0) {
            final float dT = (event.timestamp - timestamp) * NS2S;
            // Axis of the rotation sample, not normalized yet.
            float axisX = event.values[0];
            float axisY = event.values[1];
            float axisZ = event.values[2];

            // Calculate the angular speed of the sample
            float omegaMagnitude = (float) Math.sqrt(axisX*axisX + axisY*axisY + axisZ*axisZ);

            // Normalize the rotation vector if it's big enough to get the axis
            // (that is, EPSILON should represent your maximum allowable margin of error)
            if (omegaMagnitude > EPSILON) {
                axisX /= omegaMagnitude;
                axisY /= omegaMagnitude;
                axisZ /= omegaMagnitude;
            }

            // Integrate around this axis with the angular speed by the timestep
            // in order to get a delta rotation from this sample over the timestep
            // We will convert this axis-angle representation of the delta rotation
            // into a quaternion before turning it into the rotation matrix.
            float thetaOverTwo = omegaMagnitude * dT / 2.0f;
            float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
            float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);
            deltaRotationVector[0] = sinThetaOverTwo * axisX;
            deltaRotationVector[1] = sinThetaOverTwo * axisY;
            deltaRotationVector[2] = sinThetaOverTwo * axisZ;
            deltaRotationVector[3] = cosThetaOverTwo;

            float dy = deltaRotationVector[1];
            // TODO: minimize dy
        }
        timestamp = event.timestamp;
        float[] deltaRotationMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(deltaRotationMatrix, deltaRotationVector);
        // User code should concatenate the delta rotation we computed with the current rotation
        // in order to get the updated rotation.
        // rotationCurrent = rotationCurrent * deltaRotationMatrix;
    }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }
}
