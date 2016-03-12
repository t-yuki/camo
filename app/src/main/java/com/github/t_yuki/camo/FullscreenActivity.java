package com.github.t_yuki.camo;

import android.app.Activity;
import android.os.Bundle;
import android.view.View;

import com.github.t_yuki.camo.util.SystemUiHider;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

import orbotix.robot.base.Robot;
import orbotix.robot.base.RobotProvider;
import orbotix.sphero.ConnectionListener;
import orbotix.sphero.Sphero;
import orbotix.view.connection.SpheroConnectionView;


/**
 * An example full-screen activity that shows and hides the system UI (i.e.
 * status bar and navigation/system bar) with user interaction.
 *
 * @see SystemUiHider
 */
public class FullscreenActivity extends Activity implements CameraBridgeViewBase.CvCameraViewListener2 {
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

    /**
     * The Sphero Connection View
     */
    private SpheroConnectionView mSpheroConnectionView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_fullscreen);

        // カメラビューのインスタンスを変数にバインド
        mCameraView = (CameraBridgeViewBase) findViewById(R.id.camera_view);
        // リスナーの設定 (後述)
        mCameraView.setCvCameraViewListener(this);


        mSpheroConnectionView = (SpheroConnectionView) findViewById(R.id.sphero_connection_view);
        mSpheroConnectionView.addConnectionListener(new ConnectionListener() {

            @Override
            public void onConnected(Robot robot) {
                //SpheroConnectionView is made invisible on connect by default
                mRobot = (Sphero) robot;
            }

            @Override
            public void onConnectionFailed(Robot sphero) {
                // let the SpheroConnectionView handle or hide it and do something here...
            }

            @Override
            public void onDisconnected(Robot sphero) {
                mSpheroConnectionView.startDiscovery();
            }
        });
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        // カメラプレビュー開始時に呼ばれる
    }

    @Override
    public void onCameraViewStopped() {
        // カメラプレビュー終了時に呼ばれる
    }

    // CvCameraViewListener2 の場合
    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        return inputFrame.rgba();
    }

    @Override
    protected void onResume() {
        super.onResume();
        // 非同期でライブラリの読み込み/初期化を行う
        // static boolean initAsync(String Version, Context AppContext, LoaderCallbackInterface Callback)
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_9, this, mLoaderCallback);

        // Refresh list of Spheros
        mSpheroConnectionView.startDiscovery();
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
        // Disconnect Robot properly
        RobotProvider.getDefaultProvider().disconnectControlledRobots();
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
        }
    }


    /**
     * When the user clicks a control button, roll the Robot in that direction
     *
     * @param v The View that had been clicked
     */
    public void onControlClick(View v) {
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
}
