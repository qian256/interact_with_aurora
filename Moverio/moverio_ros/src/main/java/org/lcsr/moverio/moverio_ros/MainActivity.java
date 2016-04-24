package org.lcsr.moverio.moverio_ros;

import android.content.Context;
import android.hardware.Camera;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.Toast;
import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.MessageCallable;
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.android.view.RosTextView;
import org.ros.android.view.camera.RosCameraPreviewView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import sensor_msgs.CompressedImage;

public class MainActivity extends RosActivity {
    private int cameraId;
    private RosCameraPreviewView rosCameraPreviewView;
    private RosImageView<CompressedImage> rosImageView;
    private RosTextView<std_msgs.String> rosTextView;
    private LinearLayout mainLayout;


    public MainActivity() {
        super("MOVERIO_ROS", "MOVERIO_ROS");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.main);

        rosCameraPreviewView = (RosCameraPreviewView) findViewById(R.id.ros_camera_preview_view);
        rosImageView = (RosImageView<CompressedImage>) findViewById( R.id.ros_image_view);
        rosImageView.setTopicName("/moverio/view/compressed");
        rosImageView.setMessageType(CompressedImage._TYPE);
        rosImageView.setMessageToBitmapCallable(new BitmapFromCompressedImage());


        rosTextView = (RosTextView<std_msgs.String>) findViewById(R.id.ros_text_view);
        rosTextView.setTopicName("/moverio/textview");
        rosTextView.setMessageType(std_msgs.String._TYPE);

        rosTextView.setMessageToStringCallable(new MessageCallable<String, std_msgs.String>() {
            @Override
            public String call(std_msgs.String message) {
                Log.i("Main", "text message");
                return message.getData();
            }
        });

        mainLayout = (LinearLayout) findViewById( R.id.main_layout );

    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        cameraId = 0;
        String hostAddress = InetAddressFactory.newNonLoopback().getHostAddress();
        Camera cam = Camera.open(cameraId);
        Camera.Parameters param = cam.getParameters();
        param.setPictureSize(640,480);
        param.setPreviewFpsRange(10000, 15000);
        param.setPreviewFrameRate(5);
        cam.setParameters(param);
        rosCameraPreviewView.setCamera(cam);
        NodeConfiguration ncCamera = NodeConfiguration.newPublic(hostAddress);
        ncCamera.setNodeName("camera");
        ncCamera.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(rosCameraPreviewView, ncCamera);

        NodeConfiguration ncImage = NodeConfiguration.newPublic(hostAddress);
        ncImage.setMasterUri(getMasterUri());
        ncImage.setNodeName("imagebox");
        nodeMainExecutor.execute(rosImageView, ncImage);

        NodeConfiguration ncText = NodeConfiguration.newPublic(hostAddress);
        ncText.setMasterUri(getMasterUri());
        ncText.setNodeName("textbox");
        nodeMainExecutor.execute(rosTextView, ncText);
    }

    @Override
    public void onResume(){
        super.onResume();
//        hideCameraPreview();
    }

    private void hideCameraPreview() {
        FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(100,100);
        rosCameraPreviewView.setLayoutParams(params);
    }
}
