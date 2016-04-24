package org.lcsr.moverio.moverio_ros;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import org.ros.internal.message.RawMessage;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import geometry_msgs.PoseStamped;
import geometry_msgs.Quaternion;

/**
 * Created by qian on 11/3/15.
 */
public class SensorPublisher extends AbstractNodeMain {
    private final SensorManager sensorManager;
    private SensorPublisher.SensorInfoListener infoListener;

    public SensorPublisher(SensorManager sensorManager) {
        this.sensorManager = sensorManager;
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("moverio/sensor");
    }

    public void onStart(ConnectedNode connectedNode) {
        try {
            Publisher pub = connectedNode.newPublisher("moverio/pose", "geometry_msgs/PoseStamped");
            this.infoListener = new SensorPublisher.SensorInfoListener(pub);
            Sensor oriSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
            Sensor posSensor = this.sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
            this.sensorManager.registerListener(this.infoListener, oriSensor, SensorManager.SENSOR_DELAY_NORMAL);
            this.sensorManager.registerListener(this.infoListener, posSensor, SensorManager.SENSOR_DELAY_NORMAL);
        } catch (Exception var4) {
            connectedNode.getLog().fatal(var4);
        }

    }

    private final class SensorInfoListener implements SensorEventListener {
        private final Publisher<PoseStamped> publisher;
        private double seq = 0.0;
        private boolean posUpdated = false;
        private boolean oriUpdated = false;
        private PoseStamped pose;
        private float[] quaternion;

        private SensorInfoListener(Publisher<PoseStamped> var1) {
            this.publisher = var1;
            pose = this.publisher.newMessage();
            pose.getHeader().setFrameId("/map");
            quaternion = new float[4];
        }

        public void onAccuracyChanged(Sensor sensor, int accuracy) {
        }

        public void onSensorChanged(SensorEvent event) {
            if(event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
                SensorManager.getQuaternionFromVector(quaternion, event.values);
                pose.getPose().getOrientation().setW(quaternion[0]);
                pose.getPose().getOrientation().setX(quaternion[1]);
                pose.getPose().getOrientation().setY(quaternion[2]);
                pose.getPose().getOrientation().setZ(quaternion[3]);
                oriUpdated = true;
            }
            else if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
                pose.getPose().getPosition().setX(event.values[0]);
                pose.getPose().getPosition().setY(event.values[1]);
                pose.getPose().getPosition().setZ(event.values[2]);
                posUpdated = true;
            }
            if ( posUpdated && oriUpdated ) {
                pose.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
                seq += 1.0;
                this.publisher.publish(pose);
                posUpdated = false;
                oriUpdated = false;
            }

        }
    }

}
