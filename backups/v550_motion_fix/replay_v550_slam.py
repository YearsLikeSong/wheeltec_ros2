import argparse, gzip, hashlib, json, math, os, signal, subprocess, time
from pathlib import Path
import rclpy
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

parser=argparse.ArgumentParser()
parser.add_argument('--yaw',type=float,default=0.)
parser.add_argument('--queue',type=int,default=1)
parser.add_argument('--label',required=True)
parser.add_argument('--normalize',action='store_true')
args=parser.parse_args()
assert os.environ['ROS_DOMAIN_ID']=='176' and os.environ['ROS_LOCALHOST_ONLY']=='1'
root=Path('/home/wheeltec/wheeltec_ros2')
data=json.load(gzip.open(root/'log/v550_motion_recording.json.gz','rt'))['series']
cut=data['/scan'][0]['stamp']+153
scan0=next(v for v in data['/scan'] if v['stamp']>=cut)
stamp0=scan0['stamp']; recv0=scan0['t']; origin=100.
events=[]
for key in ['/scan','odom_combined -> base_footprint']:
    for v in data[key]:
        if v['t']>=recv0:events.append((v['t']-recv0,key,v))
events.sort(key=lambda v:v[0])
rclpy.init(); node=rclpy.create_node('v550_recorded_sensor_replay')
clock_pub=node.create_publisher(Clock,'/clock',10)
scan_pub=node.create_publisher(LaserScan,'/scan_slam',qos_profile_sensor_data)
import sys
sys.path.insert(0,str(root/'src/v550_navigation/scripts'))
from normalize_scan import normalize_scan
tfb=TransformBroadcaster(node); static=StaticTransformBroadcaster(node)
poses=[]; maps=[]; corrections=[]; graphs=[]
node.create_subscription(PoseWithCovarianceStamped,'/pose',lambda m:poses.append([m.pose.pose.position.x,m.pose.pose.position.y,math.atan2(2*m.pose.pose.orientation.w*m.pose.pose.orientation.z,1-2*m.pose.pose.orientation.z**2)]),20)
node.create_subscription(OccupancyGrid,'/map',lambda m:maps.append([m.info.width,m.info.height,hashlib.sha256(m.data.tobytes()).hexdigest()]),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(TFMessage,'/tf',lambda m:corrections.extend([[x.transform.translation.x,x.transform.translation.y,math.atan2(2*x.transform.rotation.w*x.transform.rotation.z,1-2*x.transform.rotation.z**2)] for x in m.transforms if x.child_frame_id=='odom_combined']),100)
node.create_subscription(MarkerArray,'/slam_toolbox/graph_visualization',lambda m:graphs.append(sum(len(v.points) for v in m.markers)),10)
def stamp(sec):
    from builtin_interfaces.msg import Time
    n=round(sec*1e9);return Time(sec=n//1000000000,nanosec=n%1000000000)
tf=TransformStamped();tf.header.frame_id='base_footprint';tf.child_frame_id='laser'
tf.transform.translation.x=.0834767944381037;tf.transform.translation.y=.0000132296970929568;tf.transform.translation.z=.11682
tf.transform.rotation.z=math.sin(args.yaw/2);tf.transform.rotation.w=math.cos(args.yaw/2)
static.sendTransform(tf)
logfile=root/f'log/v550_replay_{args.label}.log'
with logfile.open('w') as log:
    proc=subprocess.Popen(['/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node','--ros-args','--params-file',str(root/'src/v550_navigation/config/slam_v550.yaml'),'-p','use_sim_time:=true','-p',f'scan_queue_size:={args.queue}'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        ready=time.monotonic()
        while time.monotonic()-ready<2:
            clock_pub.publish(Clock(clock=stamp(origin)))
            rclpy.spin_once(node,timeout_sec=.02)
        start=time.monotonic(); idx=0
        while time.monotonic()-start<events[-1][0]+2:
            assert proc.poll() is None,'SLAM exited'
            elapsed=time.monotonic()-start
            clock_pub.publish(Clock(clock=stamp(origin+elapsed)))
            while idx<len(events) and events[idx][0]<=elapsed:
                _,key,v=events[idx];idx+=1
                when=stamp(origin+v['stamp']-stamp0)
                if key=='/scan':
                    m=LaserScan();m.header.frame_id='laser';m.header.stamp=when
                    for key in ['angle_min','angle_max','time_increment','scan_time','range_min','range_max']:setattr(m,key,v[key])
                    m.angle_increment=v['increment'];m.ranges=[float('inf') if x is None else x for x in v['ranges']]
                    scan_pub.publish(normalize_scan(m) if args.normalize else m)
                else:
                    m=TransformStamped();m.header.frame_id='odom_combined';m.child_frame_id='base_footprint';m.header.stamp=when
                    m.transform.translation.x=v['x'];m.transform.translation.y=v['y']
                    m.transform.rotation.z=math.sin(v['yaw']/2);m.transform.rotation.w=math.cos(v['yaw']/2)
                    tfb.sendTransform(m)
            rclpy.spin_once(node,timeout_sec=.005)
        result={'label':args.label,'laser_yaw':args.yaw,'queue':args.queue,'pose_count':len(poses),'map_versions':len(set(x[2] for x in maps)),'map_first_last':[maps[0],maps[-1]] if maps else [],'poses':poses,'tf_first_last':[corrections[0],corrections[-1]] if corrections else [],'graph_max_points':max(graphs,default=0)}
        (root/f'log/v550_replay_{args.label}.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result),flush=True)
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
        node.destroy_node();rclpy.shutdown()
