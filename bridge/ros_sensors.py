"""Read-only ROS2 subscriptions and bounded PointCloud2 previews."""
import copy
import json
from collections import deque
import math
import struct
import threading
import time
import sys
import os
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

TOPICS=['/SLAM_ALIGNED_POINTS','/SLAM_ACCUMULATED_POINTS_MAP','/SLAM_CLOUD_REGISTERED_BODY','/ALIGNED_POINTS','/FULL_CLOUD_MAP','/LOC_BODY_POINTS','/LIDAR/POINTS','/LIDAR/POINTS2','/NAV_POINTS']


def point_cloud(message,limit=8000):
    fields={f.name:f for f in message.fields}
    axes=[fields.get(name) for name in ['x','y','z']]
    if any(f is None or f.datatype not in [7,8] or f.count!=1 for f in axes):
        raise ValueError('点云缺少浮点 XYZ 字段')
    width,height=int(message.width),int(message.height)
    stride,row_step=int(message.point_step),int(message.row_step)
    if width<=0 or height<=0 or stride<=0 or row_step<width*stride or len(message.data)<height*row_step:
        raise ValueError('PointCloud2 数据长度或步长无效')
    formats=[('>' if message.is_bigendian else '<')+('f' if f.datatype==7 else 'd') for f in axes]
    if any(f.offset<0 or f.offset+struct.calcsize(fmt)>stride for f,fmt in zip(axes,formats)):
        raise ValueError('PointCloud2 字段偏移无效')
    # Vectorized sampling keeps large display clouds off the control callback budget.
    import numpy as np
    total=width*height;step=max(1,math.ceil(total/limit));index=np.arange(0,total,step)
    row,column=index//width,index%width;columns=[]
    for field,fmt in zip(axes,formats):
        dtype=np.dtype(('>' if message.is_bigendian else '<')+('f4' if field.datatype==7 else 'f8'))
        view=np.ndarray((height,width),dtype=dtype,buffer=message.data,offset=field.offset,strides=(row_step,stride))
        columns.append(view[row,column])
    points=np.column_stack(columns).astype(float);points=points[np.isfinite(points).all(axis=1)&(np.abs(points)<1e7).all(axis=1)]
    if not len(points):raise ValueError('当前点云没有有效 XYZ 点')
    return dict(ok=True,points=np.round(points,4).tolist(),total_points=total,shown_points=len(points),sampled=len(points)<total,
                bounds=[points.min(axis=0).tolist(),points.max(axis=0).tolist()],fields=list(fields),encoding='ROS2 PointCloud2',units='m',
                frame_id=message.header.frame_id,stamp=dict(sec=message.header.stamp.sec,nanosec=message.header.stamp.nanosec))



class RosSensors:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2,Imu
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Float64MultiArray
        self.rclpy=rclpy;self.lock=threading.RLock();self.clouds={};self.errors={};self.last={};self.imu=None;self.odom=None;self.slam_odom=None;self.autonomy_scan=None;self.autonomy_error=None;self.autonomy_last=0;self.autonomy_until=0
        rclpy.init();self.node=Node('m20_platform_sensors_'+str(os.getpid()))
        qos_profile_sensor_data=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cloud_subscriptions={};self.cloud_until={};self.point_type=PointCloud2;self.cloud_qos=qos_profile_sensor_data
        self.aligned_raw=None;self.location_quality=None;self.match_quality=None;self.refinement_error=None
        # Per-device contract, verified against its installed localization headers.
        # New public ROS2 docs use 1; this robot's installed monitor uses 0.
        contract=Path('/opt/m20-platform/location-contract.json')
        self.normal_location_status=json.loads(contract.read_text()).get('normal_status',1) if contract.is_file() else 1
        self.lidar_raw=None;self.slam_raw={};self.slam_pose_proof=None;self.slam_history=deque(maxlen=40);self.odom_history=deque(maxlen=40);self.ground_ready=threading.Condition(self.lock);self.ground_event=threading.Event();self.ground_done=threading.Event();self.ground_thread=threading.Thread(target=self.ground_loop,daemon=True);self.ground_thread.start()
        try:
            from drdds.msg import LocationStatus
            self.node.create_subscription(LocationStatus,'/LOCATION_STATUS',self.location_update,qos_profile_sensor_data)
        except ImportError:self.refinement_error='当前 ROS 环境缺少 LocationStatus 消息，需同步定位消息环境'
        self.node.create_subscription(Float64MultiArray,'/LOCATION_STATUS/MATCHING_ERROR',self.match_update,qos_profile_sensor_data)
        self.ensure_topic('/LIDAR/POINTS')
        self.cleanup_timer=self.node.create_timer(1.,self.expire_clouds)
        self.node.create_subscription(Imu,'/IMU',self.imu_update,qos_profile_sensor_data)
        self.node.create_subscription(Odometry,'/ODOM',self.odom_update,qos_profile_sensor_data)
        self.node.create_subscription(Odometry,'/SLAM_ODOM',lambda msg:self.odom_update(msg,True),qos_profile_sensor_data)
        self.spin_error=None
        self.spin_thread=threading.Thread(target=self.spin,daemon=True)
        self.spin_thread.start()

    def ensure_topic(self,topic):
        if topic not in TOPICS:raise ValueError('不支持的点云话题')
        if topic in ['/FULL_CLOUD_MAP','/SLAM_ACCUMULATED_POINTS_MAP'] and time.monotonic()<self.autonomy_until:
            raise ValueError('自主作业期间请查看当前扫描 /SLAM_ALIGNED_POINTS；大地图预览稍后恢复')
        with self.lock:
            self.cloud_until[topic]=time.monotonic()+3
            if topic not in self.cloud_subscriptions:
                self.cloud_subscriptions[topic]=self.node.create_subscription(self.point_type,topic,lambda msg,t=topic:self.cloud(t,msg),self.cloud_qos)

    def expire_clouds(self):
        with self.lock:
            now=time.monotonic()
            for topic,subscription in list(self.cloud_subscriptions.items()):
                if topic=='/LIDAR/POINTS' and now<self.autonomy_until:continue
                heavy=topic in ['/FULL_CLOUD_MAP','/SLAM_ACCUMULATED_POINTS_MAP']
                if self.cloud_until.get(topic,0)<now or heavy and now<self.autonomy_until:
                    self.node.destroy_subscription(subscription);del self.cloud_subscriptions[topic]

    def spin(self):
        try:self.rclpy.spin(self.node)
        except Exception as exc:self.spin_error=type(exc).__name__+': '+str(exc)

    def cloud(self,topic,message):
        if topic=='/ALIGNED_POINTS':
            with self.lock:self.aligned_raw=(message,time.monotonic())
        if topic in ['/SLAM_CLOUD_REGISTERED_BODY','/SLAM_ALIGNED_POINTS']:
            with self.lock:self.slam_raw[topic]=(message,time.monotonic())
        # Keep ROS delivery responsive: only the latest full scan is queued.
        if topic=='/LIDAR/POINTS':
            stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
            with self.lock:self.lidar_raw=(message,time.monotonic(),self.node.get_clock().now().nanoseconds/1e9-stamp)
            if time.monotonic()<self.autonomy_until:self.ground_event.set()
        # Parse at most one frame per second per topic, keeping DDS callbacks short.
        now=time.monotonic()
        if now-self.last.get(topic,0)<1:return
        self.last[topic]=now
        try:
            data=point_cloud(message);data.update(topic=topic,name=topic,source='ros2',received_at=time.time(),received_monotonic=now)
            with self.lock:self.clouds[topic]=data;self.errors.pop(topic,None)
        except ValueError as exc:
            with self.lock:self.errors[topic]=str(exc)

    def ground_loop(self):
        last=None
        while not self.ground_done.is_set():
            self.ground_event.wait(.3);self.ground_event.clear()
            with self.lock:raw=self.lidar_raw;enabled=time.monotonic()<self.autonomy_until
            if not raw or not enabled or raw[1]==last:continue
            message,received,lag=raw;last=received
            try:
                from autonomy_sensing import xyz,supported_floor
                if message.header.frame_id not in ['base_link','lidar_link']:raise ValueError('雷达坐标系未核对：'+message.header.frame_id)
                # Reject only scans already beyond the final age budget. A
                # 350 ms source-age gate discarded frames that could still
                # finish within 650 ms; final API/transport deadlines remain.
                if not -.1<=lag+time.monotonic()-received<=.65:raise TimeoutError('雷达源时间戳过期或时钟不一致')
                began=time.monotonic();scan=supported_floor(xyz(message))
                try:
                    from native_sensing import backend_status
                    backend=backend_status()
                    scan['processing_backend']='native' if backend.get('available') and backend.get('last_reason') is None else 'numpy'
                    scan['native_fallback_reason']=backend.get('last_reason')
                except ImportError:scan['processing_backend']='numpy'
                scan.update(received_monotonic=received,source_age=max(0,lag),processing_ms=round((time.monotonic()-began)*1000,1),
                            stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9,frame_id='base_link',topic='/LIDAR/POINTS',source_frame=message.header.frame_id)
                with self.lock:self.autonomy_scan=scan;self.autonomy_error=None;self.ground_ready.notify_all()
            except TimeoutError as exc:self.ground_failure(exc,temporal=True)
            except Exception as exc:self.ground_failure(exc)

    def ground_failure(self,error,temporal=False):
        with self.lock:
            previous=self.autonomy_scan
            fresh=previous and time.monotonic()-previous['received_monotonic']+previous['source_age']<=.65
            # A late next packet does not invalidate a still-fresh previous
            # observation. Never renew its timestamps or reuse bad geometry.
            if not temporal or not fresh:self.autonomy_scan=None
            self.autonomy_error=str(error);self.ground_ready.notify_all()

    def imu_update(self,message):
        v=lambda q:[q.x,q.y,q.z]
        row=dict(frame_id=message.header.frame_id,received_at=time.time(),angular_velocity=v(message.angular_velocity),
                 linear_acceleration=v(message.linear_acceleration),orientation=v(message.orientation)+[message.orientation.w])
        if not all(math.isfinite(x) for key in ['angular_velocity','linear_acceleration','orientation'] for x in row[key]):return
        with self.lock:self.imu=row

    def odom_update(self,message,slam=False):
        p=message.pose.pose.position;q=message.pose.pose.orientation
        if not all(math.isfinite(v) for v in [p.x,p.y,p.z,q.x,q.y,q.z,q.w]):return
        row=dict(frame_id=message.header.frame_id,child_frame_id=message.child_frame_id,received_at=time.time(),received_monotonic=time.monotonic(),
                 quaternion=[q.x,q.y,q.z,q.w],
                 stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                 tilt=math.acos(max(-1,min(1,1-2*(q.x*q.x+q.y*q.y)))),PosX=p.x,PosY=p.y,PosZ=p.z,
                 Yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        with self.lock:
            if slam:
                if not self.slam_odom or row['received_monotonic']-self.slam_odom['received_monotonic']>1:self.slam_pose_proof=None
                self.slam_odom=row;self.slam_history.append(row)
            else:
                self.odom=row
                self.odom_history.append(row)

    def status(self):
        with self.lock:
            now=time.time()
            return dict(ok=True,source='ros2',autonomy_protocol=1,refinement_protocol=1,refinement_error=self.refinement_error,autonomy_error=self.autonomy_error,imu=copy.deepcopy(self.imu),odom=copy.deepcopy(self.odom),slam_odom=copy.deepcopy(self.slam_odom),slam_pose_proof=copy.deepcopy(self.slam_pose_proof),
                        localization_quality=dict(status=self.location_quality[0],normal_status=self.normal_location_status,age=round(time.monotonic()-self.location_quality[1],3)) if self.location_quality else None,
                        matching_quality=dict(error=self.match_quality[0][0],inlier_ratio=self.match_quality[0][1],age=round(time.monotonic()-self.match_quality[1],3)) if self.match_quality else None,
                        scan_diagnostics=dict(age=round(time.monotonic()-self.autonomy_scan['received_monotonic']+self.autonomy_scan['source_age'],3),source_age=self.autonomy_scan['source_age'],processing_ms=self.autonomy_scan.get('processing_ms'),floor_cells=len(self.autonomy_scan['floor']),obstacle_cells=len(self.autonomy_scan['obstacles'])) if self.autonomy_scan else None,
                        reader_uid=os.geteuid(),reader_pid=os.getpid(),spin_alive=self.spin_thread.is_alive(),spin_error=self.spin_error,
                        discovered_topics=len(self.node.get_topic_names_and_types()),
                        lidar_publishers=self.node.count_publishers('/LIDAR/POINTS'),
                        nav_publishers=self.node.count_publishers('/NAV_POINTS'),
                        lidar_matches=self.cloud_subscriptions['/LIDAR/POINTS'].get_publisher_count() if hasattr(self.cloud_subscriptions.get('/LIDAR/POINTS'),'get_publisher_count') else None,
                        topics=[dict(topic=t,available=t in self.clouds,age=round(now-self.clouds[t]['received_at'],2) if t in self.clouds else None,
                                     frame_id=self.clouds[t]['frame_id'] if t in self.clouds else None,error=self.errors.get(t)) for t in TOPICS])

    def preview(self,topic):
        self.ensure_topic(topic)
        with self.lock:
            if topic not in self.clouds:raise ValueError(self.errors.get(topic,'尚未收到此话题，请检查建图/定位服务、ROS 环境、DDS domain 与话题可见性'))
            data=copy.deepcopy(self.clouds[topic]);data.pop('received_monotonic',None)
            data.update(age=round(time.time()-data['received_at'],2),fresh=time.time()-data['received_at']<3)
            odom=self.slam_odom if topic.startswith('/SLAM_') else self.odom
            if odom and odom['frame_id']==data['frame_id'] and time.time()-odom['received_at']<2:data['odom']=copy.deepcopy(odom)
            return data

    def autonomy(self,mapping=False):
        query_started=time.monotonic()
        self.ensure_topic('/LIDAR/POINTS')
        with self.lock:self.autonomy_until=time.monotonic()+2;self.ground_event.set()
        self.expire_clouds()
        if mapping:
            with self.lock:
                pose=copy.deepcopy(self.slam_odom);proof=self.slam_pose_proof;body=self.slam_raw.get('/SLAM_CLOUD_REGISTERED_BODY');world=self.slam_raw.get('/SLAM_ALIGNED_POINTS')
            if pose and not pose['child_frame_id'] and not proof:
                self.ensure_topic('/SLAM_CLOUD_REGISTERED_BODY');self.ensure_topic('/SLAM_ALIGNED_POINTS')
                if not body or not world or max(time.monotonic()-body[1],time.monotonic()-world[1])>.5:raise ValueError('等待新鲜的 SLAM 坐标对照点云')
                from slam_pose_check import verify_slam_body_pose
                proof=verify_slam_body_pose(body[0],world[0],pose)
                with self.lock:self.slam_pose_proof=proof
        with self.lock:
            self.autonomy_until=time.monotonic()+2
            self.ground_event.set()
            initial_stamp=self.autonomy_scan.get('stamp') if self.autonomy_scan else None
            deadline=time.monotonic()+.22
            while time.monotonic()<deadline:
                candidate=self.autonomy_scan
                if candidate:
                    age=time.monotonic()-candidate['received_monotonic']+candidate['source_age']
                    stamp=candidate.get('stamp')
                    newer=(isinstance(stamp,(int,float)) and math.isfinite(stamp)
                           and (initial_stamp is None or stamp>initial_stamp))
                    # A newly processed scan can already be 420–650 ms old on
                    # this CPU. Do not keep aging it until a fixed wait expires
                    # while hoping for an impossible <420 ms successor. A
                    # notification or duplicate/reordered stamp is not new data.
                    if age<=.42 or newer and age<=.65:break
                self.ground_ready.wait(deadline-time.monotonic())
            if not self.autonomy_scan:raise ValueError(self.autonomy_error or '等待完整雷达扫描与地面观测')
            data=copy.deepcopy(self.autonomy_scan)
            data['age']=time.monotonic()-data.pop('received_monotonic')+data.pop('source_age')
            if data['age']>.65:raise ValueError('雷达更新中断，已停止自主移动')
            if mapping:
                pose=min(self.slam_history,key=lambda p:abs(p['stamp']-data['stamp'])) if self.slam_history else None
                if not pose or time.monotonic()-pose['received_monotonic']>.65 or abs(pose['stamp']-data['stamp'])>.05:
                    raise ValueError('等待与雷达同步的 SLAM 位姿')
                if pose['child_frame_id']!='base_link' and not (pose['child_frame_id']=='' and self.slam_pose_proof):raise ValueError('SLAM 位姿子坐标系尚未核对：'+str(pose['child_frame_id']))
                if pose['tilt']>.14:raise ValueError('机身倾斜，已停止自主探索')
                data['pose']=[pose['PosX'],pose['PosY'],pose['Yaw']]
            else:
                # The point-goal executor needs the localized pose belonging to
                # this scan. Follow mode may operate without global localization,
                # so preserve its surface observation and report missing pose.
                history=getattr(self,'odom_history',())
                pose=min(history,key=lambda p:abs(p['stamp']-data['stamp'])) if history else None
                loc=getattr(self,'location_quality',None);now=time.monotonic()
                if (pose and now-pose['received_monotonic']<.65 and abs(pose['stamp']-data['stamp'])<=.10
                    and pose['frame_id']=='map' and pose['child_frame_id'] in ['','base_link'] and pose['tilt']<.14
                    and loc and now-loc[1]<1.5 and loc[0]==getattr(self,'normal_location_status',1)):
                    data.update(pose=[pose['PosX'],pose['PosY'],pose['Yaw']],pose_stamp=pose['stamp'],pose_source='/ODOM',pose_frame_id='map')
                else:data['pose_error']='等待与雷达时间匹配的正常地图定位'
            return dict(data,ok=True,protocol=1,server_elapsed_s=time.monotonic()-query_started)

    def close(self):
        self.ground_done.set();self.ground_event.set();self.ground_thread.join(timeout=2)
        self.node.destroy_node();self.rclpy.shutdown()

    def location_update(self,message):
        with self.lock:self.location_quality=(int(message.data.total_status),time.monotonic())

    def match_update(self,message):
        if len(message.data)>=2:
            with self.lock:self.match_quality=(list(message.data[:2]),time.monotonic())

    def observation(self):
        """Full aligned scan plus synchronized localization; never the UI sample."""
        self.ensure_topic('/ALIGNED_POINTS')
        import base64
        import numpy as np
        from autonomy_sensing import xyz
        from map_fusion import voxel
        with self.lock:
            raw=self.aligned_raw;pose=copy.deepcopy(self.odom);loc=self.location_quality;quality=self.match_quality
        now=time.monotonic()
        if not raw or now-raw[1]>.6:raise ValueError('等待定位模式下的新鲜完整点云 /ALIGNED_POINTS')
        if not loc or now-loc[1]>1.5 or loc[0]!=getattr(self,'normal_location_status',1):raise ValueError(self.refinement_error or '定位尚未正常，暂停积累地图观测')
        if not quality or now-quality[1]>1.5 or not np.isfinite(quality[0]).all() or quality[0][0]>.25 or quality[0][1]<.85:
            raise ValueError('定位匹配质量不足，暂停积累地图观测')
        msg=raw[0];stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        lag=self.node.get_clock().now().nanoseconds/1e9-stamp
        if not -.1<=lag<=.6:raise ValueError('定位点云源时间过期')
        if not pose or now-pose['received_monotonic']>.5 or abs(pose['stamp']-stamp)>.15:raise ValueError('等待与定位点云同步的位姿')
        if msg.header.frame_id!='map' or pose['frame_id']!=msg.header.frame_id:raise ValueError('定位点云与位姿地图坐标系不一致')
        if pose['tilt']>.14:raise ValueError('当前机身倾斜，暂停平地地图更新')
        points=xyz(msg);center=np.array([pose['PosX'],pose['PosY'],pose['PosZ']])
        radius=np.linalg.norm(points[:,:2]-center[:2],axis=1);points=voxel(points[(radius>.55)&(radius<6)],.06)
        if not 100<=len(points)<=60000:raise ValueError('当前定位点云有效点数不足或过大')
        return dict(ok=True,protocol=1,stamp=stamp,age=max(now-raw[1],max(0,lag)),frame_id='map',
                    pose=center.tolist()+pose['quaternion'],points=base64.b64encode(points.astype('<f4').tobytes()).decode(),count=len(points),quality=quality[0])
