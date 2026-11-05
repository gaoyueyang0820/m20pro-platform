#!/usr/bin/env python3
"""Explicitly activated ROS2 joint adapter; does not reset errors or enable drives.

This Python adapter is not a certified real-time controller. Hardware validation
on a supported firmware and a suspended rig is required before operational use.
"""
import argparse
import copy
import json
import math
import sys
import threading
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from m20.platform_core import JOINTS, integer, number
from bridge.http_agent import serve


class JointController:
    def __init__(self,calibration):
        if calibration.get('verified') is not True or calibration.get('sdk_mode_verified') is not True:
            raise ValueError('需完成本机校准核对与 SDK 模式核对；示例配置不能直接用于真机')
        if calibration.get('frame')!='urdf_radians': raise ValueError('仅支持 urdf_radians 校准坐标')
        rows=calibration.get('joints',[])
        if len(rows)!=16: raise ValueError('校准需包含 16 个关节')
        for i,j in enumerate(rows):
            if j.get('name')!=JOINTS[i]['name'] or j.get('direction') not in [-1,1]:
                raise ValueError('校准关节顺序或方向不匹配')
            number(j.get('offset_rad'),-15,15)
            number(j.get('minimum'),-10,10);number(j.get('maximum'),-10,10)
            if j['minimum']>=j['maximum']: raise ValueError('关节限位无效')
        self.cal=rows
        self.lock=threading.RLock()
        self.rx=0.
        self.imu_rx=0.
        self.last_stamp=None
        self.last_imu_stamp=None
        self.joints=[]
        self.session=None
        self.lease=0.
        self.trajectory=None
        self.target=None
        self.armed=False
        self.damping=False
        self.publisher_conflict=False

    def feedback(self,raw,stamp=None):
        if len(raw)!=16: return
        if stamp is not None and not any(stamp):return
        values=[]
        try:
            for i,r in enumerate(raw):
                c=self.cal[i]
                values.append(dict(position=number(r.position,-1000,1000)*c['direction']+c['offset_rad'],
                                   velocity=number(r.velocity,-100,100)*c['direction'],
                                   torque=number(r.torque,-200,200)*c['direction'],
                                   temperature=number(r.motion_temp,-40,150),
                                   driver_temperature=number(r.driver_temp,-40,150),
                                   status_word=int(r.status_word),fresh=True))
        except ValueError:
            with self.lock: self.armed=False;self.damping=True;self.session=None
            return
        with self.lock:
            self.joints=values
            if stamp is None or stamp!=self.last_stamp:
                self.rx=time.monotonic();self.last_stamp=stamp

    def imu_feedback(self,data,stamp=None):
        try:
            for key in ['roll','pitch','yaw','acc_x','acc_y','acc_z','omega_x','omega_y','omega_z']:
                number(getattr(data,key),-1e5,1e5)
        except (ValueError,AttributeError):
            self.stop();return
        if stamp is not None and not any(stamp):return
        with self.lock:
            if stamp is None or stamp!=self.last_imu_stamp:self.imu_rx=time.monotonic();self.last_imu_stamp=stamp

    def healthy(self):
        return (len(self.joints)==16 and all(j['status_word']==1 and j['temperature']<=100 for j in self.joints)
                and time.monotonic()-self.rx<.15 and time.monotonic()-self.imu_rx<.15)

    def stop(self):
        with self.lock:
            self.armed=False;self.session=None;self.trajectory=None;self.damping=True
        return dict(ok=True,message='本地驱动转阻尼，未复位驱动器错误')

    def status(self):
        with self.lock:
            return dict(ok=True,connected=True,backend='hardware',calibration_verified=True,
                        fresh=time.monotonic()-self.rx<.15,imu_fresh=time.monotonic()-self.imu_rx<.15,
                        healthy=self.healthy(),armed=self.armed,damping=self.damping,
                        publisher_conflict=self.publisher_conflict,joints=copy.deepcopy(self.joints),
                        feedback_age=round(time.monotonic()-self.rx,3) if self.rx else None)

    def dispatch(self,path,req):
        with self.lock:
            if path=='/status' and req is None: return self.status()
            if req is None: raise ValueError('接口不存在')
            if path=='/stop': return self.stop()
            if path=='/arm':
                if self.publisher_conflict: raise ValueError('检测到其他 /JOINTS_CMD 发布者，拒绝争抢控制权')
                if self.armed: raise ValueError('已有 SDK 控制会话，请先结束原会话')
                if not req.get('rig_confirmed'): raise ValueError('需确认吊架工况')
                if len(self.joints)!=16 or time.monotonic()-self.rx>=.15: raise ValueError('16 关节反馈缺失或过期')
                if not self.healthy():raise ValueError('关节驱动状态、温度或 IMU 反馈不满足开启条件')
                session=req.get('session')
                if not isinstance(session,str) or not 20<=len(session)<=100: raise ValueError('无效控制会话')
                for i,j in enumerate(self.joints):
                    if i%4!=3: number(j['position'],self.cal[i]['minimum'],self.cal[i]['maximum'])
                self.session=session;self.lease=time.monotonic();self.armed=True;self.damping=False
                self.target=[j['position'] for j in self.joints];self.trajectory=None
                return self.status()
            if not self.armed or req.get('session')!=self.session: raise ValueError('控制会话已过期')
            if time.monotonic()-self.rx>=.15: self.stop();raise ValueError('关节反馈过期，已转阻尼')
            if self.publisher_conflict: self.stop();raise ValueError('出现第二控制器，已转阻尼')
            self.lease=time.monotonic()
            if path=='/lease': return dict(ok=True)
            if path=='/trajectory':
                poses=req.get('positions',[]);wheels=req.get('wheels',[])
                if len(poses)!=16 or len(wheels)!=4: raise ValueError('目标维度必须为 16 关节、4 轮速')
                duration=number(req.get('duration'),1,8)
                origin=[j['position'] for j in self.joints]
                target=[]
                for i,value in enumerate(poses):
                    if i%4==3: target.append(origin[i]);continue
                    target.append(number(value,self.cal[i]['minimum'],self.cal[i]['maximum']))
                    delta=abs(target[i]-origin[i])
                    if delta>.8 or delta*1.5/duration>.5: raise ValueError('目标步长或插值峰值速度过大，请减小幅度或增加时长')
                wheels=[number(w,-2,2) for w in wheels]
                self.trajectory=dict(origin=origin,target=target,wheels=wheels,start=time.monotonic(),duration=duration)
                return dict(ok=True)
            raise ValueError('接口不存在')

    def command(self,now=None):
        with self.lock:
            now=time.monotonic() if now is None else now
            if self.armed and (now-self.lease>.5 or now-self.rx>.15 or self.publisher_conflict or not self.healthy()): self.stop()
            if not self.armed and not self.damping: return None
            # Never compete with a newly discovered controller: stop publishing.
            if self.publisher_conflict: return None
            out=[]
            blend=0
            if self.trajectory:
                t=min(1.,max(0.,(now-self.trajectory['start'])/self.trajectory['duration']))
                blend=t*t*(3-2*t)
            for i,c in enumerate(self.cal):
                position=0.;velocity=0.;kp=0.;kd=2.
                if self.armed:
                    p=self.target[i]
                    if self.trajectory:
                        p=self.trajectory['origin'][i]+(self.trajectory['target'][i]-self.trajectory['origin'][i])*blend
                    position=(p-c['offset_rad'])*c['direction']
                    kp=20. if i%4!=3 else 0.
                    kd=2. if i%4!=3 else .6
                    if i%4==3 and self.trajectory: velocity=self.trajectory['wheels'][i//4]*c['direction']
                out.append(dict(position=position,velocity=velocity,kp=kp,kd=kd,torque=0.,control_word=4))
            return out


def main():
    p=argparse.ArgumentParser(description='M20 机器人端 SDK 关节代理')
    p.add_argument('--hardware',action='store_true')
    p.add_argument('--sdk-authorized',action='store_true')
    p.add_argument('--rig-confirmed',action='store_true')
    p.add_argument('--calibration',required=True)
    p.add_argument('--port',type=int,default=8091)
    args=p.parse_args()
    if not all([args.hardware,args.sdk_authorized,args.rig_confirmed]):
        p.error('真机代理需显式 --hardware --sdk-authorized --rig-confirmed；请先取得厂商 SDK 模式授权并完成吊架检查')
    controller=JointController(json.loads(Path(args.calibration).read_text()))
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from drdds.msg import JointsData,JointsDataCmd,ImuData
    rclpy.init()
    node=Node('m20_platform_bridge')
    pub=node.create_publisher(JointsDataCmd,'/JOINTS_CMD',10)
    node.create_subscription(JointsData,'/JOINTS_DATA',lambda msg:controller.feedback(msg.data.joints_data,(msg.header.stamp.sec,msg.header.stamp.nanosec)),qos_profile_sensor_data)
    node.create_subscription(ImuData,'/IMU_DATA',lambda msg:controller.imu_feedback(msg.data,(msg.header.stamp.sec,msg.header.stamp.nanosec)),qos_profile_sensor_data)
    def output():
        rows=controller.command()
        if rows is None:return
        msg=JointsDataCmd()
        for i,row in enumerate(rows):
            for k,v in row.items(): setattr(msg.data.joints_data[i],k,v)
        pub.publish(msg)
    def owners():
        with controller.lock: controller.publisher_conflict=node.count_publishers('/JOINTS_CMD')>1
    node.create_timer(.005,output)
    node.create_timer(.2,owners)
    threading.Thread(target=lambda:rclpy.spin(node),daemon=True).start()
    try: serve(args.port,controller.dispatch)
    finally:
        controller.stop()
        time.sleep(.1)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
