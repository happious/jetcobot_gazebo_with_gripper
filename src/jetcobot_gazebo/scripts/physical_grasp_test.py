#!/usr/bin/env python3
"""One-box physical grasp/lift/lower/release verification. No physics attachment.

Success requires actual Gazebo box motion, bilateral finger contacts and a
stable suspended hold; a successful gripper action alone is insufficient.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import random
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTolerance
from moveit_msgs.action import MoveGroup,ExecuteTrajectory
from moveit_msgs.srv import GetPositionIK,GetCartesianPath,ApplyPlanningScene,GetPlanningScene
from moveit_msgs.msg import Constraints,JointConstraint,CollisionObject,AllowedCollisionEntry,PlanningSceneComponents
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer,TransformListener
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts

ARM=[f'{i}_Joint' for i in range(1,7)]


def down_pose(x,y,z,yaw):
    p=Pose();p.position.x=float(x);p.position.y=float(y);p.position.z=float(z)
    p.orientation.x=math.cos(yaw/2);p.orientation.y=math.sin(yaw/2)
    p.orientation.z=0.;p.orientation.w=0.
    return p


class PhysicalGraspTest(Node):
    def __init__(self):
        super().__init__('physical_grasp_test',parameter_overrides=[Parameter('use_sim_time',value=True)])
        for k,v in {'world':'grasp_test','box':'box_A','grip_z':.040,'lift_m':.08,
                    'box_size':[.07,.03,.04],'report':'/tmp/physical_grasp_result.json','speed':.1}.items():self.declare_parameter(k,v)
        self.world=self.get_parameter('world').value;self.box=self.get_parameter('box').value
        self.joints=None;self.box_pose=None;self.pose_received=0.;self.contacts={'left':{},'right':{}}
        self.lock=threading.Lock();self.rng=random.Random(10);self.records=[]
        self.create_subscription(JointState,'/joint_states',self.on_joints,20)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.ik=self.create_client(GetPositionIK,'/compute_ik');self.cart=self.create_client(GetCartesianPath,'/compute_cartesian_path')
        self.scene=self.create_client(ApplyPlanningScene,'/apply_planning_scene');self.get_scene=self.create_client(GetPlanningScene,'/get_planning_scene')
        self.move=ActionClient(self,MoveGroup,'/move_action');self.execute=ActionClient(self,ExecuteTrajectory,'/execute_trajectory')
        self.gripper=ActionClient(self,FollowJointTrajectory,'/gripper_group_controller/follow_joint_trajectory')
        self.gz=GzNode()
        self.gz.subscribe(Pose_V,f'/world/{self.world}/pose/info',self.on_pose)
        self.gz.subscribe(Pose_V,f'/world/{self.world}/dynamic_pose/info',self.on_pose)
        for side in ('left','right'):
            self.gz.subscribe(Contacts,f'/jetcobot/contacts/{side}',lambda msg,side=side:self.on_contacts(side,msg))
            self.gz.subscribe(Contacts,f'/world/{self.world}/model/jetcobot/link/gripper_{side}1/sensor/{side}_finger_contact/contact',lambda msg,side=side:self.on_contacts(side,msg))

    def on_joints(self,msg):
        if all(n in msg.name for n in ARM):self.joints=msg

    def on_pose(self,msg):
        for p in msg.pose:
            if p.name==self.box:
                with self.lock:
                    self.box_pose={'x':p.position.x,'y':p.position.y,'z':p.position.z,
                                   'q':[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]}
                    self.pose_received=time.monotonic()

    def on_contacts(self,side,msg):
        hits=[]
        for c in msg.contact:
            if self.box in (c.collision1.name+c.collision2.name):
                hits.append({'a':c.collision1.name,'b':c.collision2.name,'max_depth_m':max(c.depth,default=0.),'depth_count':len(c.depth),
                             'centroid_m':[sum(getattr(p,k) for p in c.position)/len(c.position) for k in ('x','y','z')] if c.position else None})
        with self.lock:self.contacts[side]={'time':time.monotonic(),'hits':hits}

    def until(self,predicate,timeout=15):
        end=time.monotonic()+timeout
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.02)
            if predicate():return
        raise TimeoutError('Simulation state/measurement timeout')

    def pause(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.03)

    def wait(self,future,timeout=45):
        rclpy.spin_until_future_complete(self,future,timeout_sec=timeout)
        if not future.done():raise TimeoutError('ROS request timed out')
        return future.result()

    def call(self,client,request):
        if not client.wait_for_service(timeout_sec=30):raise RuntimeError('MoveIt service missing')
        return self.wait(client.call_async(request))

    def action(self,client,goal,gripper=False):
        if not client.wait_for_server(timeout_sec=30):raise RuntimeError('Action server missing')
        handle=self.wait(client.send_goal_async(goal))
        if not handle.accepted:raise RuntimeError('Action rejected')
        try:r=self.wait(handle.get_result_async(),90)
        except TimeoutError:
            self.wait(handle.cancel_goal_async(),5);raise
        code=r.result.error_code if gripper else r.result.error_code.val
        if r.status!=4 or code!=(0 if gripper else 1):raise RuntimeError(f'Action failed status={r.status}, code={code}')
        return r

    def sample(self,stage):
        with self.lock:
            if not self.box_pose or time.monotonic()-self.pose_received>1:raise RuntimeError('Gazebo box pose is stale')
            data={'stage':stage,'box':deepcopy(self.box_pose),'contacts':deepcopy(self.contacts)}
        angles=dict(zip(self.joints.name,self.joints.position))
        data['finger_q']=angles.get('gripper_controller')
        data['finger_angles']={name:q for name,q in angles.items() if name.startswith('gripper')}
        data['sim_time_s']=self.get_clock().now().nanoseconds/1e9
        t=self.tf.lookup_transform('world','TCP',rclpy.time.Time()).transform.translation
        data['tcp']=[t.x,t.y,t.z];self.records.append(data)
        self.get_logger().info(f"{stage}: box=({data['box']['x']:.4f},{data['box']['y']:.4f},{data['box']['z']:.4f}) q={data['finger_q']:.4f}")
        return data

    def bilateral(self):
        with self.lock:
            return all(self.contacts[s].get('hits') and time.monotonic()-self.contacts[s].get('time',0)<.5 for s in ('left','right'))

    def any_contact(self):
        with self.lock:
            # Bullet also publishes separated proximity pairs (negative depth).
            return any(any(h['max_depth_m']>0 for h in self.contacts[s].get('hits',[])) and time.monotonic()-self.contacts[s].get('time',0)<.5 for s in ('left','right'))

    def collision_object(self,name,xyz,size):
        c=CollisionObject();c.header.frame_id='world';c.id=name;c.operation=CollisionObject.ADD
        primitive=SolidPrimitive();primitive.type=SolidPrimitive.BOX;primitive.dimensions=list(size)
        p=Pose();p.position.x,p.position.y,p.position.z=map(float,xyz);p.orientation.w=1.
        c.primitives=[primitive];c.primitive_poses=[p];return c

    def setup_scene(self,xyz):
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True
        req.scene.world.collision_objects=[self.collision_object('floor',[0,0,-.025],[2.,2.,.05]),
            self.collision_object('pedestal',[0,0,.05],[.1,.1,.1]),self.collision_object(self.box,xyz,list(self.get_parameter('box_size').value))]
        query=GetPlanningScene.Request();query.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix=self.call(self.get_scene,query).scene.allowed_collision_matrix
        # Planning permits intended fingertip contact only. Gazebo contacts and
        # friction remain active; no physics collision masks are changed.
        for name in ('gripper_left1','gripper_right1',self.box):
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for row in matrix.entry_values:row.enabled.append(False)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
        for finger in ('gripper_left1','gripper_right1'):
            i=matrix.entry_names.index(finger);j=matrix.entry_names.index(self.box)
            matrix.entry_values[i].enabled[j]=True;matrix.entry_values[j].enabled[i]=True
        req.scene.allowed_collision_matrix=matrix
        if not self.call(self.scene,req).success:raise RuntimeError('Planning scene update failed')

    def gripper_to(self,q,contact=False):
        self.get_logger().info(f'Gripper target {q:.3f} rad (contact allowed={contact})')
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=['gripper_controller']
        p=JointTrajectoryPoint();p.positions=[q];p.velocities=[0.];p.time_from_start.sec=3
        goal.trajectory.points=[p]
        if contact:
            # A contacting box prevents reaching the empty-hand closing angle.
            # Continue holding the final target/effort; success here is NOT the
            # grasp test. Actual contacts and lifted box height decide success.
            goal.goal_tolerance=[JointTolerance(name='gripper_controller',position=.65,velocity=1.)]
            goal.path_tolerance=[JointTolerance(name='gripper_controller',position=.65,velocity=2.)]
        else:
            goal.goal_tolerance=[JointTolerance(name='gripper_controller',position=.005,velocity=.1)]
        self.action(self.gripper,goal,True);self.pause(.7)

    def cart_request(self,start,target):
        req=GetCartesianPath.Request();req.header.frame_id='world';req.group_name='arm_group';req.link_name='TCP'
        req.start_state=start;req.start_state.is_diff=True;req.waypoints=[target];req.max_step=.002
        req.avoid_collisions=True;req.revolute_jump_threshold=.25
        req.max_velocity_scaling_factor=self.get_parameter('speed').value;req.max_acceleration_scaling_factor=.1
        return req

    def approach(self,high,low):
        self.get_logger().info('Planning approach and full vertical descent')
        solution=None
        for attempt in range(50):
            req=GetPositionIK.Request();ik=req.ik_request;ik.group_name='arm_group';ik.ik_link_name='TCP'
            ik.pose_stamped.header.frame_id='world';ik.pose_stamped.pose=high;ik.avoid_collisions=True;ik.timeout.nanosec=200000000
            ik.robot_state.joint_state=deepcopy(self.joints);ik.robot_state.is_diff=True
            if attempt:
                ranges=[3.13,2.89,2.29,2.59,2.49,2.83]
                ik.robot_state.joint_state.position=[self.rng.uniform(-ranges[ARM.index(n)],ranges[ARM.index(n)]) if n in ARM else v for n,v in zip(self.joints.name,self.joints.position)]
            out=self.call(self.ik,req)
            if out.error_code.val!=1:continue
            path=self.call(self.cart,self.cart_request(out.solution,low))
            if path.fraction>=.999:solution=out.solution.joint_state;break
        if solution is None:raise RuntimeError('No full collision-free approach/descent')
        goal=MoveGroup.Goal();r=goal.request;r.group_name='arm_group';r.start_state.is_diff=True
        r.allowed_planning_time=8.;r.num_planning_attempts=5;r.max_velocity_scaling_factor=.1;r.max_acceleration_scaling_factor=.1
        c=Constraints()
        c.joint_constraints=[JointConstraint(joint_name=n,position=v,tolerance_above=.003,tolerance_below=.003,weight=1.) for n,v in zip(solution.name,solution.position) if n in ARM]
        r.goal_constraints=[c];self.action(self.move,goal);self.settle(high)

    def settle(self,target):
        def reached():
            try:t=self.tf.lookup_transform('world','TCP',rclpy.time.Time()).transform
            except Exception:return False
            d=sum((getattr(t.translation,k)-getattr(target.position,k))**2 for k in ('x','y','z'))**.5
            dot=abs(sum(getattr(t.rotation,k)*getattr(target.orientation,k) for k in ('x','y','z','w')))
            return d<.004 and dot>.9995
        self.until(reached,12);self.pause(.3)

    def vertical(self,target,label):
        self.get_logger().info(label)
        from moveit_msgs.msg import RobotState
        state=RobotState();state.joint_state=self.joints;state.is_diff=True
        out=self.call(self.cart,self.cart_request(state,target))
        if out.error_code.val!=1 or out.fraction<.999:raise RuntimeError(f'Incomplete vertical path: {out.fraction:.3f}')
        goal=ExecuteTrajectory.Goal();goal.trajectory=out.solution;self.action(self.execute,goal);self.settle(target)

    def run(self):
        self.until(lambda:self.joints is not None and self.box_pose is not None and self.tf.can_transform('world','TCP',rclpy.time.Time()),30)
        initial=self.sample('initial');xyz=[initial['box'][k] for k in ('x','y','z')]
        self.setup_scene(xyz);self.gripper_to(.145)
        z=self.get_parameter('grip_z').value
        selected=None
        for yaw in (math.pi/2,-math.pi/2):
            low=down_pose(xyz[0],xyz[1],z,yaw);high=deepcopy(low);high.position.z+=.12
            try:self.approach(high,low);selected=(low,high);break
            except RuntimeError as e:
                if 'No full collision-free' not in str(e):raise
        if selected is None:raise RuntimeError('Neither short-side grasp orientation is reachable')
        low,high=selected;self.vertical(low,'Descend with gripper open');self.sample('at_grasp_open')
        self.gripper_to(-.45,True);self.sample('closed')
        try:self.until(self.bilateral,3)
        except TimeoutError:raise RuntimeError('Bilateral finger contact was not established; refusing to lift')
        lifted=deepcopy(low);lifted.position.z+=self.get_parameter('lift_m').value
        self.vertical(lifted,'Lift using physical finger contact only')
        samples=[]
        for i in range(5):
            if i:self.pause(.5)
            samples.append(self.sample(f'hold_{i}'))
        if min(s['box']['z'] for s in samples)<xyz[2]+.06:raise RuntimeError('Box did not remain at least 60mm above its initial height')
        if max(s['box']['z'] for s in samples)-min(s['box']['z'] for s in samples)>.01:raise RuntimeError('Box slipped during hold')
        if not self.bilateral():raise RuntimeError('Bilateral contact lost during hold')
        validation_errors=[]
        signs={'gripper_base_to_gripper_left2':1,'gripper_base_to_gripper_right2':-1,
               'gripper_base_to_gripper_right3':-1,'gripper_left3_to_gripper_left1':-1,
               'gripper_right3_to_gripper_right1':1}
        for s in samples:
            if any(abs(s['finger_angles'][name]-sign*s['finger_q'])>.03 for name,sign in signs.items()):
                validation_errors.append('Mimic joint error exceeds 0.03rad under load')
            xq,yq,zq,wq=s['box']['q']
            tilt=math.acos(max(-1.,min(1.,1-2*(xq*xq+yq*yq))))
            if tilt>math.radians(10):validation_errors.append(f'Box tilt during hold exceeds 10deg: {math.degrees(tilt):.1f}')
        depths=[hit['max_depth_m'] for s in samples for side in ('left','right')
                for hit in s['contacts'][side].get('hits',[])]
        if not depths:raise RuntimeError('No contact penetration measurements available')
        if any(h.get('depth_count',0)==0 for sample in samples for contact in sample['contacts'].values() for h in contact.get('hits',[])):raise RuntimeError('Missing depth data in contact measurement')
        if max(depths)>.003:validation_errors.append('Excessive finger/object penetration (>3mm)')
        self.vertical(low,'Lower to original surface');self.sample('lowered')
        self.gripper_to(.145);self.sample('released')
        def release_ready():
            with self.lock:pose=deepcopy(self.box_pose)
            qx,qy,_,_=pose['q']
            tilt=math.acos(max(-1.,min(1.,1-2*(qx*qx+qy*qy))))
            return not self.any_contact() and abs(pose['z']-xyz[2])<.0005 and tilt<math.radians(1)
        release_bias=[0.,0.]
        for release_attempt in range(8):
            if release_attempt>0 and release_ready():break
            # Near maximum jaw width, a few mm of slip / yaw can keep a box
            # wedged in an open hand. Recenter the real hand over the measured
            # box on the floor; never move or attach the object itself.
            with self.lock:
                placed=deepcopy(self.box_pose);touch=deepcopy(self.contacts)
            # Balance the two measured fingertip clearances. The nominal TCP
            # is approximate: repeatedly commanding just the box center can
            # leave one pad touching while the other has spare clearance.
            lh=touch['left'].get('hits',[]);rh=touch['right'].get('hits',[])
            if lh and rh:
                left=max(lh,key=lambda h:h['max_depth_m']);right=max(rh,key=lambda h:h['max_depth_m'])
                if left['centroid_m'] and right['centroid_m']:
                    axis=[left['centroid_m'][k]-right['centroid_m'][k] for k in (0,1)]
                    length=math.hypot(*axis)
                    correction=max(-.0005,min(.0005,(left['max_depth_m']-right['max_depth_m'])/2))
                    if length>.001:
                        release_bias=[max(-.002,min(.002,release_bias[k]+correction*axis[k]/length)) for k in (0,1)]
            qx,qy,qz,qw=placed['q']
            yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
            aligned=deepcopy(low);aligned.position.x=placed['x']+release_bias[0];aligned.position.y=placed['y']+release_bias[1]
            a=math.sin(yaw/2);b=math.cos(yaw/2);q=low.orientation
            aligned.orientation.x=b*q.x-a*q.y;aligned.orientation.y=a*q.x+b*q.y
            aligned.orientation.z=b*q.z+a*q.w;aligned.orientation.w=b*q.w-a*q.z
            self.vertical(aligned,'Recenter open hand to release measured box')
            self.pause(.5);self.sample(f'release_aligned_{release_attempt}')
            retract_height=high.position.z-low.position.z
            high=deepcopy(aligned);high.position.z+=retract_height
        try:self.until(release_ready,3)
        except TimeoutError:raise RuntimeError('Box has not settled flat and clear of the fingertips; refusing to retract')
        self.vertical(high,'Retract open gripper')
        self.pause(1.);final=self.sample('final')
        if abs(final['box']['z']-xyz[2])>.004:raise RuntimeError('Released box did not settle on the floor')
        if math.hypot(final['box']['x']-xyz[0],final['box']['y']-xyz[1])>.015:raise RuntimeError('Box moved too far from its original location')
        if validation_errors:raise RuntimeError('; '.join(dict.fromkeys(validation_errors)))
        return {'success':True,'physics_attachment':False,'box_size_m':list(self.get_parameter('box_size').value),'measurements':self.records}


def main():
    rclpy.init();node=PhysicalGraspTest();report=None
    try:
        report=node.run();node.get_logger().info('PHYSICAL GRASP / LIFT / HOLD / LOWER / RELEASE: PASS')
    except Exception as exc:
        node.get_logger().error(str(exc));report={'success':False,'physics_attachment':False,'error':str(exc),'box_size_m':list(node.get_parameter('box_size').value),'measurements':node.records}
    finally:
        Path(node.get_parameter('report').value).write_text(json.dumps(report,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    if not report['success']:raise SystemExit(1)


if __name__=='__main__':main()
