#!/usr/bin/env python3
import cv2, math, os, time, numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker, MarkerArray


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        params = {'camera_hfov':1.3962634, 'ball_radius':0.04,
                  'map_frame':'odom', 'camera_frame':'camera_link',
                  'marker_lifetime':0.5, 'capture_dir':'captures',
                  'capture_min_frames':3, 'block_min_area':400.0}
        for k,v in params.items():
            self.declare_parameter(k,v)
        p = lambda k: self.get_parameter(k).value
        self.hfov, self.r = float(p('camera_hfov')), float(p('ball_radius'))
        self.frame, self.cam = str(p('map_frame')), str(p('camera_frame'))
        self.life, self.n = float(p('marker_lifetime')), int(p('capture_min_frames'))
        self.area, self.path = float(p('block_min_area')), str(p('capture_dir'))
        os.makedirs(self.path, exist_ok=True)
        self.hits, self.saved = {'blue':0,'green':0}, set()
        self.kernel = np.ones((3,3), np.uint8)
        self.tf, self.listener = Buffer(), None
        self.listener = TransformListener(self.tf, self)

        self.create_subscription(Image, '/camera/image_raw', self.cb, qos_profile_sensor_data)
        self.pub = {k:self.create_publisher(PointStamped, v, 10) for k,v in {
            'center':'/vision/bbox_center', 'red':'/vision/red_ball_position',
            'green':'/vision/green_ball_position'}.items()}
        self.det = self.create_publisher(Bool, '/vision/detected', 10)
        self.img = self.create_publisher(Image, '/vision/image_debug', 10)
        self.markers = self.create_publisher(MarkerArray, '/vision/ball_markers', 10)

    def cb(self, msg):
        im = np.frombuffer(msg.data, np.uint8).reshape(msg.height,msg.width,-1)
        if msg.encoding.lower() == 'rgb8':
            im = cv2.cvtColor(im,cv2.COLOR_RGB2BGR)
        h,w = im.shape[:2]
        dbg, hsv = im.copy(), cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
        masks = {
            'red':cv2.inRange(hsv,(0,130,60),(8,255,255)) |
                  cv2.inRange(hsv,(172,130,60),(180,255,255)),
            'green':cv2.inRange(hsv,(35,70,50),(85,255,255)),
            'blue':cv2.inRange(hsv,(95,80,40),(140,255,255))}
        masks = {k:cv2.morphologyEx(v,cv2.MORPH_OPEN,self.kernel)
                 for k,v in masks.items()}

        def contours(mask, area):
            return [c for c in cv2.findContours(mask,cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)[0] if cv2.contourArea(c)>=area]

        blocks = {}
        for color in ('green','blue'):
            blocks[color] = []
            for c in contours(masks[color],self.area):
                a = cv2.approxPolyDP(c,.04*cv2.arcLength(c,True),True)
                x,y,bw,bh = cv2.boundingRect(a)
                if len(a)==4 and cv2.isContourConvex(a) and .2<=bw/max(bh,1)<=5:
                    blocks[color].append((x,y,bw,bh))
                    cv2.rectangle(dbg,(x,y),(x+bw,y+bh),
                                  (0,255,0) if color=='green' else (255,0,0),2)
            if color=='green':
                for c in contours(masks[color],self.area):
                    a=cv2.approxPolyDP(c,.04*cv2.arcLength(c,True),True)
                    x,y,bw,bh=cv2.boundingRect(a)
                    if len(a)==4 and cv2.isContourConvex(a) and .2<=bw/max(bh,1)<=5:
                        cv2.drawContours(masks[color],[c],-1,0,-1)

            if color not in self.saved and blocks[color]:
                self.hits[color]+=1
                if self.hits[color]>=self.n:
                    x,y,bw,bh=blocks[color][0]
                    crop=im[max(0,y-8):min(h,y+bh+8),
                            max(0,x-8):min(w,x+bw+8)]
                    if crop.size and cv2.imwrite(os.path.join(
                        self.path,f'{color}_{int(time.time()*1000)}.jpg'),crop):
                        self.saved.add(color)

        boxes = {}
        for color in ('red','green'):
            cs=contours(masks[color],60)
            bs=[cv2.boundingRect(c) for c in cs]
            boxes[color]=max(bs,key=lambda b:(b[1]+b[3],b[2]*b[3])) if bs else None

        detected=all(boxes.values())
        ma=MarkerArray()
        for i,color in enumerate(('red','green')):
            b=boxes[color]
            if not b: continue
            x,y,bw,bh=b
            fx=(w/2)/math.tan(self.hfov/2)
            z=fx*self.r/max((bw+bh)/4,1)
            pt=PointStamped()
            pt.header.stamp=self.get_clock().now().to_msg()
            pt.header.frame_id=self.cam
            pt.point.x=z
            pt.point.y=-((x+bw/2)-w/2)*z/fx
            pt.point.z=-((y+bh/2)-h/2)*z/fx
            try:
                pt=do_transform_point(pt,self.tf.lookup_transform(
                    self.frame,self.cam,Time()))
            except Exception:
                continue
            self.pub[color].publish(pt)
            for mid,typ in ((i,Marker.SPHERE),(i+10,Marker.TEXT_VIEW_FACING)):
                m=Marker()
                m.header.frame_id=self.frame
                m.header.stamp=self.get_clock().now().to_msg()
                m.ns,m.id,m.type='balls',mid,typ
                m.action=Marker.ADD
                m.lifetime=Duration(seconds=self.life).to_msg()
                m.pose.position=pt.point
                m.pose.orientation.w=1.
                m.color.r,m.color.g,m.color.b,m.color.a=(
                    (1.,.1,.1,1.) if color=='red' else (.1,.8,.1,1.))
                if typ==Marker.SPHERE:
                    m.scale.x=m.scale.y=m.scale.z=2*self.r
                else:
                    m.pose.position.z+=self.r+.05
                    m.scale.z=.08
                    m.text=color.upper()
                ma.markers.append(m)
            cv2.rectangle(dbg,(x,y),(x+bw,y+bh),
                          (0,0,255) if color=='red' else (0,255,0),2)

        if ma.markers: self.markers.publish(ma)
        if detected:
            centers=[(b[0]+b[2]//2,b[1]+b[3]//2) for b in boxes.values()]
            mx=sum(c[0] for c in centers)//2
            angle=-math.atan2(mx-w/2,(w/2)/math.tan(self.hfov/2))
            pt=PointStamped()
            pt.header=msg.header
            pt.point.x,pt.point.z=angle,(mx-w/2)/(w/2)
            self.pub['center'].publish(pt)
            text,bar=f'CENTER ERROR: {math.degrees(angle):+.1f} deg',(0,150,0)
        else:
            text,bar='SEARCHING RED & GREEN...',(0,100,220)

        self.det.publish(Bool(data=detected))
        cv2.rectangle(dbg,(10,10),(w-10,45),bar,-1)
        cv2.putText(dbg,text,(20,33),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),2)
        out=Image()
        out.header,out.height,out.width=msg.header,h,w
        out.encoding,out.step,out.data='bgr8',w*3,dbg.tobytes()
        self.img.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node=VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__':
    main()