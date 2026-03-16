import os
import math
import threading
import csv
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

BRAITENBERG_GAIN = 1.5

BASE_SPEED  = 0.18
GOAL_RADIUS = 0.45
DANGER_DIST = 0.8

GOALS = [
    ( 3.5,  3.5),
    (-3.5,  3.5),
    (-3.5, -3.5),
    ( 3.5, -3.5),
]

LOG_FILE = ('/ros2_ws/gain_log_' if __import__('os').path.exists('/ros2_ws') else '/home/masum/braitenberg_ws/gain_log_') + str(BRAITENBERG_GAIN) + '.csv'

class Braitenberg(Node):
    def __init__(self):
        super().__init__('braitenberg_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry,  '/odom', self.odom_cb, 10)
        self.create_timer(0.05, self.step)

        self.x = self.y = self.yaw = 0.0
        self.ranges = []
        self.rmin   = 0.12
        self.amin   = 0.0
        self.ainc   = 0.0175
        self.has_odom = False
        self.has_scan = False

        self.goal_idx      = 0
        self.all_done      = False
        self.goals_reached = []

        self.obstacle_detected = False
        self.obs_dist  = 3.5
        self.obs_angle = 0.0

        self.frame = None
        self.lock  = threading.Lock()
        threading.Thread(target=self._show, daemon=True).start()

        # open CSV log
        self.csvfile = open(LOG_FILE, 'w', newline='')
        self.writer  = csv.writer(self.csvfile)
        self.writer.writerow(['time', 'gain', 'v', 'w', 'obs_dist',
                              'obstacle', 'front', 'left', 'right'])
        self.t0 = time.time()
        self.get_logger().info(f'Braitenberg | GAIN={BRAITENBERG_GAIN} | logging to {LOG_FILE}')

    def odom_cb(self, msg):
        self.x   = msg.pose.pose.position.x
        self.y   = msg.pose.pose.position.y
        q        = msg.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        self.has_odom = True

    def scan_cb(self, msg):
        self.ranges = list(msg.ranges)
        self.rmin   = msg.range_min
        self.amin   = msg.angle_min
        self.ainc   = msg.angle_increment
        self.has_scan = True

    def ray(self, deg, hw=15):
        if not self.ranges:
            return 3.5
        n    = len(self.ranges)
        step = 360.0 / n
        ci   = int(deg / step) % n
        hw   = max(1, int(hw / step))
        vals = [self.ranges[i % n] for i in range(ci - hw, ci + hw + 1)
                if math.isfinite(self.ranges[i % n])
                and self.ranges[i % n] > self.rmin]
        return min(vals) if vals else 3.5

    @staticmethod
    def norm(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def stimulus(self, d):
        if d > DANGER_DIST:
            return 0.0
        return math.tanh(BRAITENBERG_GAIN / max(d, 0.12))

    def step(self):
        if not self.has_scan or not self.has_odom:
            return

        front = self.ray(0,   15)
        fl    = self.ray(40,  15)
        fr    = self.ray(320, 15)
        fl2   = self.ray(80,  15)
        fr2   = self.ray(280, 15)
        left  = self.ray(90,  15)
        right = self.ray(270, 15)

        if self.all_done:
            self.pub.publish(Twist())
            self._draw(0.0, 0.0)
            return

        gx, gy = GOALS[self.goal_idx]
        dist   = math.hypot(gx - self.x, gy - self.y)

        if dist < GOAL_RADIUS:
            self.goals_reached.append(self.goal_idx)
            self.goal_idx += 1
            if self.goal_idx >= len(GOALS):
                self.all_done = True
                self.pub.publish(Twist())
            return

        sf   = self.stimulus(front)
        sfl  = self.stimulus(fl)
        sfr  = self.stimulus(fr)
        sfl2 = self.stimulus(fl2)
        sfr2 = self.stimulus(fr2)
        sl   = self.stimulus(left)
        sr   = self.stimulus(right)

        obs_total = max(sf, sfl, sfr, sfl2, sfr2, sl, sr)
        self.obstacle_detected = obs_total > 0.1

        if self.ranges:
            min_r = 3.5
            min_i = 0
            for i, r in enumerate(self.ranges):
                if math.isfinite(r) and r > self.rmin and r < min_r:
                    min_r = r
                    min_i = i
            self.obs_dist  = min_r
            self.obs_angle = self.yaw + self.amin + min_i * self.ainc

        obs_turn = ((sfr + sfr2*0.5 + sr*0.3) - (sfl + sfl2*0.5 + sl*0.3)) * 3.0
        obs_slow = sf * 1.2 + max(sfl, sfr) * 0.5

        goal_angle  = math.atan2(gy - self.y, gx - self.x)
        heading_err = self.norm(goal_angle - self.yaw)

        if obs_total > 0.3:
            w = obs_turn
            v = BASE_SPEED * max(0.0, 1.0 - obs_slow)
        else:
            w = heading_err * 1.0 + obs_turn * 0.3
            v = BASE_SPEED * (1.0 - abs(heading_err) / math.pi * 0.4)

        if front < 0.25:
            v = 0.0
            w = 2.0 if left > right else -2.0

        t = Twist()
        t.linear.x  = float(np.clip(v, 0.0, BASE_SPEED))
        t.angular.z = float(np.clip(w, -2.2, 2.2))
        self.pub.publish(t)

        # log every step
        self.writer.writerow([
            round(time.time() - self.t0, 3),
            BRAITENBERG_GAIN,
            round(t.linear.x, 4),
            round(t.angular.z, 4),
            round(self.obs_dist, 3),
            int(self.obstacle_detected),
            round(front, 3),
            round(left, 3),
            round(right, 3),
        ])

        self.get_logger().info(
            f'[C{self.goal_idx+1}/4] GAIN={BRAITENBERG_GAIN} | '
            f'F:{front:.2f} L:{left:.2f} R:{right:.2f} | '
            f'obstacle:{"YES" if self.obstacle_detected else "NO"} | '
            f'dist:{dist:.2f}m | v:{t.linear.x:.2f} w:{t.angular.z:.2f}')

        self._draw(t.linear.x, t.angular.z)

    def _draw(self, v, w):
        S     = 700
        p     = np.full((S, S, 3), (20, 20, 35), dtype=np.uint8)
        cx    = S // 2
        cy    = S // 2 + 20
        scale = 30

        room = int(4.925 * scale)
        cv2.rectangle(p, (cx-room, cy-room), (cx+room, cy+room), (80,80,80), 2)

        for i, (gx, gy) in enumerate(GOALS):
            px  = int(cx + gx*scale)
            py  = int(cy - gy*scale)
            col = (60,60,60)   if i in self.goals_reached \
                else (0,255,0)   if i == self.goal_idx \
                else (0,100,200)
            cv2.circle(p, (px,py), 14, col, -1)
            cv2.putText(p, f'C{i+1}', (px-9, py+5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255,255,255), 1)

        rx = max(10, min(S-10, int(cx + self.x*scale)))
        ry = max(10, min(S-10, int(cy - self.y*scale)))

        if self.obstacle_detected:
            cv2.circle(p, (rx,ry), int(DANGER_DIST*scale), (0,0,200), 1)

        cv2.circle(p, (rx,ry), 12, (0,0,220), -1)
        cv2.arrowedLine(p, (rx,ry),
            (int(rx + 16*math.cos(self.yaw)),
             int(ry - 16*math.sin(self.yaw))),
            (0,255,255), 2, tipLength=0.4)

        if not self.all_done and self.goal_idx < len(GOALS):
            gx, gy = GOALS[self.goal_idx]
            cv2.line(p, (rx,ry),
                     (int(cx + gx*scale), int(cy - gy*scale)),
                     (0,180,0), 1)

        if self.obstacle_detected and self.obs_dist < DANGER_DIST:
            ox = int(rx + self.obs_dist*scale*math.cos(self.obs_angle))
            oy = int(ry - self.obs_dist*scale*math.sin(self.obs_angle))
            ox = max(0, min(S-1, ox))
            oy = max(0, min(S-1, oy))
            cv2.line(p, (rx,ry), (ox,oy), (0,0,255), 2)
            cv2.circle(p, (ox,oy), 8, (0,0,255), -1)
            cv2.putText(p, f'{self.obs_dist:.2f}m', (ox+5, oy-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0,100,255), 1)

        panel_y = S - 155
        cv2.line(p, (0,panel_y), (S,panel_y), (50,50,70), 1)

        if self.obstacle_detected:
            cv2.rectangle(p, (10,panel_y+10), (340,panel_y+55), (0,0,180), -1)
            cv2.putText(p, 'OBSTACLE DETECTED', (18, panel_y+42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,100,255), 2)
            cv2.putText(p, f'Distance: {self.obs_dist:.2f}m', (350, panel_y+42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0,100,255), 1)
        else:
            cv2.rectangle(p, (10,panel_y+10), (200,panel_y+55), (0,100,0), -1)
            cv2.putText(p, 'PATH CLEAR', (18, panel_y+42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,100), 2)

        goal_x, goal_y = GOALS[min(self.goal_idx, len(GOALS)-1)]
        dist   = math.hypot(goal_x - self.x, goal_y - self.y)
        status = 'ALL CORNERS DONE!' if self.all_done \
            else f'Navigating to Corner {self.goal_idx+1}/4  {GOALS[self.goal_idx]}'

        lines = [
            (status, (0,255,100) if self.all_done else (0,220,220)),
            (f'GAIN={BRAITENBERG_GAIN}   dist={dist:.2f}m   v={v:.2f}  w={w:.2f}', (255,220,0)),
            (f'pos ({self.x:.1f}, {self.y:.1f})   yaw={math.degrees(self.yaw):.0f} deg', (160,160,160)),
        ]
        for i, (txt, col) in enumerate(lines):
            cv2.putText(p, txt, (10, panel_y+75 + i*24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.46, col, 1, cv2.LINE_AA)

        with self.lock:
            self.frame = p

    def _show(self):
        cv2.namedWindow('Braitenberg Monitor', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Braitenberg Monitor', 720, 720)
        while rclpy.ok():
            with self.lock:
                f = self.frame
            if f is not None:
                cv2.imshow('Braitenberg Monitor', f)
            if cv2.waitKey(50) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = Braitenberg()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.csvfile.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
