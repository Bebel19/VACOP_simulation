import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from pynput import keyboard
import threading
import time

msg = """
=========================================
CONTROLE DU ROBOT VACOP (Mode Smooth)
=========================================
Commandes :
   Z : Avancer
   S : Reculer
   Q : Gauche
   D : Droite
   
   Espace : Frein d'urgence
   CTRL+C : Quitter
=========================================
"""

class VacopTeleop(Node):
    def __init__(self):
        super().__init__('vacop_teleop')
        
        self.propulsion_pub = self.create_publisher(
            Float64MultiArray, '/propulsion_controller/commands', 10)
        self.steering_pub = self.create_publisher(
            Float64MultiArray, '/steering_controller/commands', 10)


        self.MAX_SPEED = 20.0
        self.MAX_STEER = 0.39
        

        self.ACCEL_SPEED = 2.0
        self.ACCEL_STEER = 0.1
        
        self.DECEL_FACTOR = 1.5



        self.target_speed = 0.0
        self.target_steer = 0.0


        self.current_speed = 0.0
        self.current_steer = 0.0
        

        self.timer = self.create_timer(0.02, self.control_loop)
        
        print(msg)

    def smooth_value(self, current, target, accel):

        if current < target:
            return min(target, current + accel)
        elif current > target:
            return max(target, current - accel)
        return target

    def control_loop(self):

        

        speed_step = self.ACCEL_SPEED if self.target_speed != 0 else(self.ACCEL_SPEED * self.DECEL_FACTOR)
        steer_step = self.ACCEL_STEER if self.target_steer != 0 else (self.ACCEL_STEER * self.DECEL_FACTOR)

        self.current_speed = self.smooth_value(self.current_speed, self.target_speed, speed_step)
        self.current_steer = self.smooth_value(self.current_steer, self.target_steer, steer_step)


        prop_msg = Float64MultiArray()
        prop_msg.data = [self.current_speed] * 4
        self.propulsion_pub.publish(prop_msg)

        steer_msg = Float64MultiArray()
        steer_msg.data = [self.current_steer, self.current_steer]
        self.steering_pub.publish(steer_msg)

    def on_press(self, key):
        try:
            char = key.char.lower()
            if char == 'z': self.target_speed = self.MAX_SPEED
            elif char == 's': self.target_speed = -self.MAX_SPEED
            if char == 'q': self.target_steer = self.MAX_STEER
            elif char == 'd': self.target_steer = -self.MAX_STEER
        except AttributeError:
            if key == keyboard.Key.space:
                self.target_speed = 0.0
                self.target_steer = 0.0
                self.current_speed = 0.0

    def on_release(self, key):
        try:
            char = key.char.lower()
            if char in ['z', 's']: self.target_speed = 0.0
            if char in ['q', 'd']: self.target_steer = 0.0
        except AttributeError:
            pass

def main(args=None):
    rclpy.init(args=args)
    teleop_node = VacopTeleop()
    listener = keyboard.Listener(on_press=teleop_node.on_press, on_release=teleop_node.on_release)
    listener.start()

    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.target_speed = 0.0
        teleop_node.control_loop() 
        listener.stop()
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()