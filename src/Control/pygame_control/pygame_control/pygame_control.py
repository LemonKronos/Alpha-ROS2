#!/usr/bin/env python3
"""
pygame_control.py

ROS2 node that runs Pygame virtual joystick UI and publishes ros2_msgs::msg::ControlInterface messages to topic "control/input"
"""
import math
import time
import pygame
import rclpy
from rclpy.node import Node
from python_utils.utils import *

# Adjust this import path to match your message package
from ros2_msgs.msg import ControlInterface


MODULE_TEST = False

# Constants
HUMAN = 0
AI = 1
OUTER = 2

UNCHANGE = 0
MULTICOPTER = 1
FIXWINGS = 2

def clamp(val, low = -1, high = 1):
    if val < low:
        return low
    elif val > high:
        return high
    else:
        return val

class PygameControlNode:
    def __init__(self, node: Node):
        self.node = node

        setup_for_simulation(self.node)

        # Movement / tuning params
        self.WASD_SPEED = 10.0
        self.WASD_ACCELERATION = 5.0
        self.YAW_SPEED = math.pi
        self.YAW_ACCELERATION = 4.0
        self.UP_DOWN_SPEED = 3.0
        self.UP_DOWN_ACCELERATION = 5.0

        self.WASD_COASTING = 0.05
        self.YAW_COASTING = 0.5
        self.UP_DOWN_COASTING = 0.05

        # Publisher
        self.pub = self.node.create_publisher(ControlInterface, CONTROL_REACTIVE_TOPIC, 10)

        # UI control internals 
        self.is_armed = None  # not used for ROS publish but kept as attribute

    def run(self):
        # Pygame UI initialization (kept mostly unchanged)
        pygame.init()
        sr_width = 400
        sr_height = 400
        screen = pygame.display.set_mode((sr_width, sr_height))
        pygame.display.set_caption("Virtual Joystick")

        # Joystick parameters
        ORIGIN = [sr_width // 2, sr_height // 2]
        joystick_radius = 120  # Max distance from origin
        dot_radius = 20
        dot_pos = ORIGIN.copy()

        # Horizontal slider (above joystick)
        hslider_width = 180
        hslider_height = 20
        hslider_origin = [ORIGIN[0], ORIGIN[1] - joystick_radius - 50]
        hslider_rect = pygame.Rect(
            hslider_origin[0] - hslider_width // 2,
            hslider_origin[1] - hslider_height // 2,
            hslider_width,
            hslider_height,
        )
        hslider_square_size = 24
        hslider_min_x = hslider_rect.left
        hslider_max_x = hslider_rect.right - hslider_square_size

        # Vertical slider (right of joystick)
        vslider_width = 20
        vslider_height = 180
        vslider_origin = [ORIGIN[0] + joystick_radius + 50, ORIGIN[1]]
        vslider_rect = pygame.Rect(
            vslider_origin[0] - vslider_width // 2,
            vslider_origin[1] - vslider_height // 2,
            vslider_width,
            vslider_height,
        )
        vslider_square_size = 24
        vslider_min_y = vslider_rect.top
        vslider_max_y = vslider_rect.bottom - vslider_square_size

        # Simulated drone indicators (start at center)
        drone_joy_pos = ORIGIN.copy()
        drone_slider1_x = hslider_origin[0] - hslider_square_size // 2
        drone_slider2_y = vslider_origin[1] - vslider_square_size // 2

        last_cmd_time = 0.03
        cmd_interval = 0

        running = True
        clock = pygame.time.Clock()

        # Main loop
        while running:
            # Let ROS2 process any callbacks
            rclpy.spin_once(self.node, timeout_sec=0.0)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            keys = pygame.key.get_pressed()

            # Joystick input (WASD keys)
            offset_x = 0
            offset_y = 0
            if keys[pygame.K_w] and not keys[pygame.K_s]:
                offset_y = -joystick_radius
            elif keys[pygame.K_s] and not keys[pygame.K_w]:
                offset_y = joystick_radius
            if keys[pygame.K_e] and not keys[pygame.K_q]:
                offset_x = joystick_radius
            elif keys[pygame.K_q] and not keys[pygame.K_e]:
                offset_x = -joystick_radius
            # Joystick position
            if offset_x != 0 or offset_y != 0:
                length = (offset_x ** 2 + offset_y ** 2) ** 0.5
                if length > joystick_radius:
                    scale = joystick_radius / length
                    offset_x *= scale
                    offset_y *= scale
                dot_pos[0] = ORIGIN[0] + offset_x
                dot_pos[1] = ORIGIN[1] + offset_y
            else:
                dot_pos[0] = ORIGIN[0]
                dot_pos[1] = ORIGIN[1]
            # Joystick indicator smoothing
            dx = dot_pos[0] - drone_joy_pos[0]
            dy = dot_pos[1] - drone_joy_pos[1]
            dist = (dx ** 2 + dy ** 2) ** 0.5
            if dist > 1:
                if dot_pos != ORIGIN:
                    step = min(self.WASD_ACCELERATION, dist)
                else:
                    step = min(self.WASD_COASTING * dist, dist)
                drone_joy_pos[0] += dx / dist * step
                drone_joy_pos[1] += dy / dist * step
            else:
                drone_joy_pos[0], drone_joy_pos[1] = dot_pos[0], dot_pos[1]

            # Horizontal slider (A and D keys in original code used differently;
            # the original used a/d for slider — preserving mapping from your script)
            # Note: in the original you used a/d for hslider (yaw). Keep same.
            if keys[pygame.K_a] and not keys[pygame.K_d]:
                hslider_square_x = hslider_min_x
            elif keys[pygame.K_d] and not keys[pygame.K_a]:
                hslider_square_x = hslider_max_x
            else:
                hslider_square_x = hslider_origin[0] - hslider_square_size // 2
            # Horizontal indicator smoothing
            hslider_target_x = hslider_square_x
            hslider_center_x = hslider_origin[0] - hslider_square_size // 2
            hslider_dx = hslider_target_x - drone_slider1_x
            if abs(hslider_dx) > 1:
                if hslider_target_x != hslider_center_x:
                    step = min(self.YAW_ACCELERATION, abs(hslider_dx))
                else:
                    step = min(self.YAW_COASTING * abs(hslider_dx), abs(hslider_dx))
                drone_slider1_x += (hslider_dx / abs(hslider_dx)) * step
            else:
                drone_slider1_x = hslider_target_x

            # Vertical slider (Space and Shift keys)
            if keys[pygame.K_LSHIFT] and not keys[pygame.K_SPACE]:
                vslider_square_y = vslider_max_y
            elif keys[pygame.K_SPACE] and not keys[pygame.K_LSHIFT]:
                vslider_square_y = vslider_min_y
            else:
                vslider_square_y = vslider_origin[1] - vslider_square_size // 2
            # Vertical indicator smoothing
            vslider_target_y = vslider_square_y
            vslider_center_y = vslider_origin[1] - vslider_square_size // 2
            vslider_dy = vslider_target_y - drone_slider2_y
            if abs(vslider_dy) > 1:
                if vslider_target_y != vslider_center_y:
                    step = min(self.UP_DOWN_ACCELERATION, abs(vslider_dy))
                else:
                    step = min(self.UP_DOWN_COASTING * abs(vslider_dy), abs(vslider_dy))
                drone_slider2_y += (vslider_dy / abs(vslider_dy)) * step
            else:
                drone_slider2_y = vslider_target_y

            # Prepare normalized slider values
            now = time.time()
            hslider_value = (drone_slider1_x + hslider_square_size // 2) - hslider_origin[0]
            hslider_norm = 0.0
            denom_h = ((hslider_rect.width - hslider_square_size) // 2)
            if denom_h != 0:
                hslider_norm = hslider_value / denom_h

            vslider_value = vslider_origin[1] - (drone_slider2_y + vslider_square_size // 2)
            vslider_norm = 0.0
            denom_v = ((vslider_rect.height - vslider_square_size) // 2)
            if denom_v != 0:
                vslider_norm = vslider_value / denom_v

            # Publish at cmd_interval
            if now - last_cmd_time > cmd_interval:
                last_cmd_time = now

                # Compute velocities like original code
                forward_m_s = - (drone_joy_pos[1] - ORIGIN[1]) / joystick_radius
                if (forward_m_s > 0 and offset_y > 0) or (forward_m_s < 0 and offset_y < 0): forward_m_s = 0;

                right_m_s = (drone_joy_pos[0] - ORIGIN[0]) / joystick_radius
                if (right_m_s > 0 and offset_x < 0) or (right_m_s < 0 and offset_x > 0): right_m_s = 0

                down_m_s = - (vslider_norm)
                # if(down_m_s > 0 and keys[pygame.K_SPACE]) or (down_m_s < 0 and keys[pygame.K_LSHIFT]): down_m_s = 0

                yaw_rad_s = (hslider_norm)
                if(yaw_rad_s > 0 and keys[pygame.K_a]) or (yaw_rad_s < 0 and keys[pygame.K_d]): yaw_rad_s = 0

                # Build message
                msg = ControlInterface()
                # header
                try:
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                except Exception:
                    # In case header isn't present / different in your message, ignore
                    pass

                # which controller
                msg.control_by = ControlInterface.OPERATOR
                msg.control_state = True

                # We'll set them to 0.0 here. If you prefer mapping, change below.
                msg.roll = 0.0  # rad/s
                # msg.roll = float(clamp(right_m_s))
                msg.pitch = 0.0  # rad/s
                msg.yaw = -float(clamp(yaw_rad_s)) # frame FLU temporary fix

                # Linear velocities (m/s)
                msg.forward = float(clamp(forward_m_s))
                msg.left = -float(clamp(right_m_s)) # frame FLU temporary fix
                msg.up = -float(clamp(down_m_s)) # frame FLU temporary fix

                # wings mode left unchanged by UI
                msg.wings_mode = ControlInterface.UNCHANGE

                # Publish
                if MODULE_TEST or False:
                    self.node.get_logger().info(
                        f"[TEST] publish ControlInterface: forward={msg.forward:.2f} left={msg.left:.2f} up={msg.up:.2f} yaw={msg.yaw:.2f}"
                    )
                self.pub.publish(msg)

            # Drawing (kept from original)
            screen.fill((30, 30, 30))
            pygame.draw.circle(screen, (80, 80, 80), ORIGIN, joystick_radius, 2)
            pygame.draw.circle(screen, (200, 200, 200), ORIGIN, 4)
            pygame.draw.circle(screen, (150, 50, 50), (int(drone_joy_pos[0]), int(drone_joy_pos[1])), dot_radius)
            pygame.draw.circle(screen, (255, 0, 0), (int(dot_pos[0]), int(dot_pos[1])), dot_radius)
            pygame.draw.line(screen, (255, 0, 0), ORIGIN, (int(dot_pos[0]), int(dot_pos[1])), 4)

            # Horizontal slider visuals
            pygame.draw.rect(screen, (80, 80, 80), hslider_rect, 2)
            pygame.draw.rect(screen, (150, 150, 50), (
                int(drone_slider1_x), hslider_rect.centery - hslider_square_size // 2,
                hslider_square_size, hslider_square_size
            ))
            pygame.draw.rect(screen, (255, 255, 0), (
                int(hslider_square_x), hslider_rect.centery - hslider_square_size // 2,
                hslider_square_size, hslider_square_size
            ))

            # Vertical slider visuals
            pygame.draw.rect(screen, (80, 80, 80), vslider_rect, 2)
            pygame.draw.rect(screen, (50, 50, 150), (
                vslider_rect.centerx - vslider_square_size // 2, int(drone_slider2_y),
                vslider_square_size, vslider_square_size
            ))
            pygame.draw.rect(screen, (0, 0, 255), (
                vslider_rect.centerx - vslider_square_size // 2, int(vslider_square_y),
                vslider_square_size, vslider_square_size
            ))

            pygame.display.update()
            clock.tick(40)

        # Cleanup
        pygame.quit()
        # Shut down ROS from here if you created the node in this script's main()
        try:
            self.node.get_logger().info("Pygame UI exited, shutting down node.")
        except Exception:
            pass


def main():
    rclpy.init()
    node = Node("pygame_control_node")
    pm = PygameControlNode(node)
    try:
        pm.run()
    except KeyboardInterrupt:
        pass  # Silence Ctrl+C
    finally:
        node.destroy_node()


if __name__ == "__main__":
    MODULE_TEST = False
    main()
