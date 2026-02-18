#!/usr/bin/env python3
import threading
import math
import rclpy
from rclpy.node import Node

import pygame

from charmie_interfaces.msg import SdnlDebug

class SdnlDebugViewer(Node):
    def __init__(self):
        super().__init__("sdnl_debug_viewer")
        self.sub = self.create_subscription(SdnlDebug, "sdnl/debug", self.cb, 10)
        self.last = None

    def cb(self, msg: SdnlDebug):
        self.last = msg

def ros_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = SdnlDebugViewer()

    t = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    t.start()

    pygame.init()
    screen = pygame.display.set_mode((900, 600))
    clock = pygame.time.Clock()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((15, 15, 15))

        msg = node.last
        if msg and msg.n_samples > 0:
            # aqui vais desenhar os 3 gráficos usando msg.y_attractor, msg.y_rep_sum, msg.y_final
            # e uma linha vertical para heading/psi_target etc.
            pass

        pygame.display.flip()
        clock.tick(60)

    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == "__main__":
    main()
