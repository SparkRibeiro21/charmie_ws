#!/usr/bin/env python3
import math
import threading
import rclpy
from rclpy.node import Node

import pygame
from pathlib import Path

from charmie_interfaces.msg import SDNLDebug


class SdnlDebugViewer(Node):
    def __init__(self):
        super().__init__("sdnl_debug_viewer")

        self.topic = self.declare_parameter("topic_sdnl_debug", "sdnl/debug").value
        self.width = int(self.declare_parameter("width", 1100).value)
        self.height = int(self.declare_parameter("height", 650).value)
        self.fps = float(self.declare_parameter("fps", 60.0).value)

        self.sub = self.create_subscription(SDNLDebug, self.topic, self.cb, 10)

        self._lock = threading.Lock()
        self._last_msg = None

        self.get_logger().info(f"SDNL Debug Viewer listening on '{self.topic}'")

    def cb(self, msg: SDNLDebug):
        with self._lock:
            self._last_msg = msg

    def get_last(self):
        with self._lock:
            return self._last_msg


def draw_curve_fixed(surface, rect, y, color, y_lim):
    """Draw curve y[] in rect with fixed symmetric limits [-y_lim, +y_lim] and zero centered."""
    if not y or len(y) < 2:
        return
    n = len(y)
    if y_lim <= 1e-9:
        y_lim = 1.0

    # Mapping
    def map_x(i):
        return rect.left + int(i * (rect.width - 1) / (n - 1))

    def map_y(val):
        # val in [-y_lim, +y_lim] => y in [bottom..top]
        v = max(-y_lim, min(y_lim, val))
        norm = (v + y_lim) / (2.0 * y_lim)  # 0..1
        return rect.bottom - int(norm * (rect.height - 1))

    # Zero line always centered
    y0 = map_y(0.0)
    pygame.draw.line(surface, (60, 60, 60), (rect.left, y0), (rect.right, y0), 1)

    pts = [(map_x(i), map_y(y[i])) for i in range(n)]
    pygame.draw.lines(surface, color, False, pts, 2)

def recompute_layout(screen, margin=20):
    w, h = screen.get_size()
    plot_w = w - 2 * margin
    plot_h = (h - 4 * margin) // 3

    rect1 = pygame.Rect(margin, margin, plot_w, plot_h)
    rect2 = pygame.Rect(margin, 2 * margin + plot_h, plot_w, plot_h)
    rect3 = pygame.Rect(margin, 3 * margin + 2 * plot_h, plot_w, plot_h)
    return w, h, margin, rect1, rect2, rect3

def main():
    rclpy.init()
    node = SdnlDebugViewer()

    pygame.init()
    screen = pygame.display.set_mode((node.width, node.height), pygame.RESIZABLE)
    node.width, node.height = screen.get_size()

    home = str(Path.home())
    logo_midpath = "/charmie_ws/src/configuration_files/docs/logos/"
    icon = pygame.image.load(home+logo_midpath+"logo_light_cropped_squared.png")
    pygame.display.set_icon(icon)
    pygame.display.set_caption("CHARMIE Nav SDNL Debug")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 18)

    # Layout: 3 stacked plots on the right side (like your screenshot)
    y_lim_top = 15.0
    y_lim_mid = 15.0
    y_lim_bot = 25.0
    step = 2.0

    w, h, margin, rect1, rect2, rect3 = recompute_layout(screen)
    node.width, node.height = w, h

    running = True
    while running:
        # ROS spin once (non-blocking)
        rclpy.spin_once(node, timeout_sec=0.0)

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                mult = 5.0 if (pygame.key.get_mods() & pygame.KMOD_SHIFT) else 1.0
                inc = step * mult

                if event.key == pygame.K_q:
                    y_lim_top += inc
                elif event.key == pygame.K_a:
                    y_lim_top = max(1.0, y_lim_top - inc)

                elif event.key == pygame.K_w:
                    y_lim_mid += inc
                elif event.key == pygame.K_s:
                    y_lim_mid = max(1.0, y_lim_mid - inc)

                elif event.key == pygame.K_e:
                    y_lim_bot += inc
                elif event.key == pygame.K_d:
                    y_lim_bot = max(1.0, y_lim_bot - inc)

                elif event.key == pygame.K_r:
                    y_lim_top = 15.0
                    y_lim_mid = 15.0
                    y_lim_bot = 25.0

        # redefine tamanhos da janela
        w, h = screen.get_size()
        if w != node.width or h != node.height:    
            w, h, margin, rect1, rect2, rect3 = recompute_layout(screen)
            node.width, node.height = w, h

        screen.fill((10, 10, 10))

        # Draw plot frames
        for r in (rect1, rect2, rect3):
            pygame.draw.rect(screen, (230, 230, 230), r, 2)

        msg = node.get_last()

        if msg is None or msg.n_samples == 0:
            txt = font.render("Waiting for /sdnl/debug ...", True, (200, 200, 200))
            screen.blit(txt, (margin, node.height - margin - 20))
        else:
            y_att = list(msg.y_attractor)
            y_rep = list(msg.y_rep_sum)
            y_fin = list(msg.y_final)

            # Fixed symmetric scaling around zero (manual, per-plot)
            draw_curve_fixed(screen, rect1, y_att, (0, 255, 0), y_lim_top)
            draw_curve_fixed(screen, rect2, y_rep, (0, 200, 255), y_lim_mid)

            # Final plot: draw att + rep + final
            # Final plot: all curves share the same limits (so the sum is visually correct)
            draw_curve_fixed(screen, rect3, y_att, (80, 80, 80), y_lim_bot)
            draw_curve_fixed(screen, rect3, y_rep, (120, 120, 120), y_lim_bot)
            draw_curve_fixed(screen, rect3, y_fin, (255, 255, 0), y_lim_bot)

            # Vertical "heading" line: use psi_target_base or robot yaw mapping
            # For now: use psi_target_base mapped into [0, 2pi)
            psi = float(msg.psi_target_base)
            while psi < 0:
                psi += 2.0 * math.pi
            while psi >= 2.0 * math.pi:
                psi -= 2.0 * math.pi

            x_line = rect1.left + int((psi / (2.0 * math.pi)) * rect1.width)
            for r in (rect1, rect2, rect3):
                pygame.draw.line(screen, (255, 60, 60), (x_line, r.top), (x_line, r.bottom), 2)

            # Labels
            screen.blit(font.render("Attractor (0..2π)", True, (230, 230, 230)), (rect1.left + 8, rect1.top + 6))
            screen.blit(font.render("Repulsor sum (0..2π)", True, (230, 230, 230)), (rect2.left + 8, rect2.top + 6))
            screen.blit(font.render("Final = Att + Rep (0..2π)", True, (230, 230, 230)), (rect3.left + 8, rect3.top + 6))

            # Show current limits
            screen.blit(font.render(f"±{y_lim_top:.1f}", True, (180, 180, 180)), (rect1.right - 90, rect1.top + 6))
            screen.blit(font.render(f"±{y_lim_mid:.1f}", True, (180, 180, 180)), (rect2.right - 90, rect2.top + 6))
            screen.blit(font.render(f"±{y_lim_bot:.1f}", True, (180, 180, 180)), (rect3.right - 90, rect3.top + 6))

            # Help text (keyboard shortcuts)
            help_txt = "Top: Q/A  Mid: W/S  Bot: E/D  Shift=faster  R=reset"
            screen.blit(font.render(help_txt, True, (160, 160, 160)), (margin, node.height - margin - 45))

            # Status line
            status = f"N={msg.n_samples} dθ={msg.dtheta:.4f}  dist={msg.dist_to_target:.2f}  min_obs={msg.min_obstacle_dist_edge:.2f}"
            screen.blit(font.render(status, True, (200, 200, 200)), (margin, node.height - margin - 20))

        pygame.display.flip()
        clock.tick(node.fps)

    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
