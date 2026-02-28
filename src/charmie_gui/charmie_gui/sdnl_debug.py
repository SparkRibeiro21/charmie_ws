#!/usr/bin/env python3
import math
import threading
from collections import deque

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
        self.height = int(self.declare_parameter("height", 720).value)  # slightly taller (4 plots)
        self.fps = float(self.declare_parameter("fps", 60.0).value)

        # How many points to keep for time-series (speed plot)
        self.history_len = int(self.declare_parameter("history_len", 600).value)  # ~10s at 60 fps (approx)

        self.sub = self.create_subscription(SDNLDebug, self.topic, self.cb, 10)

        self._lock = threading.Lock()
        self._last_msg = None

        # Histories for speed + factors
        self.v_cmd_hist = deque(maxlen=self.history_len)
        self.v_max_hist = deque(maxlen=self.history_len)
        self.s_target_hist = deque(maxlen=self.history_len)
        self.s_obs_hist = deque(maxlen=self.history_len)
        self.s_turn_hist = deque(maxlen=self.history_len)

        self.get_logger().info(f"SDNL Debug Viewer listening on '{self.topic}'")

    def cb(self, msg: SDNLDebug):
        with self._lock:
            self._last_msg = msg

            # Pull new fields safely (in case you run with an older msg once)
            v_cmd = float(getattr(msg, "v_cmd", 0.0))
            v_max = float(getattr(msg, "v_max", 0.0))
            s_target = float(getattr(msg, "s_target", 1.0))
            s_obs = float(getattr(msg, "s_obs", 1.0))
            s_turn = float(getattr(msg, "s_turn", 1.0))

            self.v_cmd_hist.append(v_cmd)
            self.v_max_hist.append(v_max)
            self.s_target_hist.append(s_target)
            self.s_obs_hist.append(s_obs)
            self.s_turn_hist.append(s_turn)

    def get_last(self):
        with self._lock:
            return self._last_msg


def draw_curve_fixed(surface, rect, y, color, y_lim, width=2):
    """Draw curve y[] in rect with fixed symmetric limits [-y_lim, +y_lim] and zero centered."""
    if not y or len(y) < 2:
        return
    n = len(y)
    if y_lim <= 1e-9:
        y_lim = 1.0

    def map_x(i):
        return rect.left + int(i * (rect.width - 1) / (n - 1))

    def map_y(val):
        v = max(-y_lim, min(y_lim, val))
        norm = (v + y_lim) / (2.0 * y_lim)  # 0..1
        return rect.bottom - int(norm * (rect.height - 1))

    # Zero line
    y0 = map_y(0.0)
    pygame.draw.line(surface, (60, 60, 60), (rect.left, y0), (rect.right, y0), 1)

    pts = [(map_x(i), map_y(y[i])) for i in range(n)]
    pygame.draw.lines(surface, color, False, pts, width)


def draw_curve_0_to_max(surface, rect, y, color, y_max, width=2):
    """Draw curve y[] in rect with vertical limits [0, y_max]."""
    if not y or len(y) < 2:
        return
    n = len(y)
    y_max = max(1e-6, y_max)

    def map_x(i):
        return rect.left + int(i * (rect.width - 1) / (n - 1))

    def map_y(val):
        v = max(0.0, min(y_max, val))
        norm = v / y_max  # 0..1
        return rect.bottom - int(norm * (rect.height - 1))

    # baseline (0)
    pygame.draw.line(surface, (60, 60, 60), (rect.left, rect.bottom), (rect.right, rect.bottom), 1)

    pts = [(map_x(i), map_y(y[i])) for i in range(n)]
    pygame.draw.lines(surface, color, False, pts, width)


def draw_hline_0_to_max(surface, rect, y_value, y_max, color, width=1):
    """Draw a horizontal line at y_value (in [0, y_max]) on a 0..y_max plot."""
    y_max = max(1e-6, y_max)
    y_value = max(0.0, min(y_max, float(y_value)))
    y = rect.bottom - int((y_value / y_max) * (rect.height - 1))
    pygame.draw.line(surface, color, (rect.left, y), (rect.right, y), width)

def fill_under_curve_by_limiter(surface, rect,
                                v_hist, vmax_hist, y_max,
                                s_target_hist, s_obs_hist, s_turn_hist,
                                col_target, col_obs, col_turn,
                                col_blue=(80, 80, 255),
                                blue_thresh=0.95):

    if not v_hist or len(v_hist) < 2:
        return

    n = min(len(v_hist), len(vmax_hist),
            len(s_target_hist), len(s_obs_hist), len(s_turn_hist))
    if n < 2:
        return

    v_hist = v_hist[-n:]
    vmax_hist = vmax_hist[-n:]
    s_target_hist = s_target_hist[-n:]
    s_obs_hist = s_obs_hist[-n:]
    s_turn_hist = s_turn_hist[-n:]

    y_max = max(1e-6, float(y_max))

    def map_x(i):
        return rect.left + int(i * (rect.width - 1) / (n - 1))

    def map_y(val):
        v = max(0.0, min(y_max, float(val)))
        norm = v / y_max
        return rect.bottom - int(norm * (rect.height - 1))

    for i in range(n - 1):
        x0 = map_x(i)
        x1 = map_x(i + 1)
        if x1 < x0:
            x0, x1 = x1, x0
        w = (x1 - x0) + 1  # <- garante sem “buracos”

        v_cmd = float(v_hist[i])
        v_max_i = max(1e-9, float(vmax_hist[i]))

        st = float(s_target_hist[i])
        so = float(s_obs_hist[i])
        sr = float(s_turn_hist[i])

        # cor pelo limitador (min s_*)
        m = st
        col = col_target
        if so < m:
            m = so
            col = col_obs
        if sr < m:
            m = sr
            col = col_turn

        # override: quase vmax -> azul
        if v_cmd >= blue_thresh * v_max_i:
            col = col_blue

        y_top = map_y(v_cmd)
        h = rect.bottom - y_top
        if h <= 0:
            continue

        pygame.draw.rect(surface, col, pygame.Rect(x0, y_top, w, h))

def recompute_layout(screen, margin=20):
    w, h = screen.get_size()
    plot_w = w - 2 * margin

    # 4 stacked plots
    plot_h = (h - 5 * margin) // 4

    rect1 = pygame.Rect(margin, margin, plot_w, plot_h)
    rect2 = pygame.Rect(margin, 2 * margin + 1 * plot_h, plot_w, plot_h)
    rect3 = pygame.Rect(margin, 3 * margin + 2 * plot_h, plot_w, plot_h)
    rect4 = pygame.Rect(margin, 4 * margin + 3 * plot_h, plot_w, plot_h)
    return w, h, margin, rect1, rect2, rect3, rect4


def main():
    rclpy.init()
    node = SdnlDebugViewer()

    pygame.init()
    screen = pygame.display.set_mode((node.width, node.height), pygame.RESIZABLE)
    node.width, node.height = screen.get_size()

    home = str(Path.home())
    logo_midpath = "/charmie_ws/src/configuration_files/docs/logos/"
    icon = pygame.image.load(home + logo_midpath + "logo_light_cropped_squared.png")
    pygame.display.set_icon(icon)
    pygame.display.set_caption("CHARMIE Nav SDNL Debug")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 18)

    # Curve limits (same as before)
    y_lim_top = 15.0
    y_lim_mid = 15.0
    y_lim_bot = 25.0
    step = 2.0

    w, h, margin, rect1, rect2, rect3, rect4 = recompute_layout(screen)
    node.width, node.height = w, h

    running = True
    while running:
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

        # Window resize
        w, h = screen.get_size()
        if w != node.width or h != node.height:
            w, h, margin, rect1, rect2, rect3, rect4 = recompute_layout(screen)
            node.width, node.height = w, h

        screen.fill((10, 10, 10))

        # Plot frames
        for r in (rect1, rect2, rect3, rect4):
            pygame.draw.rect(screen, (230, 230, 230), r, 2)

        msg = node.get_last()

        if msg is None or msg.n_samples == 0:
            txt = font.render("Waiting for /sdnl/debug ...", True, (200, 200, 200))
            screen.blit(txt, (margin, node.height - margin - 20))
        else:
            y_att = list(msg.y_attractor)
            y_rep = list(msg.y_rep_sum)
            y_fin = list(msg.y_final)

            # Plot 1: attractor
            draw_curve_fixed(screen, rect1, y_att, (0, 255, 0), y_lim_top)

            status = (
                f"target_dist={msg.dist_to_target:.2f}  "
            )
            screen.blit(font.render(status, True, (230, 230, 230)), (rect1.left + 8, rect1.bottom - 24))

            # Plot 2: individual repulsors + sum
            n_samples = int(msg.n_samples)
            n_rep = int(getattr(msg, "n_repulsors", 0))
            y_rep_all = list(getattr(msg, "y_rep_all", []))

            if n_rep > 0 and len(y_rep_all) >= n_rep * n_samples:
                for k in range(n_rep):
                    start = k * n_samples
                    end = start + n_samples
                    yk = y_rep_all[start:end]
                    draw_curve_fixed(screen, rect2, yk, (90, 90, 90), y_lim_mid, width=1)

            draw_curve_fixed(screen, rect2, y_rep, (255, 60, 60), y_lim_mid, width=2)

            # --- Obstacle info on Plot 2 ---
            min_obs = float(getattr(msg, "min_obstacle_dist_edge", float("inf")))
            rep_n = int(getattr(msg, "n_repulsors", 0))
            if math.isfinite(min_obs):
                obs_txt = f"min_obs_edge={min_obs:.2f} m   repulsors={rep_n}"
            else:
                obs_txt = f"min_obs_edge=inf m    repulsors={rep_n}"
            screen.blit(font.render(obs_txt, True, (230, 230, 230)), (rect2.left + 8, rect2.bottom - 24))

            # Plot 3: final
            draw_curve_fixed(screen, rect3, y_att, (80, 80, 80), y_lim_bot)
            draw_curve_fixed(screen, rect3, y_rep, (120, 120, 120), y_lim_bot)
            draw_curve_fixed(screen, rect3, y_fin, (255, 255, 0), y_lim_bot, width=2)

            # Vertical line = robot heading in MAP
            psi = float(msg.robot_map.theta)
            psi = (psi + math.pi) % (2.0 * math.pi) - math.pi
            x_line = rect1.left + int(((psi + math.pi) / (2.0 * math.pi)) * rect1.width)
            for r in (rect1, rect2, rect3):
                pygame.draw.line(screen, (0, 200, 255), (x_line, r.top), (x_line, r.bottom), 2)

            # Plot 4: speed history
            v_hist = list(node.v_cmd_hist)
            vmax_hist = list(node.v_max_hist)

            vmax_plot = max(vmax_hist) if len(vmax_hist) > 0 else 0.5
            vmax_plot = max(0.05, vmax_plot)

            fill_under_curve_by_limiter(
                screen, rect4,
                v_hist, vmax_hist, vmax_plot,
                list(node.s_target_hist), list(node.s_obs_hist), list(node.s_turn_hist),
                col_target=(0, 255, 0),       # igual ao attractor (verde)
                col_obs=(255, 60, 60),        # igual ao repulsor sum (vermelho)
                col_turn=(255, 255, 0),       # igual ao final (amarelo)
                col_blue=(0, 200, 255),       # igual ao v_cmd line
                blue_thresh=0.95
            )

            # Draw v_cmd on top
            draw_curve_0_to_max(screen, rect4, v_hist, (0, 200, 255), vmax_plot, width=3)

            # --- Draw s_* influence lines as speed caps: s_* * v_max ---
            if len(vmax_hist) > 0:
                v_max_now = max(0.0, float(vmax_hist[-1]))

                s_target_now = float(node.s_target_hist[-1]) if len(node.s_target_hist) > 0 else 1.0
                s_obs_now    = float(node.s_obs_hist[-1])    if len(node.s_obs_hist) > 0 else 1.0
                s_turn_now   = float(node.s_turn_hist[-1])   if len(node.s_turn_hist) > 0 else 1.0

                v_cap_target = s_target_now * v_max_now
                v_cap_obs    = s_obs_now    * v_max_now
                v_cap_turn   = s_turn_now   * v_max_now

                # draw 3 horizontal cap lines
                draw_hline_0_to_max(screen, rect4, v_cap_target, vmax_plot, (100, 255, 100), width=2)  # green-ish
                draw_hline_0_to_max(screen, rect4, v_cap_obs,    vmax_plot, (255, 120, 120), width=2)  # red-ish
                draw_hline_0_to_max(screen, rect4, v_cap_turn,   vmax_plot, (255, 255, 120), width=2)  # yellow-ish

            # Draw a reference line at current v_max (last) if available
            """ if len(vmax_hist) > 0:
                vref = max(0.0, float(vmax_hist[-1]))
                vref = min(vref, vmax_plot)
                yref = rect4.bottom - int((vref / vmax_plot) * (rect4.height - 1))
                pygame.draw.line(screen, (120, 120, 255), (rect4.left, yref), (rect4.right, yref), 1) """
            
            # Labels
            screen.blit(font.render("Attractor [-π, π]", True, (230, 230, 230)), (rect1.left + 8, rect1.top + 6))
            screen.blit(font.render("Repulsor sum [-π, π]", True, (230, 230, 230)), (rect2.left + 8, rect2.top + 6))
            screen.blit(font.render("Final = Att + Rep [-π, π]", True, (230, 230, 230)), (rect3.left + 8, rect3.top + 6))
            screen.blit(font.render("Speed [m/s]", True, (230, 230, 230)), (rect4.left + 8, rect4.top + 6))

            # Limits
            screen.blit(font.render(f"(Q/A)  ±{y_lim_top:.1f}", True, (180, 180, 180)), (rect1.right - 150, rect1.top + 6))
            screen.blit(font.render(f"(W/S)  ±{y_lim_mid:.1f}", True, (180, 180, 180)), (rect2.right - 150, rect2.top + 6))
            screen.blit(font.render(f"(E/D)  ±{y_lim_bot:.1f}", True, (180, 180, 180)), (rect3.right - 150, rect3.top + 6))
            screen.blit(font.render(f"        {vmax_plot:.2f}", True, (180, 180, 180)), (rect4.right - 150, rect4.top + 6))

            # Show limiter info (who is smallest)
            v_cmd = float(getattr(msg, "v_cmd", 0.0))
            v_max = float(getattr(msg, "v_max", 0.0))
            s_target = float(getattr(msg, "s_target", 1.0))
            s_obs = float(getattr(msg, "s_obs", 1.0))
            s_turn = float(getattr(msg, "s_turn", 1.0))

            factors = {
                "target": s_target,
                "obs": s_obs,
                "turn": s_turn
            }
            limiting_name = min(factors, key=factors.get)
            limiting_val = factors[limiting_name]

            # print data below the speed plot, colored by who is limiting 
            base_x = margin
            base_y = rect4.bottom + 0

            # --- v_cmd + v_max (neutral color)
            part1 = font.render(f"v_cmd={v_cmd:.3f} v_max={v_max:.2f} | ", True, (200, 200, 200))
            screen.blit(part1, (base_x, base_y))
            base_x += part1.get_width()

            # --- s_tar (green - same as attractor)
            part2 = font.render(f"s_tar={s_target:.2f} ", True, (0, 255, 0))
            screen.blit(part2, (base_x, base_y))
            base_x += part2.get_width()

            # --- s_obs (red - same as repulsor)
            part3 = font.render(f"s_obs={s_obs:.2f} ", True, (255, 60, 60))
            screen.blit(part3, (base_x, base_y))
            base_x += part3.get_width()

            # --- s_rot (yellow - same as final)
            part4 = font.render(f"s_rot={s_turn:.2f}", True, (255, 255, 0))
            screen.blit(part4, (base_x, base_y))

            # Status line
            status = (
                f"N={msg.n_samples} dθ={msg.dtheta:.4f}  "
                f"Shift=faster  R=reset"
            )
            screen.blit(font.render(status, True, (230, 230, 230)), (margin, 0))

        pygame.display.flip()
        clock.tick(node.fps)

    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()