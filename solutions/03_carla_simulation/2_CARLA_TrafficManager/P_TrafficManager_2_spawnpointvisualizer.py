#!/usr/bin/env python3
#======================================================================#
# Module:      NGV_TrafficManager_1_spawnpointvisualizer.py
# Description: Visualize spawn points using pygame
#
# Authors: Seokhwan Jeong (shjeong00@hanyang.ac.kr)
#
# Revision History
#      Feb  18, 2026: Seokhwan Jeong - Created.
#======================================================================#

import carla
import pygame

W, H = 1600, 950
MARGIN = 30
WAYPOINT_SPACING_M = 3.0
DOT_SIZE = 5
DOT_COLOR = (137, 207, 240)
BG_COLOR = (0, 0, 0)

SPAWN_COLOR = (255, 200, 0)
SPAWN_RADIUS = 4

def make_world_to_pixel(bounds, w=W, h=H, margin=MARGIN):
    min_x, max_x, min_y, max_y = bounds
    sx = (w - 2 * margin) / max(1e-6, (max_x - min_x))
    sy = (h - 2 * margin) / max(1e-6, (max_y - min_y))
    scale = min(sx, sy)

    def f(loc: carla.Location):
        x = (loc.x - min_x) * scale + margin
        y = (loc.y - min_y) * scale + margin 
        return int(x), int(y)

    return f

def compute_bounds_from_locations(locs):
    xs = [p.x for p in locs]
    ys = [p.y for p in locs]
    return (min(xs), max(xs), min(ys), max(ys))

def draw_text_outline(surf, font, text, pos, fg=(255,255,255), outline=(0,0,0)):
    x, y = pos
    for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,1),(-1,1),(1,-1)]:
        surf.blit(font.render(text, True, outline), (x+dx, y+dy))
    surf.blit(font.render(text, True, fg), (x, y))

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    original_settings = world.get_settings()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    m = world.get_map()

    all_wps = m.generate_waypoints(WAYPOINT_SPACING_M)
    driving_wps = [wp for wp in all_wps if wp.lane_type == carla.LaneType.Driving]

    spawn_points = m.get_spawn_points()

    for i, spawn_point in enumerate(spawn_points):
        world.debug.draw_string(spawn_point.location, str(i), life_time=50)

    locs_for_bounds = [wp.transform.location for wp in driving_wps] + [sp.location for sp in spawn_points]
    bounds = compute_bounds_from_locations(locs_for_bounds)
    world_to_px = make_world_to_pixel(bounds)

    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("CARLA: Thick Lane Dots + Spawn Points (2D)")
    clock = pygame.time.Clock()

    font_small = pygame.font.SysFont("DejaVu Sans", 16, bold=True)

    base = pygame.Surface((W, H))
    base.fill(BG_COLOR)

    half = DOT_SIZE // 2
    for wp in driving_wps:
        x, y = world_to_px(wp.transform.location)
        rx = x - half
        ry = y - half
        if 0 <= rx < W and 0 <= ry < H:
            base.fill(DOT_COLOR, (rx, ry, DOT_SIZE, DOT_SIZE))

    show_spawn_labels = True
    running = True

    try:
        while running:
            world.tick()

            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False
                elif e.type == pygame.KEYDOWN:
                    if e.key == pygame.K_ESCAPE:
                        running = False
                    elif e.key == pygame.K_s:
                        show_spawn_labels = not show_spawn_labels

            screen.blit(base, (0, 0))
            for i, sp in enumerate(spawn_points):
                p = world_to_px(sp.location)
                pygame.draw.circle(screen, SPAWN_COLOR, p, SPAWN_RADIUS)
                if show_spawn_labels:
                    draw_text_outline(screen, font_small, str(i), (p[0] + 6, p[1] - 10), fg=(230,230,230))

            hud = f"LaneDots: spacing={WAYPOINT_SPACING_M}m  dot={DOT_SIZE}px  |  S labels  |  ESC quit"
            draw_text_outline(screen, font_small, hud, (10, 10), fg=(240,240,240))

            pygame.display.flip()
            clock.tick(30)

    finally:
        try:
            world.apply_settings(original_settings)
        except Exception:
            pass
        pygame.quit()

if __name__ == "__main__":
    main()
