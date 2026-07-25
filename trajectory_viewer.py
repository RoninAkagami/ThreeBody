import argparse
import math
from pathlib import Path

import pygame
from PIL import Image


G = 0.5
DT = 0.25
UPDATES_PER_FRAME = 2
SPEED_MULTIPLIER = 1
MAX_DISTANCE = 5000.0

DEFAULT_TRAIL_FRAMES = 2200
MIN_TRAIL_FRAMES = 200
MAX_TRAIL_FRAMES = 20000
MAX_TRAIL_MARKERS = 10


def parse_params_from_name(path: Path):
    stem = path.stem
    parts = stem.split("c")
    if len(parts) != 5:
        raise ValueError(
            f"filename must look like y2cvx2cvy2cvx3cvy3.png, got: {path.name}"
        )
    try:
        y2, vx2, vy2, vx3, vy3 = (float(part) for part in parts)
    except ValueError as exc:
        raise ValueError(f"filename contains non-numeric parameters: {path.name}") from exc
    return {
        "y2": y2,
        "vx2": vx2,
        "vy2": vy2,
        "vx3": vx3,
        "vy3": vy3,
    }


def choose_default_image():
    candidates = sorted(Path.cwd().glob("*c*c*c*c*.png"))
    if candidates:
        return candidates[0]
    fallback = Path("sim_output_percentile_inverted.png")
    if fallback.exists():
        return fallback
    raise FileNotFoundError("no parameter PNG found in current directory")


def total_distance(bodies):
    d1 = math.hypot(bodies[0][0] - bodies[1][0], bodies[0][1] - bodies[1][1])
    d2 = math.hypot(bodies[1][0] - bodies[2][0], bodies[1][1] - bodies[2][1])
    d3 = math.hypot(bodies[2][0] - bodies[0][0], bodies[2][1] - bodies[0][1])
    return d1 + d2 + d3


def compute_forces(bodies):
    for i in range(3):
        fx = 0.0
        fy = 0.0
        for j in range(3):
            if i == j:
                continue
            dx = bodies[j][0] - bodies[i][0]
            dy = bodies[j][1] - bodies[i][1]
            r = math.hypot(dx, dy)
            if r < 1.0:
                r = 1.0
            force = G * 1250.0 * 1250.0 / (r * r)
            fx += force * dx / r
            fy += force * dy / r
        bodies[i][2] += fx / 1250.0 * DT
        bodies[i][3] += fy / 1250.0 * DT


def update_positions(bodies):
    for body in bodies:
        body[0] += body[2] * DT
        body[1] += body[3] * DT


def simulate_trajectory(x3, y3, params, frame_limit):
    bodies = [
        [0.0, 0.0, 0.0, 0.0],
        [0.1, params["y2"], params["vx2"], params["vy2"]],
        [x3, y3, params["vx3"], params["vy3"]],
    ]
    trails = [[(body[0], body[1])] for body in bodies]
    escaped_at = None

    for frame in range(1, frame_limit + 1):
        for _ in range(UPDATES_PER_FRAME * SPEED_MULTIPLIER):
            compute_forces(bodies)
            update_positions(bodies)
        for i, body in enumerate(bodies):
            trails[i].append((body[0], body[1]))
        if total_distance(bodies) > MAX_DISTANCE:
            escaped_at = frame
            break

    return trails, escaped_at


def pixel_to_initial(pixel_x, pixel_y, width, height):
    return pixel_x - width // 2, height // 2 - pixel_y


def make_surface_from_image(path: Path):
    image = Image.open(path).convert("RGB")
    return pygame.image.fromstring(image.tobytes(), image.size, "RGB"), image.size


def fit_scale(src_w, src_h, dst_rect):
    return min(dst_rect.width / src_w, dst_rect.height / src_h)


def image_origin(area, image_w, image_h, image_scale, image_pan):
    x = area.centerx - image_w * image_scale * 0.5 + image_pan[0]
    y = area.centery - image_h * image_scale * 0.5 + image_pan[1]
    return x, y


def screen_to_pixel(mx, my, area, image_w, image_h, image_scale, image_pan):
    origin_x, origin_y = image_origin(area, image_w, image_h, image_scale, image_pan)
    px = int((mx - origin_x) / image_scale)
    py = int((my - origin_y) / image_scale)
    if 0 <= px < image_w and 0 <= py < image_h and area.collidepoint(mx, my):
        return px, py
    return None


def draw_pixel_image(surface, image_surface, area, image_w, image_h, image_scale, image_pan):
    pygame.draw.rect(surface, (12, 13, 16), area)
    pygame.draw.rect(surface, (85, 90, 100), area, 1)

    origin_x, origin_y = image_origin(area, image_w, image_h, image_scale, image_pan)
    src_left = max(0, int(math.floor((area.left - origin_x) / image_scale)))
    src_top = max(0, int(math.floor((area.top - origin_y) / image_scale)))
    src_right = min(image_w, int(math.ceil((area.right - origin_x) / image_scale)))
    src_bottom = min(image_h, int(math.ceil((area.bottom - origin_y) / image_scale)))
    if src_right <= src_left or src_bottom <= src_top:
        return

    src_rect = pygame.Rect(src_left, src_top, src_right - src_left, src_bottom - src_top)
    dest_rect = pygame.Rect(
        int(origin_x + src_left * image_scale),
        int(origin_y + src_top * image_scale),
        max(1, int(math.ceil(src_rect.width * image_scale))),
        max(1, int(math.ceil(src_rect.height * image_scale))),
    )

    clipped_dest = dest_rect.clip(area)
    if clipped_dest.width <= 0 or clipped_dest.height <= 0:
        return

    visible = image_surface.subsurface(src_rect)
    scaled = pygame.transform.scale(visible, dest_rect.size)
    surface.set_clip(area)
    surface.blit(scaled, dest_rect)
    surface.set_clip(None)


def world_bounds(trails):
    xs = [p[0] for trail in trails for p in trail]
    ys = [p[1] for trail in trails for p in trail]
    if not xs:
        return -1.0, 1.0, -1.0, 1.0
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    if math.isclose(min_x, max_x):
        min_x -= 1.0
        max_x += 1.0
    if math.isclose(min_y, max_y):
        min_y -= 1.0
        max_y += 1.0
    return min_x, max_x, min_y, max_y


def world_to_screen(point, rect, bounds, zoom):
    min_x, max_x, min_y, max_y = bounds
    cx = (min_x + max_x) * 0.5
    cy = (min_y + max_y) * 0.5
    span_x = (max_x - min_x) / zoom
    span_y = (max_y - min_y) / zoom
    span = max(span_x, span_y, 1.0)
    sx = rect.centerx + (point[0] - cx) / span * rect.width * 0.9
    sy = rect.centery - (point[1] - cy) / span * rect.height * 0.9
    return int(sx), int(sy)


def draw_text(surface, font, text, pos, color=(230, 230, 230)):
    rendered = font.render(text, True, color)
    surface.blit(rendered, pos)


def draw_trajectory(surface, rect, trails, escaped_at, selected, frame_limit, zoom, font):
    pygame.draw.rect(surface, (20, 22, 26), rect)
    pygame.draw.rect(surface, (85, 90, 100), rect, 1)

    if not trails:
        return

    bounds = world_bounds(trails)
    colors = [(95, 185, 255), (255, 195, 80), (255, 95, 130)]
    labels = ["body 1", "body 2", "body 3"]

    for i, trail in enumerate(trails):
        if len(trail) < 2:
            continue
        points = [world_to_screen(p, rect, bounds, zoom) for p in trail]
        pygame.draw.lines(surface, colors[i], False, points, 1)
        marker_count = min(MAX_TRAIL_MARKERS, max(0, len(points) - 2))
        if marker_count:
            step = (len(points) - 1) / (marker_count + 1)
            for marker in range(1, marker_count + 1):
                idx = min(len(points) - 1, max(0, int(round(marker * step))))
                pygame.draw.circle(surface, colors[i], points[idx], 2)
        pygame.draw.circle(surface, colors[i], points[-1], 4)
        draw_text(surface, font, labels[i], (rect.x + 14, rect.y + 16 + i * 18), colors[i])

    x3, y3 = selected
    status = f"x3={x3:.0f}  y3={y3:.0f}  frames={frame_limit}"
    if escaped_at is not None:
        status += f"  escape={escaped_at}"
    else:
        status += "  bounded-in-window"
    draw_text(surface, font, status, (rect.x + 14, rect.bottom - 26), (220, 220, 225))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image", nargs="?", type=Path, help="parameter PNG, e.g. 100.1c0c20c1c0.png")
    args = parser.parse_args()

    image_path = args.image or choose_default_image()
    params = parse_params_from_name(image_path)

    pygame.init()
    pygame.display.set_caption(f"Three-body parameter explorer - {image_path.name}")
    font = pygame.font.SysFont("consolas", 16)
    small_font = pygame.font.SysFont("consolas", 14)

    image_surface, image_size = make_surface_from_image(image_path)
    image_w, image_h = image_size
    window = pygame.display.set_mode((1180, 740), pygame.RESIZABLE)
    clock = pygame.time.Clock()

    trails = None
    escaped_at = None
    selected_pixel = None
    selected_initial = (0.0, 0.0)
    frame_limit = DEFAULT_TRAIL_FRAMES
    zoom = 1.0
    image_scale = None
    image_pan = [0.0, 0.0]
    dragging_image = False
    drag_last = (0, 0)
    dirty = True
    running = True

    while running:
        width, height = window.get_size()
        margin = 16
        top_h = 42
        left_w = min(width - 2 * margin, max(420, int(width * 0.55)))
        if width < 960:
            left_w = width - 2 * margin
        image_area = pygame.Rect(margin, top_h + margin, left_w, height - top_h - 2 * margin)

        if image_scale is None:
            image_scale = fit_scale(image_w, image_h, image_area)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    frame_limit = min(MAX_TRAIL_FRAMES, int(frame_limit * 1.35))
                    dirty = True
                elif event.key == pygame.K_MINUS:
                    frame_limit = max(MIN_TRAIL_FRAMES, int(frame_limit / 1.35))
                    dirty = True
                elif event.key == pygame.K_LEFTBRACKET:
                    zoom = max(0.25, zoom / 1.25)
                elif event.key == pygame.K_RIGHTBRACKET:
                    zoom = min(16.0, zoom * 1.25)
                elif event.key == pygame.K_s:
                    pygame.image.save(window, "trajectory_viewer_screenshot.png")
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 3 and image_area.collidepoint(event.pos):
                    dragging_image = True
                    drag_last = event.pos
                elif event.button in (4, 5) and image_area.collidepoint(event.pos):
                    before = screen_to_pixel(event.pos[0], event.pos[1], image_area, image_w, image_h, image_scale, image_pan)
                    old_scale = image_scale
                    if event.button == 4:
                        image_scale = min(24.0, image_scale * 1.25)
                    else:
                        image_scale = max(0.08, image_scale / 1.25)
                    if before is not None:
                        old_origin = image_origin(image_area, image_w, image_h, old_scale, image_pan)
                        new_origin_base = image_origin(image_area, image_w, image_h, image_scale, (0.0, 0.0))
                        image_pan[0] = event.pos[0] - before[0] * image_scale - new_origin_base[0]
                        image_pan[1] = event.pos[1] - before[1] * image_scale - new_origin_base[1]
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 3:
                    dragging_image = False
            elif event.type == pygame.MOUSEMOTION:
                if dragging_image:
                    dx = event.pos[0] - drag_last[0]
                    dy = event.pos[1] - drag_last[1]
                    image_pan[0] += dx
                    image_pan[1] += dy
                    drag_last = event.pos

        window.fill((12, 13, 16))

        if width >= 960:
            traj_rect = pygame.Rect(image_area.right + margin, image_area.y, width - image_area.right - 2 * margin, image_area.height)
        else:
            traj_rect = pygame.Rect(margin, image_area.bottom + margin, width - 2 * margin, max(260, height - image_area.bottom - 2 * margin))

        draw_text(
            window,
            font,
            f"{image_path.name}   y2={params['y2']:g} vx2={params['vx2']:g} vy2={params['vy2']:g} vx3={params['vx3']:g} vy3={params['vy3']:g}",
            (margin, 14),
        )

        draw_pixel_image(window, image_surface, image_area, image_w, image_h, image_scale, image_pan)

        mx, my = pygame.mouse.get_pos()
        pixel = screen_to_pixel(mx, my, image_area, image_w, image_h, image_scale, image_pan)
        if pixel is not None:
            px, py = pixel
            if selected_pixel != (px, py):
                selected_pixel = (px, py)
                selected_initial = pixel_to_initial(px, py, image_w, image_h)
                dirty = True

            origin_x, origin_y = image_origin(image_area, image_w, image_h, image_scale, image_pan)
            cx = int(origin_x + (px + 0.5) * image_scale)
            cy = int(origin_y + (py + 0.5) * image_scale)
            pygame.draw.line(window, (255, 255, 255), (cx, image_area.y), (cx, image_area.bottom), 1)
            pygame.draw.line(window, (255, 255, 255), (image_area.x, cy), (image_area.right, cy), 1)

        if selected_pixel is None:
            selected_pixel = (image_w // 2, image_h // 2)
            selected_initial = pixel_to_initial(selected_pixel[0], selected_pixel[1], image_w, image_h)
            dirty = True

        if dirty:
            trails, escaped_at = simulate_trajectory(selected_initial[0], selected_initial[1], params, frame_limit)
            dirty = False

        draw_trajectory(window, traj_rect, trails, escaped_at, selected_initial, frame_limit, zoom, small_font)

        hint = "left image: wheel zoom, right-drag pan   trajectory: +/- frames, [/ ] zoom   S screenshot   Esc quit"
        draw_text(window, small_font, hint, (margin, height - 22), (165, 170, 180))

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()


if __name__ == "__main__":
    main()
