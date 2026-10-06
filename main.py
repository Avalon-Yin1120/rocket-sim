# -*- coding: utf-8 -*-
import math
import sys
import pygame

from config import (WIDTH, HEIGHT, FPS, TIME_SCALES, load_vehicle,
                    load_font, next_preset)
import rendering as R
from camera import Camera
from physics import Rocket, FlightLearner


PARTICLE_RENDER_CUTOFF = 50.0

# 积分预算：每帧最多切多少块、每块最多多长。
# 推力段必须细（TVC 作动器时间常数 0.05 s，块长超过 0.1 s 就会震荡）；
# 滑行段可以很粗（RK4 + 0.5 s，圆轨道跑 120 s 能量漂移 3e-14）。
# 预算用满时自动降低实际倍速，而不是让积分变粗。
CHUNK_BURN = 0.05
CHUNK_COAST = 0.5
CHUNK_BUDGET = 400


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("火箭模拟 · 入轨 / 探月 · TVC·RCS（自动学习）")
    clock = pygame.time.Clock()
    R.init_fonts()

    countdown_font = load_font(72, bold=True)

    # 跨多次发射共享的学习器（按预设分别记表）
    learner = FlightLearner()

    rocket = None
    cam = Camera()
    time_idx = 0
    time_scale = TIME_SCALES[time_idx]
    eff_scale = time_scale
    limited = False

    def new_flight(preset=None):
        """（重新）开始一次飞行。preset=None 表示按配置文件/上次的预设。"""
        nonlocal rocket, cam, time_idx, time_scale, eff_scale, limited
        veh = load_vehicle(preset)
        learner.for_preset(veh["preset"], veh["pitch_v_ref"])
        rocket = Rocket(veh, learner=learner)
        cam = Camera()
        time_idx = 0
        time_scale = TIME_SCALES[0]
        eff_scale = time_scale
        limited = False

    new_flight()

    t_anim = 0.0
    hud_visible = True
    running = True

    while running:
        raw_dt = clock.tick(FPS) / 1000.0
        raw_dt = min(raw_dt, 1.0 / FPS * 10.0)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    new_flight(None)          # 同一枚火箭重飞
                elif event.key == pygame.K_p:
                    new_flight(next_preset(rocket.preset))   # 换下一枚火箭
                elif event.key in (pygame.K_LCTRL, pygame.K_RCTRL):
                    rocket.auto_mode = not rocket.auto_mode
                    rocket._log("切换为%s控制" %
                                ("自动" if rocket.auto_mode else "手动"))
                elif event.key == pygame.K_w:
                    # 发动机主阀：一次按键切换点火/关机（不需要一直按住）
                    rocket.toggle_engine()
                elif event.key == pygame.K_t:
                    hud_visible = not hud_visible
                elif event.key == pygame.K_m:
                    cam.toggle_mode()
                elif event.key == pygame.K_UP:
                    if time_idx < len(TIME_SCALES) - 1:
                        time_idx += 1
                        time_scale = TIME_SCALES[time_idx]
                        rocket._log(f"时间 ×{time_scale:.0f}")
                elif event.key == pygame.K_DOWN:
                    if time_idx > 0:
                        time_idx -= 1
                        time_scale = TIME_SCALES[time_idx]
                        rocket._log(f"时间 ×{time_scale:.0f}")
            elif event.type == pygame.MOUSEWHEEL:
                if event.y > 0:
                    cam.zoom(1.3)
                elif event.y < 0:
                    cam.zoom(1 / 1.3)

        keys = pygame.key.get_pressed()
        key_a = keys[pygame.K_a]
        key_d = keys[pygame.K_d]

        # ---- 积分预算：步长按段选，块数受预算限制 ----
        # 预算用满时降低实际推进的物理时间（= 自动降速），而不是把步长放粗，
        # 否则 TVC 回路会在大步长下自激发散。
        dt_want = raw_dt * time_scale
        if rocket.state == Rocket.STATE_PRELAUNCH or rocket.engine_on:
            chunk_max = CHUNK_BURN
        else:
            chunk_max = CHUNK_COAST
        n_chunks = max(1, int(math.ceil(dt_want / chunk_max)))
        if n_chunks > CHUNK_BUDGET:
            n_chunks = CHUNK_BUDGET
        dt_total = min(dt_want, n_chunks * chunk_max)
        eff_scale = dt_total / raw_dt if raw_dt > 1e-9 else time_scale
        limited = eff_scale < time_scale * 0.98
        chunk = dt_total / n_chunks
        real_chunk = raw_dt / n_chunks
        for _ in range(n_chunks):
            rocket.update(chunk, key_a, key_d, real_dt=real_chunk)

        # ---- 任务结束：记录一次，微调学习器 ----
        # 注意用的是 mission_done：入轨任务=进地球轨道，探月任务=进绕月轨道。
        # 否则探月火箭刚进停泊轨道就会被当成"飞完了"。
        prev_flag = getattr(rocket, "_learn_recorded", False)
        cur_flag = (rocket.crashed or rocket.broken
                    or rocket.burned_up or rocket.landed_soft
                    or rocket.mission_done)
        if cur_flag and not prev_flag:
            if rocket.used_auto:
                reward = learner.observe_flight_end(rocket)
                rocket._log(f"自动学习: 奖励 {reward:+.2f} · 第 {learner.generation} 代")
            else:
                rocket._log("手动飞行，不参与自动学习")
            rocket._learn_recorded = True

        # 相机与高度表量程都按**物理时间**推进：旧代码用 raw_dt，100× 倍速下
        # 火箭在 0.3 秒真实时间里爬升几十公里，镜头和量程会严重滞后。
        # 取景参考高度用 focus_alt：进月球影响球后就是距月面高度。
        cam.update(dt_total, rocket.body.position.x,
                   rocket.body.position.y, rocket.focus_alt)

        t_anim += raw_dt

        # ---- 渲染 ----
        R.draw_sky(screen, cam)
        R.draw_stars(screen, cam, rocket.altitude, t_anim)
        R.draw_earth(screen, cam, t_anim)
        R.draw_moon(screen, cam, rocket)
        R.draw_surface_objects(screen, cam)

        R.draw_trail(screen, cam, rocket.trail)
        R.draw_predicted_orbit(screen, cam, rocket)
        for d in rocket.debris:
            R.draw_debris(screen, cam, d)
        R.draw_rocket(screen, cam, rocket)
        R.draw_shock_and_sonicboom(screen, cam, rocket)

        if time_scale < PARTICLE_RENDER_CUTOFF:
            R.draw_particles(screen, cam, rocket.exhaust)
            R.draw_particles(screen, cam, rocket.particles)

        if hud_visible:
            cam.update_altitude_span(dt_total, rocket.focus_alt)
            R.draw_altimeter(screen, rocket.focus_alt, cam.alt_max,
                             rocket.primary)
            R.draw_hud(screen, rocket, time_scale, eff_scale if limited else None)

        if rocket.state == Rocket.STATE_PRELAUNCH:
            sec = max(0, int(math.ceil(rocket.countdown_t)))
            text = f"T-{sec:02d}"
            surf_t = countdown_font.render(text, True, (255, 240, 120))
            screen.blit(surf_t, (WIDTH // 2 - surf_t.get_width() // 2, 100))

        mode_text = "相机：自动" if cam.mode == "auto" else "相机：手动"
        mode_surf = R.FONT_SM.render(mode_text, True, (180, 200, 230))
        screen.blit(mode_surf, (WIDTH - mode_surf.get_width() - 20, 40))

        # 火箭预设 + 学习进度
        gen_text = f"{rocket.vehicle_title} · 第 {learner.generation} 代"
        gen_surf = R.FONT_SM.render(gen_text, True, (255, 210, 130))
        screen.blit(gen_surf, (WIDTH - gen_surf.get_width() - 20, 62))

        # 倍速被积分预算压低时明确说明，避免"为什么按了 10000× 却没那么快"
        if limited:
            spd_text = f"时倍 ×{time_scale:.0f} → 实际 ×{eff_scale:.0f}（步长预算）"
            spd_surf = R.FONT_SM.render(spd_text, True, (255, 170, 120))
            screen.blit(spd_surf, (WIDTH - spd_surf.get_width() - 20, 84))

        tip_txt = ("A D 姿态 | W 点火/关机 | ↑ ↓ 时倍 | Ctrl 自动 | M 相机 | "
                   "T 显隐HUD | P 换火箭 | 滚轮 缩放 | R 重开 | ESC 退出")
        tip = R.FONT_SM.render(tip_txt, True, (200, 210, 225))
        screen.blit(tip, (16, HEIGHT - 30))

        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()