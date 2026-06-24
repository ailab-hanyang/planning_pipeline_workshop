#!/usr/bin/env python3
"""기본: 동기(synchronous) 모드 켜고 일정 틱 진행 후 복구.

노트북 셀로 쪼개면 freeze/deadlock 나는 부분 -> 스크립트로 한 번에 실행.
실행: python3 run_syncmode.py
"""
import carla


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    original_settings = world.get_settings()
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05      # 20Hz
        world.apply_settings(settings)
        print('동기 모드 진입 — 200틱(10초) 진행')
        for i in range(200):
            world.tick()
            if i % 20 == 0:
                print(f'  tick {i}  (sim time {i*0.05:.1f}s)')
        print('완료')
    except KeyboardInterrupt:
        print('\n[Ctrl+C] 종료')
    finally:
        world.apply_settings(original_settings)   # 비동기로 복구 (중요)
        print('원래 설정 복구 완료')


if __name__ == '__main__':
    main()
