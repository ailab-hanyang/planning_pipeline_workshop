#!/usr/bin/env bash
#======================================================================#
# Module:      check_env.sh
# Description: 자율주행 Planning Pipeline 실습 PC 환경 점검 스크립트
#              (Ubuntu 22.04 / NVIDIA 드라이버 / Python 패키지 / VS Code / 한글 입력)
#
# Authors: Hanyang University, Automotive Intelligence Laboratory (AILAB)
#
# 사용법
#   bash check_env.sh                              # 기본 python3 기준 점검 (약 1분)
#   bash check_env.sh --python ~/venv/bin/python   # 가상환경 python 지정
#
#   - 바탕화면(GUI) 세션의 터미널에서 실행 (sudo 불필요)
#   - 결과 로그: 현재 폴더의 env_check_<호스트명>_<날짜>.log  (문제가 있을 때 강사에게 보여 주기)
#   - 판정: [PASS] 정상 / [WARN] 확인 권장 (실습 진행은 가능) / [FAIL] 실습 전 조치 필요
#======================================================================#

set -u

PY="python3"
while [ $# -gt 0 ]; do
    case "$1" in
        --python) PY="$2"; shift 2 ;;
        -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
        *) echo "알 수 없는 옵션: $1 (--help 참고)"; exit 2 ;;
    esac
done

export PYGAME_HIDE_SUPPORT_PROMPT=1
LOG="$(pwd)/env_check_$(hostname)_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOG") 2>&1

N_PASS=0; N_WARN=0; N_FAIL=0
FAIL_LIST=(); WARN_LIST=()
pass() { echo -e "  \033[32m[PASS]\033[0m $1"; N_PASS=$((N_PASS+1)); }
warn() { echo -e "  \033[33m[WARN]\033[0m $1"; N_WARN=$((N_WARN+1)); WARN_LIST+=("$1"); }
fail() { echo -e "  \033[31m[FAIL]\033[0m $1"; N_FAIL=$((N_FAIL+1)); FAIL_LIST+=("$1"); }
info() { echo "         $1"; }
section() { echo; echo "=== $1 ==="; }
has() { command -v "$1" >/dev/null 2>&1; }
# 하위 python 블록 출력의 [PASS]/[WARN]/[FAIL] 을 집계 ($2 = 요약에 쓸 이름)
tally() {
    echo "$1"
    local p w f
    p=$(grep -c '\[PASS\]' <<<"$1"); w=$(grep -c '\[WARN\]' <<<"$1"); f=$(grep -c '\[FAIL\]' <<<"$1")
    N_PASS=$((N_PASS + p)); N_WARN=$((N_WARN + w)); N_FAIL=$((N_FAIL + f))
    [ "$w" -gt 0 ] && WARN_LIST+=("$2: ${w}개")
    [ "$f" -gt 0 ] && FAIL_LIST+=("$2: ${f}개")
    return 0
}

echo "실습 PC 환경 점검  $(date '+%Y-%m-%d %H:%M:%S')  host=$(hostname)  user=$(whoami)"

# ----------------------------------------------------------------------
section "1. 운영체제 / 하드웨어"
# ----------------------------------------------------------------------
. /etc/os-release 2>/dev/null
if [ "${VERSION_ID:-}" = "22.04" ]; then pass "OS: ${PRETTY_NAME}"; else fail "OS: ${PRETTY_NAME:-알 수 없음} (Ubuntu 22.04 필요)"; fi
info "커널: $(uname -r)"
info "CPU: $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2 | sed 's/^ //') ($(nproc) threads)"
MEM_GB=$(awk '/MemTotal/ {printf "%d", $2/1024/1024}' /proc/meminfo)
if [ "$MEM_GB" -ge 30 ]; then pass "RAM: ${MEM_GB} GB"; else warn "RAM: ${MEM_GB} GB (32 GB 이상 권장)"; fi
FREE_GB=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc '0-9')
if [ "$FREE_GB" -ge 50 ]; then pass "디스크 여유 공간(홈): ${FREE_GB} GB"; else warn "디스크 여유 공간(홈): ${FREE_GB} GB (50 GB 이상 권장)"; fi
if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    pass "GUI 세션: ${XDG_SESSION_TYPE:-unknown} / ${XDG_CURRENT_DESKTOP:-unknown}"
else
    warn "GUI 세션 없음 (원격 SSH 등) -> 바탕화면 터미널에서 재실행 권장"
fi

# ----------------------------------------------------------------------
section "2. NVIDIA 그래픽 드라이버 / Vulkan (CARLA 실행 조건)"
# ----------------------------------------------------------------------
if has nvidia-smi && nvidia-smi >/dev/null 2>&1; then
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)
    DRV=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)
    VRAM=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader | head -1)
    pass "nvidia-smi 동작: ${GPU_NAME}, 드라이버 ${DRV}, VRAM ${VRAM}"
    DRV_MAJOR=${DRV%%.*}
    LIC=$(modinfo -F license nvidia 2>/dev/null)
    # RTX 50 시리즈(Blackwell): 드라이버 570 이상 + 오픈 커널 모듈(nvidia-driver-XXX-open) 필수
    if echo "$GPU_NAME" | grep -qE 'RTX 50[0-9]{2}|Blackwell'; then
        if [ "$DRV_MAJOR" -ge 570 ]; then pass "RTX 50 시리즈 드라이버 버전: ${DRV} (570 이상)"; else fail "RTX 50 시리즈는 드라이버 570 이상 필요 (현재 ${DRV})"; fi
        if [ "$LIC" = "Dual MIT/GPL" ]; then pass "커널 모듈: 오픈 모듈 (license=${LIC})"; else fail "RTX 50 시리즈는 오픈 커널 모듈 필요 (현재 license=${LIC:-알 수 없음}) -> nvidia-driver-${DRV_MAJOR}-open 설치"; fi
    else
        info "커널 모듈 license=${LIC:-알 수 없음}"
        if [ "$DRV_MAJOR" -ge 535 ]; then pass "드라이버 버전: ${DRV}"; else warn "드라이버 버전 ${DRV} (535 이상 권장)"; fi
    fi
    if lsmod | grep -q '^nouveau'; then fail "nouveau 드라이버가 로드됨 (NVIDIA 드라이버와 충돌)"; fi
else
    fail "nvidia-smi 실행 실패 -> NVIDIA 드라이버 미설치 또는 재부팅 필요"
fi
if ldconfig -p 2>/dev/null | grep -q 'libvulkan.so.1'; then pass "Vulkan 로더(libvulkan1) 설치됨"; else fail "libvulkan1 미설치 (CARLA 는 Vulkan 사용) -> sudo apt install libvulkan1"; fi
if [ -f /usr/share/vulkan/icd.d/nvidia_icd.json ] || [ -f /etc/vulkan/icd.d/nvidia_icd.json ]; then
    pass "NVIDIA Vulkan ICD(nvidia_icd.json) 존재"
else
    fail "NVIDIA Vulkan ICD(nvidia_icd.json) 없음 -> NVIDIA 드라이버 재설치 필요"
fi

# ----------------------------------------------------------------------
section "3. Python  (python: ${PY})"
# ----------------------------------------------------------------------
if ! has "$PY" && [ ! -x "$PY" ]; then
    fail "python 실행 파일 없음: ${PY}"
    PY=""
else
    PYV=$("$PY" -c 'import sys; print("%d.%d.%d" % sys.version_info[:3])')
    if [[ "$PYV" == 3.10.* ]]; then pass "Python ${PYV} ($("$PY" -c 'import sys; print(sys.executable)'))"; else fail "Python ${PYV} (실습 기준 3.10, CARLA 0.9.16 wheel 은 cp310)"; fi
    "$PY" -m pip --version >/dev/null 2>&1 && pass "pip 사용 가능" || fail "pip 없음 -> sudo apt install python3-pip"
fi

# ----------------------------------------------------------------------
section "4. Python 패키지"
# ----------------------------------------------------------------------
if [ -n "$PY" ]; then
    # 묶음 | 배포 이름 | import 이름 | 조건
    #   Day 1·2  : requirements/requirements.txt (VS Code 노트북 실행 = ipykernel, 슬라이더 = ipywidgets)
    #   Day 3    : CARLA 0.9.16 wheel + requirements/requirements_carla.txt
    #   CARLA 예제: CARLA 설치 폴더의 PythonAPI/examples/requirements.txt
    OUT=$("$PY" - 2>&1 <<'PYEOF'
import importlib, importlib.metadata as md, re, warnings
warnings.filterwarnings('ignore')
G, Y, R, E = '\033[32m', '\033[33m', '\033[31m', '\033[0m'
def ver(s): return tuple(int(x) for x in re.findall(r'\d+', s)[:3])
groups = {
    'Day 1·2 (requirements.txt)': [
        ('numpy', 'numpy', '>=1.24'), ('scipy', 'scipy', '>=1.15'), ('matplotlib', 'matplotlib', '>=3.10'),
        ('ipykernel', 'ipykernel', '>=6.0'), ('ipywidgets', 'ipywidgets', '>=8.0')],
    'Day 3 (CARLA wheel + requirements_carla.txt)': [
        ('carla', 'carla', '==0.9.16'), ('pygame', 'pygame', '>=2.6')],
    'CARLA 예제 (PythonAPI/examples/requirements.txt)': [
        ('future', 'future', ''), ('networkx', 'networkx', ''), ('shapely', 'shapely', ''),
        ('open3d', 'open3d', ''), ('pillow', 'PIL', ''), ('invertedai', 'invertedai', '')],
}
for title, pkgs in groups.items():
    print(f'  --- {title}')
    for dist, mod, cond in pkgs:
        try:
            v = md.version(dist)
        except md.PackageNotFoundError:
            print(f'  {R}[FAIL]{E} {dist} 미설치'); continue
        try:
            if mod: importlib.import_module(mod)
        except Exception as e:
            print(f'  {R}[FAIL]{E} {dist} {v} 설치됨, import 실패: {type(e).__name__}: {str(e)[:120]}'); continue
        if cond.startswith('=='):
            ok = ver(v) == ver(cond)
        else:
            ok = not cond or ver(v) >= ver(cond)
        if ok: print(f'  {G}[PASS]{E} {dist} {v}')
        else:  print(f'  {R}[FAIL]{E} {dist} {v} (필요 {cond})')
PYEOF
)
    tally "$OUT" "Python 패키지 (4번 항목)"
    info "설치: pip install -r requirements/requirements.txt -r requirements/requirements_carla.txt"
    info "      pip install <CARLA경로>/PythonAPI/carla/dist/carla-0.9.16-cp310-cp310-manylinux_2_31_x86_64.whl"
    info "      pip install -r <CARLA경로>/PythonAPI/examples/requirements.txt"

    # matplotlib 중복 설치 (apt python3-matplotlib + pip) -> 노트북마다 Axes3D 경고 출력
    if "$PY" -W error -c 'import mpl_toolkits.mplot3d' >/dev/null 2>&1; then
        pass "matplotlib 단일 설치 (Axes3D 경고 없음)"
    else
        warn "matplotlib 중복 설치 의심 (노트북마다 'Unable to import Axes3D' 경고) -> sudo apt remove python3-matplotlib 또는 venv 사용"
    fi
    # 노트북 Step04 그림의 한글 표시용 폰트
    if fc-list :lang=ko 2>/dev/null | grep -qi 'nanum\|noto.*cjk'; then pass "한글 폰트 설치됨 (그림 내 한글 표시)"; else warn "한글 폰트 없음 -> sudo apt install fonts-nanum (그림 속 한글이 □ 로 깨짐)"; fi
fi

# ----------------------------------------------------------------------
section "5. VS Code"
# ----------------------------------------------------------------------
if has code; then
    pass "VS Code $(code --version 2>/dev/null | head -1)"
    EXT=$(code --list-extensions 2>/dev/null)
    for e in ms-python.python ms-toolsai.jupyter; do
        echo "$EXT" | grep -qi "^${e}$" && pass "확장: ${e}" || warn "확장 미설치: ${e} -> code --install-extension ${e}"
    done
else
    fail "VS Code(code) 미설치"
fi

# ----------------------------------------------------------------------
section "6. 한/영 입력"
# ----------------------------------------------------------------------
if dpkg -s ibus-hangul >/dev/null 2>&1; then IM=ibus; pass "ibus-hangul 설치됨"
elif dpkg -s fcitx5-hangul >/dev/null 2>&1 || dpkg -s fcitx-hangul >/dev/null 2>&1; then IM=fcitx; pass "fcitx hangul 설치됨"
else IM=""; fail "한글 입력기 미설치 -> sudo apt install ibus-hangul"; fi
if [ "$IM" = "ibus" ] && has gsettings; then
    SRC=$(gsettings get org.gnome.desktop.input-sources sources 2>/dev/null)
    echo "$SRC" | grep -q "hangul" && pass "입력 소스에 한국어(Hangul) 등록: ${SRC}" || fail "입력 소스에 한국어(Hangul) 없음: ${SRC} -> 설정 > 키보드 > 입력 소스 추가"
    SW=$(gsettings get org.freedesktop.ibus.engine.hangul switch-keys 2>/dev/null)
    XKB=$(gsettings get org.gnome.desktop.input-sources xkb-options 2>/dev/null)
    info "한/영 전환 키(ibus-hangul switch-keys): ${SW:-알 수 없음},  xkb-options: ${XKB:-[]}"
    if echo "$SW $XKB" | grep -qE "Hangul|Alt_R|ralt_hangul"; then
        pass "한/영 전환 키 설정됨 (Hangul / 오른쪽 Alt)"
    else
        warn "한/영 키 전환 미설정 -> ibus-hangul 설정에서 '한영 전환 키'에 Hangul 또는 Alt_R 추가"
    fi
fi
info "※ 직접 확인: 텍스트 편집기에서 한/영 키를 눌러 '한글' 입력이 되는지 확인"

# ----------------------------------------------------------------------
section "7. 기능 테스트 (Python / Jupyter)"
# ----------------------------------------------------------------------
if [ -n "$PY" ]; then
    TMPD=$(mktemp -d)
    if SDL_VIDEODRIVER=dummy MPLBACKEND=Agg PYTHONWARNINGS=ignore "$PY" - "$TMPD" <<'PYEOF' 2>&1
import sys, os
import numpy as np, matplotlib.pyplot as plt
from scipy.optimize import minimize
r = minimize(lambda z: (z[0] - 1) ** 2 + (z[1] - 2) ** 2, [0, 0], method='SLSQP',
             constraints=[{'type': 'ineq', 'fun': lambda z: 2.5 - z[0] - z[1]}])
assert r.success and abs(r.x.sum() - 2.5) < 1e-4, 'SLSQP 결과 이상'
plt.plot([0, 1], [0, 1]); plt.savefig(os.path.join(sys.argv[1], 't.png'))
import pygame; pygame.init(); pygame.display.set_mode((64, 64)); pygame.quit()
import carla
t = carla.Transform(carla.Location(x=1.5, z=2.4), carla.Rotation(pitch=-15.0))
assert abs(t.rotation.pitch + 15.0) < 1e-6
print('ok')
PYEOF
    then pass "numpy / scipy(SLSQP) / matplotlib / pygame / carla API 동작"; else fail "Python 기능 테스트 실패 (위 메시지 참고)"; fi
    # VS Code 가 노트북을 실행하는 방식과 동일: 이 python 의 ipykernel 커널을 띄워 코드 실행
    if PYTHONWARNINGS=ignore timeout 120 "$PY" - <<'PYEOF' >/dev/null 2>&1
import sys
from jupyter_client import KernelManager
km = KernelManager(kernel_cmd=[sys.executable, '-m', 'ipykernel_launcher', '-f', '{connection_file}'])
km.start_kernel(); kc = km.client(); kc.start_channels(); kc.wait_for_ready(timeout=60)
out = []
kc.execute_interactive('import numpy as np, ipywidgets; print(np.arange(3).sum())', timeout=60,
                       output_hook=lambda m: out.append(m['content'].get('text', '')))
kc.stop_channels(); km.shutdown_kernel(now=True)
assert '3' in ''.join(out)
PYEOF
    then
        pass "노트북 커널(ipykernel) 실행 -> VS Code 노트북 실행 조건"
    else
        fail "노트북 커널 실행 실패 -> ${PY} -m pip install ipykernel ipywidgets"
    fi
    rm -rf "$TMPD"
fi

# ----------------------------------------------------------------------
section "요약"
# ----------------------------------------------------------------------
echo "  PASS ${N_PASS}  /  WARN ${N_WARN}  /  FAIL ${N_FAIL}"
if [ "$N_FAIL" -gt 0 ]; then echo "  조치 필요 (FAIL):"; for x in "${FAIL_LIST[@]}"; do echo "    - $x"; done; fi
if [ "$N_WARN" -gt 0 ]; then echo "  확인 권장 (WARN):"; for x in "${WARN_LIST[@]}"; do echo "    - $x"; done; fi
[ "$N_FAIL" -eq 0 ] && echo "  => 실습 환경 준비 완료" || echo "  => FAIL 항목 조치 후 다시 실행"
echo "  로그 파일: ${LOG}"
[ "$N_FAIL" -eq 0 ]
