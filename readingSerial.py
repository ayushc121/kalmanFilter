import serial
import re
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---- Config --------------------------------------------------
PORT     = 'COM3'
BAUDRATE = 115200

# ---- Regex patterns ------------------------------------------
RE_IMU = re.compile(
    r'Accel \[X:([\d\.\-]+) Y:([\d\.\-]+) Z:([\d\.\-]+)\]'
    r'\s+Gyro \[X:([\d\.\-]+) Y:([\d\.\-]+) Z:([\d\.\-]+)\]'
)
RE_QUAT = re.compile(
    r'Quat \[W:([\d\.\-]+) X:([\d\.\-]+) Y:([\d\.\-]+) Z:([\d\.\-]+)\]'
)
RE_POS = re.compile(
    r'Pos \[X:([\d\.\-]+) Y:([\d\.\-]+) Z:([\d\.\-]+)\]'
)
RE_GPS_POS = re.compile(
    r'Lat:\s*([\d\.\-]+)\s+Lon:\s*([\d\.\-]+)\s+Alt:\s*([\d\.\-]+)m'
)
RE_GPS_VEL = re.compile(
    r'Speed:\s*([\d\.\-]+) m/s\s+Course:\s*([\d\.\-]+) deg'
)
RE_GPS_SIG = re.compile(
    r'Sats:\s*(\d+)\s+HDOP:\s*([\d\.]+)'
)

# ---- State ---------------------------------------------------
state_lock = threading.Lock()
state = {
    'accel_x': None, 'accel_y': None, 'accel_z': None,
    'gyro_x':  None, 'gyro_y':  None, 'gyro_z':  None,
    'quat_w':  1.0,  'quat_x':  0.0, 'quat_y':  0.0, 'quat_z': 0.0,
    'pos_x':   0.0,  'pos_y':   0.0, 'pos_z':   0.0,
    'lat': None, 'lon': None, 'alt': None,
    'speed': None, 'course': None,
    'sats': None, 'hdop': None,
}

# ---- Quaternion → rotation matrix ----------------------------
def quat_to_rot(w, x, y, z):
    """Unit quaternion → 3×3 rotation matrix (row-major)."""
    n = np.sqrt(w*w + x*x + y*y + z*z)
    if n < 1e-10:
        return np.eye(3)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])

# ---- Rectangular prism geometry ------------------------------
# Half-extents: make it clearly non-cubic so rotation is obvious
# Scaled to real-world metres: original (0.5, 1.5, 0.2) * 0.12
# so the 1.5-unit dimension maps to ~0.18 m on screen
HX, HY, HZ = 0.06, 0.18, 0.024   # width, depth, height (metres)

# 8 corners of the box in body frame
BASE_VERTS = np.array([
    [-HX, -HY, -HZ],
    [ HX, -HY, -HZ],
    [ HX,  HY, -HZ],
    [-HX,  HY, -HZ],
    [-HX, -HY,  HZ],
    [ HX, -HY,  HZ],
    [ HX,  HY,  HZ],
    [-HX,  HY,  HZ],
])

# 6 faces as vertex index lists
FACES = [
    [0, 1, 2, 3],  # bottom
    [4, 5, 6, 7],  # top
    [0, 1, 5, 4],  # front
    [2, 3, 7, 6],  # back
    [1, 2, 6, 5],  # right
    [0, 3, 7, 4],  # left
]

FACE_COLORS = [
    '#4e9af1',  # bottom  – blue
    '#f1a24e',  # top     – orange
    '#6ec96e',  # front   – green
    '#c96e6e',  # back    – red
    '#c9c96e',  # right   – yellow
    '#9e6ec9',  # left    – purple
]

# ---- Parsing -------------------------------------------------
def parse_line(line: str):
    m = RE_QUAT.search(line)
    if m:
        with state_lock:
            state['quat_w'] = float(m.group(1))
            state['quat_x'] = float(m.group(2))
            state['quat_y'] = float(m.group(3))
            state['quat_z'] = float(m.group(4))
        return

    m = RE_POS.search(line)
    if m:
        with state_lock:
            state['pos_x'] = float(m.group(1))
            state['pos_y'] = float(m.group(2))
            state['pos_z'] = float(m.group(3))
        return

    m = RE_IMU.search(line)
    if m:
        with state_lock:
            state['accel_x'] = float(m.group(1))
            state['accel_y'] = float(m.group(2))
            state['accel_z'] = float(m.group(3))
            state['gyro_x']  = float(m.group(4))
            state['gyro_y']  = float(m.group(5))
            state['gyro_z']  = float(m.group(6))
        return

    m = RE_GPS_POS.search(line)
    if m:
        with state_lock:
            state['lat'] = float(m.group(1))
            state['lon'] = float(m.group(2))
            state['alt'] = float(m.group(3))
        return

    m = RE_GPS_VEL.search(line)
    if m:
        with state_lock:
            state['speed']  = float(m.group(1))
            state['course'] = float(m.group(2))
        return

    m = RE_GPS_SIG.search(line)
    if m:
        with state_lock:
            state['sats'] = int(m.group(1))
            state['hdop'] = float(m.group(2))
        return

    # No regex matched — print raw so startup messages / unknown lines are visible
    print(f"  [uart] {line}")

def print_state():
    try:
        with state_lock:
            s = state.copy()
        print("\n===== Latest Telemetry =====")
        if all(s[k] is not None for k in ('accel_x', 'accel_y', 'accel_z',
                                           'gyro_x',  'gyro_y',  'gyro_z')):
            print(f"  Accel  : X={s['accel_x']:>8.3f}g   Y={s['accel_y']:>8.3f}g   Z={s['accel_z']:>8.3f}g")
            print(f"  Gyro   : X={s['gyro_x']:>8.2f}°/s  Y={s['gyro_y']:>8.2f}°/s  Z={s['gyro_z']:>8.2f}°/s")
        print(f"  Quat   : W={s['quat_w']:.4f}  X={s['quat_x']:.4f}  Y={s['quat_y']:.4f}  Z={s['quat_z']:.4f}")
        print(f"  Pos    : X={s['pos_x']:.4f}m  Y={s['pos_y']:.4f}m  Z={s['pos_z']:.4f}m")
        if all(s[k] is not None for k in ('lat', 'lon', 'alt', 'speed', 'course', 'sats', 'hdop')):
            print(f"  GPS    : Lat={s['lat']:.6f}  Lon={s['lon']:.6f}  Alt={s['alt']:.1f}m")
            print(f"  Motion : Speed={s['speed']:.2f} m/s  Course={s['course']:.1f}°")
            print(f"  Signal : Sats={s['sats']}  HDOP={s['hdop']:.2f}")
        else:
            print("  GPS    : Waiting for satellite lock...")
        print("============================")
    except Exception as e:
        print(f"  [print_state error] {e}")

# ---- Serial thread -------------------------------------------
def serial_thread():
    try:
        print(f"Connecting to {PORT} at {BAUDRATE} baud...")
        esp_serial = serial.Serial(PORT, BAUDRATE, timeout=1)
        print("Connected. Listening (Ctrl+C to stop)...\n")
        while True:
            try:
                if esp_serial.in_waiting > 0:
                    raw = esp_serial.readline().decode('utf-8', errors='ignore').strip()
                    if not raw:
                        continue
                    parse_line(raw)
                    if raw.startswith('---'):
                        with state_lock:
                            ready = state['accel_x'] is not None
                        if ready:
                            print_state()
            except Exception as e:
                print(f"  [serial loop error] {e}")
    except serial.SerialException as e:
        print(f"\nFailed to connect: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if 'esp_serial' in locals() and esp_serial.is_open:
            esp_serial.close()
            print("Serial port closed.")

# ---- 3D visualisation ----------------------------------------
def build_viz():
    fig = plt.figure(figsize=(7, 6))
    ax  = fig.add_subplot(111, projection='3d')
    fig.patch.set_facecolor('#1a1a2e')
    ax.set_facecolor('#1a1a2e')
    ax.tick_params(colors='#888888')
    for spine in ax.spines.values():
        spine.set_edgecolor('#444444')

    PLOT_RANGE = 2.0   # metres each side of origin
    ax.set_xlim(-PLOT_RANGE, PLOT_RANGE)
    ax.set_ylim(-PLOT_RANGE, PLOT_RANGE)
    ax.set_zlim(-PLOT_RANGE, PLOT_RANGE)
    ax.set_xlabel('X (m)', color='#888888')
    ax.set_ylabel('Y (m)', color='#888888')
    ax.set_zlabel('Z (m)', color='#888888')
    ax.set_title('IMU Position & Orientation', color='white', pad=10)

    # Body-axis arrows scaled to ~15 % of the plot range so they're
    # visible without swamping the metre-scale position movement
    AXIS_LEN = 0.25
    quivers = [
        ax.quiver(0, 0, 0, AXIS_LEN, 0, 0, color='#ff4444', linewidth=2, arrow_length_ratio=0.2),
        ax.quiver(0, 0, 0, 0, AXIS_LEN, 0, color='#44ff44', linewidth=2, arrow_length_ratio=0.2),
        ax.quiver(0, 0, 0, 0, 0, AXIS_LEN, color='#4444ff', linewidth=2, arrow_length_ratio=0.2),
    ]

    poly = Poly3DCollection(
        [BASE_VERTS[f] for f in FACES],
        alpha=0.65,
        facecolor=FACE_COLORS,
        edgecolor='white',
        linewidth=0.5,
    )
    ax.add_collection3d(poly)

    title_text = ax.text2D(0.5, 0.96, "", transform=ax.transAxes,
                           ha='center', color='#aaaaaa', fontsize=9)

    def update(_frame):
        with state_lock:
            w  = state['quat_w'];  x  = state['quat_x']
            y  = state['quat_y'];  z  = state['quat_z']
            px = state['pos_x'];   py = state['pos_y'];  pz = state['pos_z']

        pos = np.array([px, py, pz])
        R   = quat_to_rot(w, x, y, z)

        # Rotate body-frame vertices then translate to world position
        rotated = BASE_VERTS @ R.T + pos

        poly.set_verts([rotated[f] for f in FACES])

        # Body-axis arrows: origin at pos, pointing along rotated body axes
        for q in quivers:
            q.remove()
        dirs = R @ np.eye(3)   # columns = rotated X, Y, Z unit vectors
        quivers[0] = ax.quiver(*pos, *dirs[:,0]*AXIS_LEN, color='#ff4444', linewidth=2, arrow_length_ratio=0.2)
        quivers[1] = ax.quiver(*pos, *dirs[:,1]*AXIS_LEN, color='#44ff44', linewidth=2, arrow_length_ratio=0.2)
        quivers[2] = ax.quiver(*pos, *dirs[:,2]*AXIS_LEN, color='#4444ff', linewidth=2, arrow_length_ratio=0.2)

        title_text.set_text(
            f"Pos ({px:.2f}, {py:.2f}, {pz:.2f}) m  |  "
            f"Q ({w:.2f}, {x:.2f}, {y:.2f}, {z:.2f})"
        )
        return [poly] + quivers

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
    plt.tight_layout()
    plt.show()
    return ani   # keep reference alive

# ---- Entry point ---------------------------------------------
if __name__ == '__main__':
    t = threading.Thread(target=serial_thread, daemon=True)
    t.start()
    ani = build_viz()   # blocks until window is closed