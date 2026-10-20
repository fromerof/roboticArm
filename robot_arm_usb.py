import serial
import time
import threading
import queue
import tkinter as tk
try:
    from PIL import Image, ImageTk
    import cv2
    from ultralytics import YOLO
    VISION_AVAILABLE = True
except ImportError:
    VISION_AVAILABLE = False
    print("[Warning] Faltan dependencias de vision (Pillow, opencv-python, ultralytics). Solo control manual disponible.")

# ── CONFIG ────────────────────────────────────────────────────────────────────
SERIAL_PORT      = 'COM3'                          # Cambiar segun puerto Arduino (ej: COM4, COM5)
BAUD_RATE        = 9600
ESP32_STREAM_URL = 'http://192.168.1.100:81/stream' # Cambiar a la IP del ESP32-CAM
YOLO_MODEL       = 'yolov8n.pt'                    # Se descarga automaticamente si no existe
STARTING_POS     = [90, 180, 140, 90, 50, 40]
# ─────────────────────────────────────────────────────────────────────────────

serial_queue    = queue.Queue()
frame_queue     = queue.Queue(maxsize=1)
detections      = []
detections_lock = threading.Lock()


# ── SERIAL THREAD ─────────────────────────────────────────────────────────────
class SerialWorker(threading.Thread):
    """Procesa comandos de serial_queue y espera ACK del Arduino (no bloquea la UI)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(2)

    def run(self):
        while True:
            command = serial_queue.get()
            try:
                self.arduino.write(command.encode())
                self._wait_ack()
            except Exception as e:
                print(f'[Serial] Error: {e}')

    def _wait_ack(self):
        while True:
            if self.arduino.in_waiting > 0:
                line = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                if line == 'ACK':
                    return
                # Las lineas de debug del Arduino (Raw input, Parsed...) se ignoran
            time.sleep(0.005)


# ── VIDEO / YOLO THREAD ───────────────────────────────────────────────────────
class VideoWorker(threading.Thread):
    """Captura frames del ESP32-CAM, corre YOLO y pone frames anotados en frame_queue."""

    def __init__(self):
        super().__init__(daemon=True)
        self.model = YOLO(YOLO_MODEL)

    def run(self):
        cap = cv2.VideoCapture(ESP32_STREAM_URL)
        if not cap.isOpened():
            print(f'[Video] No se pudo conectar a {ESP32_STREAM_URL}')
            return

        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue

            results = self.model(frame, verbose=False)

            with detections_lock:
                detections.clear()
                for box in results[0].boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls   = int(box.cls[0])
                    label = results[0].names[cls]
                    conf  = float(box.conf[0])
                    detections.append({'label': label, 'conf': conf, 'box': (x1, y1, x2, y2)})

            annotated = results[0].plot()
            img = Image.fromarray(cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)).resize((480, 360))

            # Descartar frame anterior si la UI no lo leyo aun
            if frame_queue.full():
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    pass
            frame_queue.put(img)

        cap.release()


# ── CONTROL ───────────────────────────────────────────────────────────────────
def send_angle(servo, angle):
    if 0 <= angle <= 180 and 1 <= servo <= 6:
        serial_queue.put(f'{servo}{angle}\n')
    else:
        print('Invalid angle or servo number')

def set_starting_position():
    for i, pos in enumerate(STARTING_POS):
        send_angle(i + 1, pos)

def increase_angle(servo, scale):
    new_angle = min(scale.get() + 10, 180)
    scale.set(new_angle)
    send_angle(servo, new_angle)

def decrease_angle(servo, scale):
    new_angle = max(scale.get() - 10, 0)
    scale.set(new_angle)
    send_angle(servo, new_angle)


# ── GRAB LOGIC (step 5 - pendiente) ──────────────────────────────────────────
def grab_detected_object():
    """
    TODO (step 5): Traducir bounding box de YOLO a angulos de servo y ejecutar agarre.
    """
    with detections_lock:
        current = list(detections)

    if not current:
        print('[Grab] No hay objetos detectados')
        return

    target = max(current, key=lambda d: d['conf'])
    x1, y1, x2, y2 = target['box']
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    print(f'[Grab] Target: {target["label"]} ({target["conf"]:.2f}) centro=({cx},{cy})')
    # TODO: mapear (cx, cy) a angulos de servo


# ── UI ────────────────────────────────────────────────────────────────────────
root = tk.Tk()
root.title('Robotic Arm Control')

# Video widget (solo si las dependencias estan disponibles)
if VISION_AVAILABLE:
    video_frame = tk.Frame(root, bg='black', width=480, height=360)
    video_frame.pack(pady=5)
    video_label = tk.Label(video_frame, bg='black', text='Esperando stream ESP32-CAM...', fg='white')
    video_label.pack(fill=tk.BOTH, expand=True)

# Controles de servo
controls_frame = tk.Frame(root)
controls_frame.pack(pady=5)

servo_scales = []
for i in range(6):
    row = tk.Frame(controls_frame)
    row.pack(pady=4)
    tk.Label(row, text=f'Servo {i+1}', width=8).pack(side=tk.LEFT)
    scale = tk.Scale(row, from_=0, to=180, orient=tk.HORIZONTAL, length=200)
    scale.set(STARTING_POS[i])
    tk.Button(row, text='◀', repeatdelay=50, repeatinterval=50,
              command=lambda s=i+1, sc=scale: decrease_angle(s, sc)).pack(side=tk.LEFT)
    scale.pack(side=tk.LEFT, padx=4)
    tk.Button(row, text='▶', repeatdelay=50, repeatinterval=50,
              command=lambda s=i+1, sc=scale: increase_angle(s, sc)).pack(side=tk.LEFT)
    tk.Button(row, text='Set', command=lambda s=i+1, sc=scale: send_angle(s, sc.get())).pack(side=tk.LEFT, padx=4)
    servo_scales.append(scale)

# Botones inferiores
bottom_frame = tk.Frame(root)
bottom_frame.pack(pady=8)
tk.Button(bottom_frame, text='Set Starting Position', command=set_starting_position).pack(side=tk.LEFT, padx=6)
if VISION_AVAILABLE:
    tk.Button(bottom_frame, text='Grab Object', command=grab_detected_object).pack(side=tk.LEFT, padx=6)


# ── VIDEO UPDATE LOOP ─────────────────────────────────────────────────────────
def update_video():
    try:
        img = frame_queue.get_nowait()
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk
        video_label.configure(image=imgtk, text='')
    except queue.Empty:
        pass
    root.after(30, update_video)  # ~33 fps


# ── START ─────────────────────────────────────────────────────────────────────
serial_worker = SerialWorker()
serial_worker.start()

if VISION_AVAILABLE:
    video_worker = VideoWorker()
    video_worker.start()
    update_video()

set_starting_position()
root.mainloop()
