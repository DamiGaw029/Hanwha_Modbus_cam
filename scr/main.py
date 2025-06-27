from math import floor

import logging as logging
import threading
import time
import pypylon.pylon as pylon
import numpy as np
import cv2

from pymodbus.server import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusServerContext, ModbusSlaveContext
from pymodbus.datastore.store import ModbusSequentialDataBlock

# ----------------------------- Logger config -----------------------------
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)
logging.getLogger("pymodbus").setLevel(logging.WARNING)

# -------------------------- Init Modbus ----------------------
data_block = ModbusSequentialDataBlock(0, [0]*20)  # 20 rejestrów, wszystkie początkowo 0
store = ModbusSlaveContext(hr=data_block)
context = ModbusServerContext(slaves=store, single=True)

identity = ModbusDeviceIdentification()
identity.VendorName = 'Modbus'
identity.ProductCode = 'PM'
identity.VendorUrl = 'polsl'
identity.ProductName = 'Modbus Slave Dynamic'
identity.ModelName = 'Modbus TCP Server'
identity.MajorMinorRevision = '1.0'

# -------------------------- Start Modbus TCP -----------------------
def start_modbus_server():
    log.info("Launching Modbus TCP server on port 502...")
    StartTcpServer(context, identity=identity, address=("0.0.0.0", 502))
# ---------------------- Global variables with lock ----------------------
photopos_lock = threading.Lock()
photopos_value = 0
# ---------------------- Read registers 10–19 ----------------------
def read_modbus_inputs():
    global photopos_value
    while True:
        values = context[0].getValues(3, 10, count=10)
        for i, val in enumerate(values, start=10):
            print(f"[MODBUS] Register {i}: {val}")

        with photopos_lock:
            photopos_value = values[0]  # save var with lock

        time.sleep(0.25)

# Run server and read data in background
threading.Thread(target=start_modbus_server, daemon=True).start()
threading.Thread(target=read_modbus_inputs, daemon=True).start()

# ---------------------- Init cam via Pylon SDK --------------------
tlf = pylon.TlFactory.GetInstance()
cam = pylon.InstantCamera(tlf.CreateFirstDevice())
cam.Open()

cam.PixelFormat = "BGR8"
cam.Width.Value = 1280
cam.Height.Value = 720
cam.OffsetX = 1290
cam.OffsetY = 728

cam.Gain = 0
cam.ExposureTime = 1100  # µs

print("Resolution:", cam.Width.Value, "x", cam.Height.Value)

cam.StartGrabbing()

# --------------------- Main loop: frame operations -----------------------
while cam.IsGrabbing():
    with cam.RetrieveResult(2000) as result:
        if result.GrabSucceeded():
            frame = result.Array  # Grabbed frame

            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to gray
            frame_gray = cv2.medianBlur(frame_gray, 5)      # Filter

            circles = cv2.HoughCircles(
                frame_gray,
                cv2.HOUGH_GRADIENT,
                dp=1.2,
                minDist=30,
                param1=100,
                param2=30,
                minRadius=10,
                maxRadius=100
            )

            with photopos_lock: # locked var
                current_photopos = photopos_value

            if circles is not None and current_photopos == 1:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :1]:  # Only first circle
                    x, y, r = i
                    cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                    cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
                    context[0].setValues(3, 1, [x])
                    context[0].setValues(3, 2, [y])
                    context[0].setValues(3, 3, [r])
                    print(f"[CIRCLE] X={x}, Y={y}, R={r}")

                    break

            # elif True:  #for faster testing, without waiting for robot
            elif current_photopos == 2:
                detector = cv2.QRCodeDetector()
                retval, decoded_info, points, _ = detector.detectAndDecodeMulti(frame)

                if retval and points is not None and decoded_info:
                    for i, point_group in enumerate(points):
                        pts = point_group.astype(int)
                        x_coords = pts[:, 0]
                        y_coords = pts[:, 1]

                        # Dimensions in px
                        x_min, x_max = np.min(x_coords), np.max(x_coords)
                        y_min, y_max = np.min(y_coords), np.max(y_coords)
                        center_x = int(np.mean(x_coords))
                        center_y = int(np.mean(y_coords))

                        # QR dimensions in px
                        width_px = x_max - x_min
                        height_px = y_max - y_min

                        # QR dimensions in mm
                        ppm_x = 17 / width_px
                        width_mm = int(np.round(width_px * ppm_x))

                        # Center of frame
                        frame_center_x = cam.Width.Value // 2
                        frame_center_y = cam.Height.Value // 2

                        # Offset of QR from frame center
                        offs_x_px = center_x - frame_center_x
                        offs_y_px = center_y - frame_center_y

                        # Offset in [mm] in Modbus format
                        offs_x_mm = int(np.round(offs_x_px * ppm_x * 10) + 32768) # *10 to save first number after coma
                        offs_y_mm = int(np.round(offs_y_px * ppm_x * 10) + 32768) # +32768 to send negative numbers

                        # Show data in console
                        print(f"[QR] QR value: {decoded_info[i]}")
                        print(f"[QR] QR center: ({center_x}, {center_y})")
                        print(f"[QR] Frame center: ({frame_center_x}, {frame_center_y})")
                        print(f"[QR] QR offset: dx={offs_x_px} px ({offs_x_mm} mm), dy={offs_y_px} px ({offs_y_mm} mm)")

                        # Marks on frame
                        cv2.polylines(frame, [pts], isClosed=True, color=(255, 0, 0), thickness=2)
                        cv2.circle(frame, (center_x, center_y), 4, (0, 255, 0), -1)
                        cv2.circle(frame, (frame_center_x, frame_center_y), 4, (0, 0, 255), -1)  # środek kadru
                        qr_value = decoded_info[i]

                        try:
                            qr_int_value = int(decoded_info[0][0])
                        except (IndexError, ValueError):
                            qr_int_value = 0  # default value

                        # Set values to Modbus registers
                        context[0].setValues(3, 0, [1])              # Done OK
                        context[0].setValues(3, 1, [offs_x_mm])      # Offset X [mm]
                        context[0].setValues(3, 2, [offs_y_mm])      # Offset Y [mm]
                        context[0].setValues(3, 3, [center_x])       # QR center X [px]
                        context[0].setValues(3, 4, [center_y])       # QR center Y [px]
                        context[0].setValues(3, 5, [width_mm])       # QR width w mm
                        context[0].setValues(3, 6, [qr_int_value])  # QR text length

                        # Show values in window
                        cv2.putText(frame, qr_value, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (0, 255, 255), 1, cv2.LINE_AA)

                        break  # only first QR

            else: # if no image is analyzed set registers to 0
                context[0].setValues(3, 0, [0])  # Done OK
                context[0].setValues(3, 1, [0])  # Offset X [mm]
                context[0].setValues(3, 2, [0])  # Offset Y [mm]
                context[0].setValues(3, 3, [0])  # QR center X [px]
                context[0].setValues(3, 4, [0])  # QR center Y [px]
                context[0].setValues(3, 5, [0])  # QR width w mm
                context[0].setValues(3, 6, [0])  # QR text length

            cv2.imshow("Camera view", frame)

            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break

# ------------------------- Exit & close all -------------------------------
cam.StopGrabbing()
cam.Close()
cv2.destroyAllWindows()
