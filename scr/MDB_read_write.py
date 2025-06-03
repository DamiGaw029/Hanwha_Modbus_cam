#########################################################################
#                                                                       #
#                           TESTING MODBUS LIB                          #
#                                                                       #                                                                 
#########################################################################
import logging
import threading
import time
from Tools.scripts.make_ctype import values
from pymodbus.server import ModbusTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusServerContext, ModbusSlaveContext
from pymodbus.datastore.store import ModbusSequentialDataBlock

# Logger config
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

# 20 registers (0–19)
initial_data = [0]*20
data_block = ModbusSequentialDataBlock(0, initial_data)
store = ModbusSlaveContext(hr=data_block)
context = ModbusServerContext(slaves=store, single=True)

# Server info
identity = ModbusDeviceIdentification()
identity.VendorName = 'Modbus'
identity.ProductCode = 'PM'
identity.VendorUrl = 'polsl'
identity.ProductName = 'Modbus Slave Dynamic'
identity.ModelName = 'Modbus TCP Server'
identity.MajorMinorRevision = '1.0'

# Read thread (reg1–reg10)
def read_registers():
    while True:
        values = context[0].getValues(3, 0, count=10)
        for i, val in enumerate(values):
            log.info(f"[ODCZYT] reg{i+1} = {val}")
        time.sleep(5)

#  Write thread (10–19 from shared_data)
shared_data = [0]*10  # registers 10–19

def write_registers():
    while True:
        context[0].setValues(3, 10, shared_data)
        for i, val in enumerate(shared_data):
            log.info(f"[ZAPIS] reg{10+i+1} ustawiono na: {val}")
        time.sleep(5)

#  Sever Modbus TCP as thread
def run_server():
    server = ModbusTcpServer(context, identity=identity, address=("0.0.0.0", 502))
    log.info("Serwer Modbus TCP uruchomiony w tle.")
    server.serve_forever()

# Run threads
#threading.Thread(target=read_registers, daemon=True).start()
#threading.Thread(target=write_registers, daemon=True).start()
#threading.Thread(target=run_server, daemon=True).start()

#th1= threading.Thread(target=read_registers())
#th1.daemon = True
#th1.start()

#th2= threading.Thread(target=write_registers())
#th2.daemon = True
#th2.start()

th3= threading.Thread(target=run_server())
th3.daemon = True
th3.start()



# -------------------------- MAIN THREAD --------------------------------
try:
    counter = 100
    while True:
        # Simulate value change
        for i in range(10):
            shared_data[i] = counter + i
        log.info(f"[MAIN] shared_data has been set to: {shared_data}")
        counter += 1
        time.sleep(5)
except KeyboardInterrupt:
    print("Close")
