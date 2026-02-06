#!/usr/bin/env python3

import usb1
import time
import struct
import numpy as np
from threading import Lock

class HamamatsuTeensy:
  CUSTOM_INTERFACE = 2
  CONTROL_OUT_ENDPOINT = 5
  CONTROL_IN_ENDPOINT = 6
  BULK_IN_ENDPOINT = 7

  STRUCT_STATE = struct.Struct("<BIIBII")

  FAXITRON_STATE_WARMING_UP = "warming_up"
  FAXITRON_STATE_DOOR_OPEN = "door_open"
  FAXITRON_STATE_READY = "ready"

  FAXITRON_MODE_FRONT_PANEL = "front_panel"
  FAXITRON_MODE_REMOTE = "remote"

  # Full resolution: 2400x2320 (slight vertical crop), 12-bit packed
  FRAME_WIDTH = 2400
  FRAME_HEIGHT = 2320

  @staticmethod
  def unpack_12bit(packed_data):
    """Unpack 12-bit packed data to 16-bit array."""
    packed = np.frombuffer(packed_data, dtype=np.uint8)
    packed = packed[:len(packed) - len(packed) % 3].reshape(-1, 3)
    p0 = (packed[:, 0].astype(np.uint16)) | ((packed[:, 1].astype(np.uint16) & 0x0F) << 8)
    p1 = ((packed[:, 1].astype(np.uint16) >> 4)) | (packed[:, 2].astype(np.uint16) << 4)
    result = np.empty(len(p0) * 2, dtype=np.uint16)
    result[0::2] = p0
    result[1::2] = p1
    return result

  def __init__(self):
    self._handle = None
    self._serial_lock = Lock()
    self.connect()

  def connect(self):
    if self._handle is not None:
      self._handle.close()

    self._handle = usb1.USBContext().openByVendorIDAndProductID(0x16c0, 0x0483)
    assert self._handle is not None, "Could not find Teensy. Make sure it's connected."

    self._handle.claimInterface(HamamatsuTeensy.CUSTOM_INTERFACE)

    print("Connected to Teensy")

  def _control_out(self, data):
    return self._handle.bulkWrite(
      endpoint=HamamatsuTeensy.CONTROL_OUT_ENDPOINT,
      data=data,
    )

  def _control_in(self, size):
    return self._handle.bulkRead(
      endpoint=HamamatsuTeensy.CONTROL_IN_ENDPOINT,
      length=size,
    )

  def _bulk_in(self, size, timeout=30000):
    data = b""
    while len(data) < size:
      remaining = size - len(data)
      chunk = self._handle.bulkRead(
        endpoint=HamamatsuTeensy.BULK_IN_ENDPOINT,
        length=remaining,
        timeout=timeout,
      )
      if len(chunk) == 0:
        break
      data += chunk
    return data

  def _command(self, cmd, data=None):
    if data is None:
      data = b""
    self._control_out(struct.pack("<BI", cmd, len(data)) + data)
    resp = self._control_in(512)
    if len(resp) < 4:
      raise Exception("Invalid response length", len(resp))
    resp_len = struct.unpack("<I", resp[:4])[0]
    resp = resp[4:]
    while len(resp) < resp_len:
      resp += self._control_in(resp_len - len(resp))
    return resp

  def _faxitron_serial_command(self, data, no_lock=False):
    if no_lock:
      return self._command(0x10, data)
    else:
      with self._serial_lock:
        return self._command(0x10, data)

  def ping(self):
    dat = self._command(0x00, b"")
    assert len(dat) == 1, "Response does not match expected size"
    assert dat[0] == 0xA5, "Invalid ping response"

  def get_state(self):
    dat = self._command(0x01, b"")
    assert len(dat) == self.STRUCT_STATE.size, f"Response does not match expected struct size: {len(dat)} != {self.STRUCT_STATE.size}"
    dat_unpacked = self.STRUCT_STATE.unpack(dat)
    return {
      'state': dat_unpacked[0],
      'row': dat_unpacked[1],
      'col': dat_unpacked[2],
      'stop_reason': dat_unpacked[3],
      'dma_wait': dat_unpacked[4],
      'overhead_max_us': dat_unpacked[5],
    }

  def get_frame(self):
    dat = self._command(0x02, b"")
    assert len(dat) == 4, "Response does not match expected size"
    frame_len = struct.unpack("<I", dat[:4])[0]
    return self._bulk_in(frame_len)

  def start_trigger(self):
    dat = self._command(0x03)
    assert len(dat) == 1, "Response does not match expected size"
    if dat[0] != 0:
      raise Exception("Failed to start trigger, is another readout in progress?")

  def get_faxitron_state(self):
    dat = self._faxitron_serial_command(b"?S").decode()
    assert len(dat) == 3, "Response does not match expected size"
    if dat == "?SR":
      return self.FAXITRON_STATE_READY
    elif dat == "?SD":
      return self.FAXITRON_STATE_DOOR_OPEN
    elif dat == "?SW":
      return self.FAXITRON_STATE_WARMING_UP
    print(dat)
    return None

  def get_faxitron_exposure_time(self):
    dat = self._faxitron_serial_command(b"?T").decode()
    assert "?T" in dat, "Response does not match expected format"
    return int(dat[2:])/10

  def set_faxitron_exposure_time(self, exposure_time):
    assert 0 < exposure_time <= 99.9, "Exposure time must be between 0 and 99.9s"
    self._faxitron_serial_command(b"!T" + str(int(exposure_time * 10)).rjust(4, "0")[:4].encode())

  def get_faxitron_voltage(self):
    dat = self._faxitron_serial_command(b"?V").decode()
    assert "?V" in dat, "Response does not match expected format"
    return int(dat[2:])

  def set_faxitron_voltage(self, voltage):
    assert 0 < voltage <= 35, "Voltage must be between 0 and 35"
    self._faxitron_serial_command(b"!V" + str(voltage).rjust(2, "0")[:2].encode())

  def get_faxitron_mode(self):
    dat = self._faxitron_serial_command(b"?M").decode()
    assert len(dat) == 3, "Response does not match expected size"
    if dat == "?MF":
      return self.FAXITRON_MODE_FRONT_PANEL
    elif dat == "?MR":
      return self.FAXITRON_MODE_REMOTE
    return None

  def set_faxitron_mode(self, mode):
    assert mode in [self.FAXITRON_MODE_FRONT_PANEL, self.FAXITRON_MODE_REMOTE], "Invalid mode"
    modes = {
      self.FAXITRON_MODE_FRONT_PANEL: b"!MF",
      self.FAXITRON_MODE_REMOTE: b"!MR",
    }
    self._faxitron_serial_command(modes[mode])

  def perform_faxitron_exposure(self):
    with self._serial_lock:
      dat = self._faxitron_serial_command(b"!B", no_lock=True)
      assert dat == b"X", "Response does not match expected format"
      dat = self._faxitron_serial_command(b"C", no_lock=True)
      assert dat == b"P", "Response does not match expected format"
      while len(dat := self._faxitron_serial_command(b"", no_lock=True)) == 0:
        pass
      assert dat == b"S", "Response does not match expected format"

if __name__ == "__main__":
  hs = HamamatsuTeensy()
  hs.ping()

  # Take first frame (discard - sensor may be saturated)
  #print("Taking first frame (will be discarded)...")
  #hs.start_trigger()
  #while hs.get_state()['state'] != 3:
  #  time.sleep(0.1)
  #_ = hs.get_frame()  # Discard first frame

  # Wait 1 second for sensor to settle
  #print("Waiting 1 second...")
  #time.sleep(1.0)

  # Take second frame (keep this one)
  print("Taking second frame...")
  print("State:", hs.get_state())
  hs.start_trigger()
  while hs.get_state()['state'] != 3:
    time.sleep(0.1)

  # Unpack 12-bit data to 16-bit
  frame = HamamatsuTeensy.unpack_12bit(hs.get_frame()).reshape((HamamatsuTeensy.FRAME_HEIGHT, HamamatsuTeensy.FRAME_WIDTH))
  print("Frame:", frame.shape)

  frame = np.clip(frame, 0, 4095)

  # print(frame[100:200, 100:110])
  # for i in list(map(lambda i: bin(i), frame[110, 100:150].flatten())):
  #   print(i)

  import matplotlib.pyplot as plt
  plt.imshow(frame, cmap='gray')
  plt.show()

  # print("Faxitron state:", dalsa_teensy.get_faxitron_state())
  # dalsa_teensy.set_faxitron_exposure_time(30)
  # print("Faxitron exposure time:", dalsa_teensy.get_faxitron_exposure_time())
  # dalsa_teensy.set_faxitron_voltage(20)
  # print("Faxitron voltage:", dalsa_teensy.get_faxitron_voltage())
  # print("Faxitron mode:", dalsa_teensy.get_faxitron_mode())
  # dalsa_teensy.set_faxitron_mode(DalsaTeensy.FAXITRON_MODE_REMOTE)
  # print("Faxitron mode:", dalsa_teensy.get_faxitron_mode())
