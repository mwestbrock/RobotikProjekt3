from platformio.public import DeviceMonitorFilterBase

class CobsMessageFilter(DeviceMonitorFilterBase):
  NAME = "cobs_message_filter"

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    print("Cobs message filter loaded. Will print line break at 00.")

  def rx(self, text):
    text = text.replace("00", "00\n")
    return text

  def tx(self, text):
    return text