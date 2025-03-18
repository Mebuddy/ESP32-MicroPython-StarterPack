import network

ssid = "your_SSID"
password = "your_PASSWORD"

wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(ssid, password)

while not wifi.isconnected():
    pass

print("Connected:", wifi.ifconfig()[0])
