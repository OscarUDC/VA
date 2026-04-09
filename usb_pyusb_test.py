import usb.core
import usb.backend.libusb1
import os

# Fuerza backend usando libusb incluida en libusb-package
backend = usb.backend.libusb1.get_backend()

print("Backend:", backend)

devs = list(usb.core.find(find_all=True, backend=backend))
print("Dispositivos encontrados:", len(devs))

for d in devs:
    print(hex(d.idVendor), hex(d.idProduct))