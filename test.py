import subprocess
import os

# bus_id = subprocess.Popen("lsusb | grep Camera", stdout=subprocess.PIPE)\
bus_id = subprocess.Popen("lsusb", stdout=subprocess.PIPE)
out = bus_id.stdout.read().decode('utf-8').split('\n')
for line in out:
    if "Camera" in line:
        out = line
        break
else:
    print("not found")

out = out.split()
bus_id, device_id = out[1], out[3]

cmd = "echo ubuntu | sudo -S /home/ubuntu/usb_reset /dev/bus/usb/%s/%s" % (bus_id, device_id)
os.system(cmd)