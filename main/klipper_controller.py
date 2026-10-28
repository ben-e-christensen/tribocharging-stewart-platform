"""
KlipperController - talks to the CNC router's Klipper/Moonraker instance.
Unchanged from the merged script, just moved into its own module.
"""

import requests


class KlipperController:
    def __init__(self, ip_address="127.0.0.1", port=7125):
        self.base_url = f"http://{ip_address}:{port}"

    def send_gcode(self, gcode_cmd):
        url = f"{self.base_url}/printer/gcode/script"
        try:
            response = requests.post(url, json={"script": gcode_cmd}, timeout=5)
            if response.status_code == 200:
                return True
            else:
                print(f"\n[!] KLIPPER ERROR: {response.text}\n")
                return False
        except requests.exceptions.RequestException:
            return False

    def get_position(self):
        url = f"{self.base_url}/printer/objects/query?motion_report=live_position"
        try:
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                data = response.json()
                pos = data['result']['status']['motion_report']['live_position']
                return pos[0], pos[1]
        except requests.exceptions.RequestException:
            pass
        return None, None

    def get_homed_axes(self):
        url = f"{self.base_url}/printer/objects/query?toolhead=homed_axes"
        try:
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                data = response.json()
                return data['result']['status']['toolhead']['homed_axes']
        except requests.exceptions.RequestException:
            pass
        return ""

    def is_fully_homed(self, axes="xy"):
        homed = self.get_homed_axes()
        return all(a in homed for a in axes)
