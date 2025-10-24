# SPDX-FileCopyrightText: 2024 Volodymyr Shymanskyy
# SPDX-License-Identifier: Apache-2.0

import gc, sys, time, machine, json, asyncio
from umqtt.simple2 import MQTTClient, MQTTException

LOGO = r"""
      ___  __          __
     / _ )/ /_ _____  / /__
    / _  / / // / _ \/  '_/
   /____/_/\_, /_//_/_/\_\
          /___/
"""

def _noop(*_): pass


class BlynkDevice:
    def __init__(
        self,
        template_id: str,
        auth_token: str,
        broker: str | None = None,
        firmware_version: str = "0.0.1",
        cafile: str = "ISRG_Root_X1.der",
        on_connected=_noop,
        on_disconnected=_noop,
        on_message=_noop,
    ):
        print(LOGO)

        self.template_id = template_id
        self.auth_token = auth_token
        self.broker = broker
        self.firmware_version = firmware_version

        self.on_connected = on_connected
        self.on_disconnected = on_disconnected
        self.user_on_message = on_message

        self.connection_count = 0
        self._connected = False
        self._ssl_ctx = None

        if sys.platform in ("esp32", "rp2", "linux"):
            import ssl
            self._ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
            self._ssl_ctx.verify_mode = ssl.CERT_REQUIRED
            try:
                self._ssl_ctx.load_verify_locations(cafile=cafile)
            except Exception as e:
                # If CA isn't available, fall back to insecure (not recommended)
                print("CA load failed:", e)

        self._mqtt = MQTTClient(
            client_id="",
            server=self.broker,
            ssl=self._ssl_ctx,
            user="device",
            password=self.auth_token,
            keepalive=45,
        )
        self._mqtt.set_callback(self._on_message)
        self._mqtt.connect()

    # ---------- Public API ----------

    async def run(self):
        """Main loop; schedule this with asyncio.create_task(...) or await directly."""
        while True:
            await asyncio.sleep_ms(10)
            if not self._connected:
                if self._ssl_ctx:
                    # Ensure system time is sane before TLS
                    while not self._update_ntp_time():
                        await asyncio.sleep(1)
                try:
                    await self._mqtt_connect()
                    self._connected = True
                except Exception as e:
                    if isinstance(e, OSError):
                        print("Connection failed:", e)
                        await asyncio.sleep(5)
                    elif isinstance(e, AttributeError):
                        # Happens during reconnect on some ports of umqtt
                        pass
                    elif isinstance(e, MQTTException) and (e.value == 4 or e.value == 5):
                        print("Invalid BLYNK_AUTH_TOKEN")
                        await asyncio.sleep(15 * 60)
                    else:
                        sys.print_exception(e)
                        await asyncio.sleep(5)
            else:
                try:
                    self._mqtt.check_msg()
                except Exception:
                    self._connected = False
                    try:
                        self.on_disconnected()
                    except Exception as cb_e:
                        sys.print_exception(cb_e)

    def publish(self, topic: bytes, payload: str | bytes, retain: bool = False, qos: int = 0):
        self._mqtt.publish(topic, payload if isinstance(payload, (bytes, bytearray)) else str(payload), retain, qos)

    def subscribe(self, topic: bytes, qos: int = 0):
        self._mqtt.subscribe(topic, qos)

    def check_msg(self):
        self._mqtt.check_msg()

    def disconnect(self):
        try:
            self._mqtt.disconnect()
        except Exception:
            pass
        self._connected = False

    # ---------- Internals ----------

    async def _mqtt_connect(self):
        self.disconnect()
        gc.collect()
        print("Connecting to MQTT broker...")
        self._mqtt.connect()
        self._mqtt.subscribe("downlink/#")
        print("Connected to Blynk.Cloud", "[secure]" if self._ssl_ctx else "[insecure]")

        info = {
            "type": self.template_id,
            "tmpl": self.template_id,
            "ver": self.firmware_version,
            "rxbuff": 1024,
        }
        self._mqtt.publish("info/mcu", json.dumps(info))
        self.connection_count += 1
        try:
            self.on_connected()
        except Exception as cb_e:
            sys.print_exception(cb_e)

    def _on_message(self, topic, payload, ret, dup):
        topic = topic.decode("utf-8")
        payload = payload.decode("utf-8")
        print("msg")

        if topic == "downlink/redirect":
            _, host, port, _ = self._parse_url(payload)
            if host:
                self._mqtt.server = host
            if port:
                self._mqtt.port = port
            print("Redirecting...")
            self.disconnect()
        elif topic == "downlink/reboot":
            print("Rebooting...")
            machine.reset()
        elif topic == "downlink/ping":
            # umqtt handles QOS1 ping response
            pass
        else:
            try:
                self.user_on_message(topic, payload, ret, dup)
            except Exception as cb_e:
                sys.print_exception(cb_e)

    @staticmethod
    def _parse_url(url: str):
        # Returns (scheme, host, port, path)
        scheme = None
        host = None
        port = None
        path = ""
        try:
            scheme, rest = url.split("://", 1)
        except ValueError:
            rest = url
        try:
            netloc, path = rest.split("/", 1)
        except ValueError:
            netloc = rest
        if ":" in netloc:
            host, p = netloc.split(":", 1)
            try:
                port = int(p)
            except Exception:
                port = None
        else:
            host = netloc
        if port is None:
            port = 443 if (scheme and scheme.lower().startswith("ssl")) else 1883
        return scheme, host, port, path

    @staticmethod
    def _time2str(t):
        y, m, d, H, M, S, w, _ = t
        a = ("Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun")[w]
        return f"{a} {y}-{m:02d}-{d:02d} {H:02d}:{M:02d}:{S:02d}"

    @classmethod
    def _update_ntp_time(cls) -> bool:
        Jan24 = 756_864_000 if (time.gmtime(0)[0] == 2000) else 1_704_067_200
        if time.time() > Jan24:
            return True
        print("Getting NTP time...")
        import ntptime
        try:
            ntptime.timeout = 5
            ntptime.settime()
            if time.time() > Jan24:
                print("UTC Time:", cls._time2str(time.gmtime()))
                return True
        except Exception as e:
            print("NTP failed:", e)
        return False


# ---------- Minimal usage example ----------
# async def main():
#     def on_msg(topic, payload):
#         print("RX:", topic, payload)
#
#     dev = BlynkDevice(
#         template_id="YOUR_TEMPLATE_ID",
#         auth_token="YOUR_AUTH_TOKEN",
#         broker="blynk.cloud",        # or None to use redirect
#         firmware_version="0.0.1",
#         on_message=on_msg,
#     )
#     # dev.subscribe("uplink/#")  # if you need extra subs
#     await dev.run()
#
# asyncio.run(main())
