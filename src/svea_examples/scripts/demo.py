#! /usr/bin/env python3

import json
import requests
from pprint import pformat

from svea_core import rosonic as rx
from std_msgs.msg import String
from std_srvs.srv import Empty


class demo(rx.Node):
    """Call Ericsson APIs for demo."""

    DRYCALLS = rx.Parameter(False)
    SESSION_DURATION = rx.Parameter(60)  # seconds
    UE_IP = rx.Parameter("10.47.75.52")
    QOS_PROFILE = rx.Parameter("er_kip_4_10")

    pub_load_status = rx.Publisher(String, '/load/status')

    def on_startup(self):
        self.create_service(Empty, '/qod', self.qod_cb)
        self.create_service(Empty, '/load_on', self.load_on_cb)
        self.create_service(Empty, '/load_off', self.load_off_cb)
        self.create_service(Empty, '/load_status', self.load_status_cb)

    def qod_cb(self, req, rep):
        self.do_qod_request()
        self.get_logger().info('QoD Call!')
        return rep

    def load_status_cb(self, req, rep):
        self.do_load_status()
        self.get_logger().info('Load-Status Call!')
        return rep

    def load_on_cb(self, req, rep):
        self.do_load_request(enable=True)
        self.get_logger().info('Load-On Call!')
        return rep

    def load_off_cb(self, req, rep):
        self.do_load_request(enable=False)
        self.get_logger().info('Load-Off Call!')
        return rep

    def do_load_status(self) -> None:
        url = f"http://10.13.22.21:31396/congestion/status"

        call = dict(
            method='GET',
            url=url,
            headers={
                "Host": "er-k3s-husky.live.ericsson.net",
            },
            verify=False,
            timeout=15,
        )

        if self.DRYCALLS:
            self.get_logger().warn(pformat(call))
            return

        try:
            response = requests.request(**call)
        except requests.RequestException as exc:
            self.get_logger().warn(f"Request failed: {exc}")

        if not response.ok:
            self.get_logger().warn(f"Error: HTTP {response.status_code} {response.reason}")
        else:
            self.get_logger().info(pformat(response.json()))

            msg = String()
            msg.data = response.text
            self.pub_load_status(msg)

    def do_load_request(self, enable: bool) -> None:
        url = f"http://10.13.22.21:31396/congestion/{'start' if enable else 'stop'}"

        call = dict(
            method="POST",
            url=url,
            headers={
                "Content-Type": "application/json",
                "Host": "er-k3s-husky.live.ericsson.net",
            },
            verify=False,
            timeout=15,
        )

        if self.DRYCALLS:
            self.get_logger().warn(pformat(call))
            return

        try:
            response = requests.request(**call)
        except requests.RequestException as exc:
            self.get_logger().warn(f"Request failed: {exc}")

        if not response.ok:
            self.get_logger().warn(f"Error: HTTP {response.status_code} {response.reason}")

    def do_qod_request(self) -> None:
        url = f"http://10.13.22.21:31396/qod-proxy/sessions"

        call = dict(
            method="POST",
            url=url,
            headers={
                "Content-Type": "application/json",
                "Host": "er-k3s-husky.live.ericsson.net",
            },
            json={
                "duration": self.SESSION_DURATION,
                "ueId": {"ipv4addr": self.UE_IP},
                "asId": {"ipv4addr": "0.0.0.0/0"},
                "uePorts": {"ranges": [{"from": 1, "to": 65535}]},
                "asPorts": {"ranges": [{"from": 1, "to": 65535}]},
                "qos": self.QOS_PROFILE,
                "notificationUri": "http://10.4.128.10/echo-server/",
            },
            verify=False,
            timeout=15,
        )

        if self.DRYCALLS:
            self.get_logger().warn(pformat(call))
            return

        try:
            response = requests.request(**call)
        except requests.RequestException as exc:
            self.get_logger().warn(f"Request failed: {exc}")

        if not response.ok:
            self.get_logger().warn(f"Error: HTTP {response.status_code} {response.reason}")

if __name__ == '__main__':
    demo.main()

