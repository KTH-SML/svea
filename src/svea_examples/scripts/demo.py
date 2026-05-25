#! /usr/bin/env python3

import json
import requests
from pprint import pformat

from svea_core import rosonic as rx
from std_srvs.srv import Empty


class demo(rx.Node):
    """Call Ericsson APIs for demo."""

    DRYCALLS = True
    SESSION_DURATION = rx.Parameter(60)  # seconds
    UE_IP = rx.Parameter("10.47.75.52")
    QOS_PROFILE = rx.Parameter("er_kip_4_10")

    def on_startup(self):
        self.create_service(Empty, '/qod', self.qod_cb)
        self.create_service(Empty, '/load', self.load_cb)

    def qod_cb(self, req, rep):
        self.do_request()
        self.get_logger().info('QoD Call!')
        return rep

    def load_cb(self, req, rep):
        self.do_request()
        self.get_logger().info('Start Load Call!')
        return rep

    def get_auth_header(self) -> dict[str, str]:
        """
        Replace this with your actual auth logic.

        Example:
            return {"Authorization": f"Bearer {token}"}
        """
        return {}
        raise NotImplementedError("Implement get_auth_header()")

    def do_request(self) -> None:
        url = f"http://10.13.22.21:31396/qod-proxy/sessions"

        call = dict(
            method="POST",
            url=url,
            headers={
                **self.get_auth_header(),
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
            # proxies=proxy and {'http': proxy, 'https': proxy},
            timeout=30,
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

