#! /usr/bin/env python3

import json
import requests
from pprint import pformat

from svea_core import rosonic as rx
from std_srvs.srv import Empty


class demo(rx.Node):
    """Call Ericsson APIs for demo."""

    DRYCALLS = True

    def on_startup(self):
        self.create_service(Empty, '/qod', self.qod_cb)
        self.create_service(Empty, '/load', self.load_cb)

    def qod_cb(self, req, rep):
        self.get_logger().info('QoD Call!')
        return rep

    def load_cb(self, req, rep):
        self.get_logger().info('Start Load Call!')
        return rep

    def get_auth_header(self) -> dict[str, str]:
        """
        Replace this with your actual auth logic.

        Example:
            return {"Authorization": f"Bearer {token}"}
        """
        raise NotImplementedError("Implement get_auth_header()")

    def do_request(
        self,
        path: str,
        body: dict | None,
        *,
        cces_ip: str,
        cces_hostname: str,
        ca_cert: str | None = None,
        proxy: str | None = None,
    ) -> None:
        url = f"https://{cces_ip}:443/hub/qod/v0/sessions{path}"

        call = dict(
            method="POST",
            url=url,
            headers={
                **get_auth_header(),
                "Content-Type": "application/json",
                "Host": cces_hostname,
            },
            json={
                "duration": SESSION_DURATION,
                "ueId": {"ipv4addr": UE_IP},
                "asId": {"ipv4addr": "0.0.0.0/0"},
                "uePorts": {"ranges": [{"from": 1, "to": 65535}]},
                "asPorts": {"ranges": [{"from": 1, "to": 65535}]},
                "qos": QOS_PROFILE,
                "notificationUri": NOTIFICATION_URL,
            },
            verify=False,
            proxies=proxy and {'http': proxy, 'https': proxy},
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

