import os
import time
import rclpy
from rclpy.node import Node
import requests
import json
from socket import *
from std_msgs.msg import String

TYPE = os.getenv("TYPE")

if TYPE != "TEST":
    from .gps import GPS


class Car:
    def __init__(self, color, model):
        self.color = color
        self.model = model


class HTTPReq(Node):
    def __init__(self, car, uuid, dest="172.17.0.1", dport=5000):
        super().__init__("HTTP_REQ")
        self.logger__ = self.get_logger()
        self.logger__.info("HTTP Node is ready")
        self.destination = dest
        self.dport = dport
        self.car = car
        self.uuid = uuid

        self.subscribe__ = self.create_subscription(
            String,
            "/face_monitor",
            self.emergency(self.uuid),
            10,
        )

    def emergency(self, uuid):
        if TYPE == "TEST":
            longitude = 20
            latitude = 155
        else:
            self.gps = GPS()
            coordinates = self.gps.read()
            longitude = coordinates["lon"]
            latitude = coordinates["lat"]

        def callback(msg: String):
            data = msg.data
            self.logger__.info(data)
            if data == "SLEEPING":
                return
            success = False
            while True:
                if not success:
                    success = self.send_sos(uuid, longitude, latitude)
                self.send_warning("DRVR_FAINTED")
                time.sleep(0.5)

        return callback

    def send_sos(self, uuid, longitude, latitude) -> bool:
        destination = f"http://{self.destination}:{self.dport}/cars/sos"
        data = {
            "uuid": uuid,
            "longitude": longitude,
            "latitude": latitude,
            "driver_status": "fainted",
            "color": self.car.color,
            "model": self.car.model,
        }

        headers = {"Content-Type": "application/json"}

        response = requests.request("POST", destination, json=data, headers=headers)

        obj = json.loads(response.text)
        return obj["status"] == "success"

    def send_warning(self, msg: str):
        s = socket(AF_INET, SOCK_DGRAM)
        s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
        s.bind(('', 5555))
        s.sendto(msg.encode(), ('<broadcast>', 5555))


def main(args=None):
    UUID = "bd21a29e-7521-400e-81c0-f35ea2de5c1c"
    car = Car("red", "lamborgini")
    try:
        rclpy.init(args=args)
        node = HTTPReq(car=car, uuid=UUID)
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        rclpy.shutdown()
        if TYPE != "TEST":
            node.gps.close()


if __name__ == "__main__":
    main()

