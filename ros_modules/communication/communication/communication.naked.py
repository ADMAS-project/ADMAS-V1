import rclpy
from rclpy.node import Node
import requests
import json
from socket import *
from std_msgs.msg import String



class HTTPReq(Node):
    def __init__(self, dest="172.17.0.1", dport=5000):
        super().__init__("HTTP_REQ")
        self.logger__ = self.get_logger()
        self.logger__.info("HTTP Node is ready")
        self.destination = dest
        self.dport = dport

        self.subscribe__ = self.create_subscription(
            String, 
            "/face_monitor", 
            self.emergency("bd21a29e-7521-400e-81c0-f35ea2de5c1c"),
            10,
            )

    def emergency(self, uuid):
        # coordinates = self.gps.read()
        longitude = 2
        latitude = 3
        def callback(msg: String):
            data = msg.data
            self.logger__.info(data)
            while not self.send_sos(uuid, longitude, latitude):
                self.logger__.info("didn't pass")

            self.send_warning("DRVR_FAINTED")

        return callback

    def send_sos(self, uuid, longitude, latitude) -> bool:
        destination = "http://{}:{}/cars/sos".format(self.destination, self.dport)
        data = {
            "uuid": uuid,
            "longitude": longitude,
            "latitude": latitude,
            "driver_status": "fainted",
            }

        headers = {
            "Content-Type": "application/json"
        }

        response = requests.request("POST", destination, json=data, headers=headers)
        
        obj = json.loads(response.text)
        return obj["status"] == "success"

    def send_warning(self, msg: str):
        s = socket(AF_INET, SOCK_DGRAM)
        s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

        s.bind(('', 5555))

        s.sendto(msg.encode(), ('<broadcast>', 5555))
        




def main(args=None):
    try:
        rclpy.init(args=args)
        node = HTTPReq()
        rclpy.spin(node=node)

    except KeyboardInterrupt:
        rclpy.shutdown()
        node.gps.close()



if __name__ == "__main__":
    main()
