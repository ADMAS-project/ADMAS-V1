
# 🔒 ADMAS ROS 2 Security Policies

This directory contains all SROS2 security policy definitions used to enforce **strict node-level access control** in the ADMAS system.

Each `.xml` file defines **fine-grained permissions** for a ROS 2 node, including allowed topics for publishing and subscribing. These are signed and deployed into SROS2 enclaves to enforce DDS security at runtime.

---

## Included Policies

| Node                       | Enclave Name                | Permissions                                                    |
|----------------------------|-----------------------------|----------------------------------------------------------------|
| `face_monitoring_publisher`| `/face_monitoring_publisher`| Publish `/face_monitor`                                        |
| `dashboard`                | `/dashboard`                | Subscribe `/face_monitor`, `/lane_warning`, `/road_sign_topic` |
| `communication`            | `/communication`            | Subscribe `/face_monitor`                                      |
| `emergency_parking`        | `/emergency_parking`        | Subscribe `/face_monitor`                                      |
| `lane_departure_warning`   | `/lane_departure_warning`   | Publish `/lane_warning`                                        |
| `traffic_sign_detection`   | `/traffic_sign_detection`   | Publish `/road_sign_topic`                                     |
| `security_policy.xsd`      | —                           | XML Schema for validating all policies                         |

---

## Usage: Generate Permission Files

From inside this folder:

```bash
ros2 security create_permission ../sros2_keystore /face_monitoring_publisher face_monitoring_publisher.xml
ros2 security create_permission ../sros2_keystore /dashboard dashboard.xml
ros2 security create_permission ../sros2_keystore /communication communication.xml
ros2 security create_permission ../sros2_keystore /emergency_parking emergency_parking.xml
ros2 security create_permission ../sros2_keystore /lane_departure_warning lane_departure_warning.xml
ros2 security create_permission ../sros2_keystore /traffic_sign_detection traffic_sign_detection.xml
````

> Requires `security_policy.xsd` in this directory for schema validation.

---

##  Security Architecture

* **SROS2 Enclaves**: Each node runs in a dedicated enclave with signed certs and permissions.
* **Topic-Level Isolation**: No node can access a topic unless explicitly authorized.
* **Schema-Driven Policies**: Validated against `security_policy.xsd` to enforce consistency and correctness.
* **No Trust Assumed**: Only nodes with valid identity & permissions may participate in DDS.

---

## Pro Tips

* Use `--ros-args --enclave /node_name` or set `ROS_SECURITY_ENCLAVE` to load policies correctly.
* Add `ROS_SECURITY_ENABLE=true` and `ROS_SECURITY_STRATEGY=Enforce` to your launch setup or `.bashrc`.

---

##  Maintained by
╭────────────────────────────────────────────╮
│                                            │
│       Ahmad Alaa (**@0xAhmadAlaa**)        │
│                                            │
╰────────────────────────────────────────────╯

