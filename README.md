# 🚗 ADMAS – Advanced Driver Monitoring & Assistance System

> Graduation Project | SCU 2025  
> A real-time, AI-powered vehicle safety system that **detects fatigue**, **autonomously responds**, and **connects emergency services**.  

---

## Project Overview

ADMAS is a full-stack safety platform for vehicles, combining:

- **Real-time Driver Monitoring** – Fatigue detection via facial landmarks.
- **Traffic Sign & Lane Recognition** – YOLO-based detection + lane tracking.
- **GUI dashboard** – Displays signs, lane status, and fatigue alerts
- **Emergency Parking** –  auto-stopping if driver is unresponsive.
- **Alerts** – Sends live coordinates  + UDP to responders.
- **Live Web App** – Emergency dashboard with car status and live location.
- **Security** – Full DDS-level ROS 2 SROS2 encryption and permission control.

---

##  Architecture Breakdown

```bash
graduation-project/
├── ros_modules/           # All ROS 2 packages (ADMAS logic)
│   ├── face_monitoring_publisher/
│   ├── dashboard/
│   ├── lane_departure_warning/
│   ├── emergency_parking/
│   ├── communication/
│   ├── traffic_sign_detection/
│   ├── README.md                  # full ROS2 breakdown
│   └── security_policies/         #  [Detailed security README inside]
├── web/                   # Web emergency app
│   ├── backend/ (Go)              # Token auth, Datastorage management, REST API
│   ├── react/ (Frontend)         # Live responder dashboard
│   └── frontend/ (Vue)           # Early dashboard attempt (optional now)
├── firmware/              #  ESP32 for v2v in the other car
├── README.md              #  This file
└── requirements.txt     
````
---

# 🔵 ROS 2 Modules

- `face_monitoring_publisher/`: Dlib-based fatigue detection
- `dashboard/`:GUI Displays signs, lane status, and fatigue alerts
- `lane_departure_warning/`: Lane detection and alerting logic
- `traffic_sign_detection/`: YOLO-based traffic sign recognition
- `emergency_parking/`: Controls auto-parking on driver unresponsiveness
- `communication/`: Sends GPS & V2V/UDP alerts 


 For a full breakdown, visit [`ros_modules/README.md`](./ros_modules/README.md)

---

##  ROS 2 Security (SROS2)

*  All nodes use **ROS 2 SROS2** enclaves with permissions XML.
*  `ros_modules/security_policies/` includes:

  * XML policies per node
  * `security_policy.xsd` schema

 See the dedicated [`security_policies/README.md`](./ros_modules/security_policies/README.md) for setup instructions, permission structure, and validation flow.


---

##  Emergency Web app 

Real-time emergency response platform built with:

*  **Go** backend: Auth + Data Storage system (PostgreSQL or Redis) + RESTful API + SSE stream
*  **React** frontend: Live map, emergency resolution UI
*  Real-time updates via WebSockets
*  Full GPS distress flow

 Check `/web/` for full docs and folder-level READMEs.

---

##  More Docs Inside...

Every major folder has its **own README.md** – don’t miss them:

* `ros_modules/`: Node responsibilities, topics, security
* `web/backend/`: Go APIs + token auth
* `web/react/`: Emergency UI flow
* `firmware/`: ESP32 implementation
* `security_policies/`: All security logic and generation commands

This repo is modular, scalable, and fully documented for contributors, students, and engineers alike.

---

##  Final Words

**ADMAS isn’t just a graduation project.**
It’s a **real, working prototype** for future-ready, AI-driven, safety-first vehicles.
With full-stack integration, ROS 2 security, embedded autonomy, and real-time connectivity—it’s built to **save lives**.

---
##  License

This project is licensed under the [MIT License](LICENSE).  
© 2025 ADMAS SCU Graduation Team

