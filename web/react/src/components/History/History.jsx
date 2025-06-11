"use client";

import React, { useEffect, useState } from "react";
import Header from "../Header/Header";
import { MapContainer, TileLayer, Marker, Popup } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import L from "leaflet";
import "./Map.css";
import { useMap } from "react-leaflet";


const customIcon = new L.Icon({
    iconUrl: "https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png",
    iconSize: [25, 41],
    iconAnchor: [12, 41],
    popupAnchor: [1, -34],
});

const MapView = ({ lat, long, zoom }) => {
    const map = useMap();

    useEffect(() => {
        if (lat && long) {
            map.setView([lat, long], zoom); // استخدم zoom من state
        }
    }, [lat, long, zoom, map]);

    return null;
};

const History = () => {
    const [cars, setCars] = useState([]);
    const [selectedCar, setSelectedCar] = useState(null);
    const [lat, setLat] = useState(30.0444);
    const [long, setLong] = useState(31.2357);
    const [zoom, setZoom] = useState(5);
    useEffect(() => {
        const eventSource = new EventSource("http://localhost:5000/events?stream=messages", { withCredentials: true });

        eventSource.onmessage = (event) => {
            console.log("📩 بيانات جديدة من SSE:", event.data);

            if (event.data.trim() === "ping") {
                console.log("🔄 تم تجاهل رسالة ping");
                return;
            }

            try {
                const newCarData = JSON.parse(event.data);
                console.log("🚗 بيانات السيارة المستلمة:", newCarData);

                if (newCarData.UUID && typeof newCarData.UUID === "string" && newCarData.UUID.length === 36) {
                    setCars((prevCars) => [
                        ...prevCars,
                        {
                            id: prevCars.length + 1,
                            name: `Car ${newCarData.color}, ${newCarData.model}`,
                            uuid: newCarData.UUID,
                            latitude: newCarData.latitude,
                            longitude: newCarData.longitude
                        }
                    ]);
                } else {
                    console.warn("⚠️ UUID غير صالح أو مفقود:", newCarData.UUID);
                }
            } catch (error) {
                console.error("❌ خطأ في تحليل البيانات:", error);
            }
        };

        eventSource.onerror = (err) => {
            console.error("❌ خطأ في SSE:", err);
            eventSource.close();
            setTimeout(() => {
                window.location.reload();
            }, 3000);
        };

        return () => {
            eventSource.close();
        };
    }, []);


    return (
        <main className="container-fluid bg-light">
            <Header />
            <div className="row">
                <div className="col-md-4 p-3">
                    <CarList cars={cars} onSelectCar={(car) => {
                        setSelectedCar(car.uuid);
                        setLat(car.latitude);
                        setLong(car.longitude);
                    }} />
                </div>
                <div className="col-md-8 p-3">
                    <Map lat={lat} long={long} zoom={zoom} />
                </div>
            </div>
        </main>
    );
};

const CarList = ({ cars, onSelectCar }) => {
    return (
        <div className="container border border-dark rounded-3 p-4" style={{ height: "90vh", overflowY: "auto" }}>
            <div className="row">
                {cars.map((car, index) => (
                    <div key={car.id} className="col-12 mb-3">
                        <CarItem car={car} isLast={index === cars.length - 1} onSelectCar={onSelectCar} />
                    </div>
                ))}
            </div>
        </div>
    );
};

const CarItem = ({ car, isLast, onSelectCar }) => {
    return (
        <article className={`w-100 ${!isLast ? "mb-4" : ""}`}>
            <div className="d-flex flex-wrap justify-content-between align-items-center">
                <h2 className="fw-bold fs-4 mb-0" style={{ color: "#463689" }}>{car.name}</h2>
                <button
                    type="button"
                    className="btn p-0 border-0"
                    onClick={() => {
                        onSelectCar(car); // تحديد السيارة ونقل الخريطة إليها
                    }}
                    style={{
                        width: "40px",
                        height: "40px",
                        background: `url(https://cdn.builder.io/api/v1/image/assets/TEMP/017b4044c87041e2eae6b20a9c082e09e2320ba337a9a45fd053f8c4352034c4) no-repeat center center`,
                        backgroundSize: "cover",
                    }}
                    aria-label="Car status button"
                />
            </div>
            <hr className="mt-3 border-dark" />
        </article>
    );
};
const Map = ({ lat, long }) => {
    return (
        <MapContainer
            center={[lat, long]}
            zoom={5}
            style={{ height: "500px", width: "100%" }}
            className="rounded-3 shadow border border-dark"
        >
            <TileLayer
                url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            />
            <MapView lat={lat} long={long} />
            <Marker position={[lat, long]} icon={customIcon}>
                <Popup>Car Location</Popup>
            </Marker>
        </MapContainer>
    );
};

export default History
