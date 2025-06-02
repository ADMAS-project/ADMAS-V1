"use client";

import React, { useEffect, useState } from "react";
import PropTypes from "prop-types";
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
            map.setView([lat, long], zoom); // استخدام التكبير الديناميكي
        }
    }, [lat, long, zoom, map]);

    return null;
};
const Emergency = () => {
    const [cars, setCars] = useState([]);
    const [selectedCar, setSelectedCar] = useState(null);
    const [lat, setLat] = useState(null);
    const [long, setLong] = useState(null);
    const [zoom, setZoom] = useState(5);


    const removeCar = (uuid) => {
        setCars((prevCars) => prevCars.filter(car => car.uuid !== uuid));
    };

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
            {selectedCar ? (
                <Map selectedCar={selectedCar} lat={lat} long={long} zoom={zoom} onBack={() => setSelectedCar(null)} />


            ) : (
                <section className="row d-flex justify-content-center">
                    <div className="col-lg-6 col-md-8 col-sm-10 p-3 rounded-3">
                        <CarList
                            cars={cars}
                            onSelectCar={(car) => {
                                setSelectedCar(car.uuid);
                                setLat(car.latitude);
                                setLong(car.longitude);
                            }}
                            onRemoveCar={removeCar}
                        />
                    </div>
                </section>
            )}
        </main>
    );
};

const CarList = ({ cars, onSelectCar, onRemoveCar }) => {
    return (
        <div className="container border border-dark rounded-3 p-4" style={{ height: "90vh", overflowY: "auto" }}>
            <div className="row">
                {cars.map((car, index) => (
                    <div key={car.id} className="col-12 mb-3">
                        <CarItem car={car} isLast={index === cars.length - 1} onSelectCar={onSelectCar} onRemoveCar={onRemoveCar} />
                    </div>
                ))}
            </div>
        </div>
    );
};

const CarItem = ({ car, isLast, onSelectCar, onRemoveCar }) => {
    return (
        <article className={`w-100 ${!isLast ? "mb-4" : ""}`}>
            <div className="d-flex flex-wrap justify-content-between align-items-center">
                <h2 className="fw-bold fs-4 mb-0" style={{ color: "#463689" }}>{car.name}</h2>
                <div className="d-flex gap-3 align-items-center">
                    <button
                        type="button"
                        className="btn p-0 border-0"
                        onClick={() => onSelectCar(car)}
                        style={{
                            width: "40px",
                            height: "40px",
                            background: `url(https://cdn.builder.io/api/v1/image/assets/TEMP/017b4044c87041e2eae6b20a9c082e09e2320ba337a9a45fd053f8c4352034c4) no-repeat center center`,
                            backgroundSize: "cover",
                        }}
                        aria-label="Car status button"
                    />
                    <button
                        type="button"
                        className="btn  d-flex align-items-center justify-content-center"
                        style={{ width: "40px", height: "40px" }}
                        onClick={() => onRemoveCar(car.uuid)}
                        aria-label="Remove car"
                    >
                        ❌
                    </button>
                </div>
            </div>
            <hr className="mt-3 border-dark" />
        </article>
    );
};

const Map = ({ selectedCar, lat, long, onBack, zoom }) => {
    const handleDone = () => {
        console.log("Done clicked");
        onBack(); // العودة إلى قائمة السيارات
    };

    return (
        <main className="container-fluid bg-light">
            <div className="container text-center">
                <section className="container text-center">
                    <div className="row justify-content-center">
                        <div className="col-12 col-md-8 col-lg-6">
                            <MapContainer
                                center={[lat || 30.0444, long || 31.2357]}
                                zoom={zoom}
                                style={{ height: "300px", width: "100%" }}
                                className="rounded-3 shadow border border-dark"
                            >
                                <TileLayer
                                    url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                                    attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
                                />
                                {lat && long && (
                                    <>
                                        <MapView lat={lat} long={long} />
                                        <Marker position={[lat, long]} icon={customIcon}>
                                            <Popup>Emergency Location</Popup>
                                        </Marker>
                                    </>
                                )}
                            </MapContainer>

                            <div className="p-3 pb-0 bg-white rounded-3 shadow border border-dark mx-auto paragragh" style={{ width: "90%" }}>
                                <h2 className="fw-bold">Preliminary Diagnosis:</h2>
                                <ul className="list-unstyled fs-4 fs-sm-6">
                                    <li>Uncontrolled hypertension.</li>
                                    <li>Possible myocardial infarction (heart attack).</li>
                                </ul>
                            </div>
                        </div>
                    </div>
                </section>

                <div className="d-flex justify-content-center pt-2">
                    <button
                        onClick={handleDone}
                        className="btn fw-bold text-white fs-4 rounded-pill"
                        style={{
                            background: "linear-gradient(90deg, #0D3B66 0%, #AA2DC7 100%)",
                            border: "none",
                            width: "44%",
                        }}
                    >
                        Done
                    </button>
                </div>
            </div>
        </main>
    );
};


export default Emergency;
