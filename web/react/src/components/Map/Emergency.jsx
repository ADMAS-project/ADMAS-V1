"use client";

import React from "react";
import { MapContainer, TileLayer, Marker, Popup } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import L from "leaflet";
import Header from "../Header/Header";
import "./Map.css";

// إعداد أيقونة Leaflet لتجنب مشاكل التحميل الافتراضي
const customIcon = new L.Icon({
    iconUrl: "https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png",
    iconSize: [25, 41],
    iconAnchor: [12, 41],
    popupAnchor: [1, -34],
});

export const Emergencys = () => {
    const handleDone = () => {
        console.log("Done clicked");
    };

    return (
        <main className="container-fluid bg-light">
            <div className="container text-center">
                <Header />

                {/* خريطة التشخيص مع التشخيص المبدئي */}
                <section className="container text-center">
                    <div className="row justify-content-center">
                        <div className="col-12 col-md-8 col-lg-6">
                            {/* خريطة Leaflet */}
                            <MapContainer
                                center={[30.0444, 31.2357]} // القاهرة كنموذج افتراضي
                                zoom={13}
                                style={{ height: "300px", width: "100%" }}
                                className="rounded-3 shadow border border-dark"
                            >
                                <TileLayer
                                    url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                                    attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
                                />
                                <Marker position={[30.0444, 31.2357]} icon={customIcon}>
                                    <Popup>Emergency Location</Popup>
                                </Marker>
                            </MapContainer>

                            <div
                                className="p-3 pb-0 bg-white rounded-3 shadow border border-dark mx-auto paragragh"
                                style={{ width: "90%" }}
                            >
                                <h2 className="fw-bold">Preliminary Diagnosis:</h2>
                                <ul className="list-unstyled fs-4 fs-sm-6">
                                    <li>Uncontrolled hypertension.</li>
                                    <li>Possible myocardial infarction (heart attack).</li>
                                </ul>
                            </div>
                        </div>
                    </div>
                </section>

                {/* زر الإنهاء */}
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

export default Emergencys;
