import React, { useEffect, useState } from "react";
import PropTypes from "prop-types";

export const CarList = () => {
    const [cars, setCars] = useState([]); // قائمة السيارات

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

                // ✅ التحقق من أن `UUID` صحيح قبل إضافته
                if (newCarData.UUID && typeof newCarData.UUID === "string" && newCarData.UUID.length === 36) {
                    setCars((prevCars) => {
                        const newId = prevCars.length + 1;
                        return [...prevCars, { id: newId, name: `${newCarData.color}, ${newCarData.model}`, uuid: newCarData.UUID }];
                    });
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
        <div className="container border border-dark rounded-3 p-4" style={{ height: "90vh", overflowY: "auto" }}>
            <div className="row">
                {cars.map((car, index) => (
                    <div key={car.id} className="col-12 mb-3">
                        <CarItem name={car.name} uuid={car.uuid} isLast={index === cars.length - 1} />
                    </div>
                ))}
            </div>
        </div>
    );
};

// ✅ **تصحيح تمرير `uuid` إلى `CarItem` والتحقق من صحته**
const CarItem = ({ name, uuid, isLast }) => {
    console.log(`🔍 عرض السيارة: ${name}, UUID: ${uuid}`);

    const handleCheckStatus = async () => {
        if (!uuid || uuid.length !== 36) {
            console.warn("⚠️ لا يمكن جلب حالة السيارة، UUID غير صالح:", uuid);
            return;
        }

        try {
            const response = await fetch(`http://localhost:5000/view/${uuid}`);
            const data = await response.json();
            console.log(`🚗 حالة السيارة (${uuid}):`, data);
        } catch (error) {
            console.error("❌ خطأ في جلب بيانات السيارة:", error);
        }
    };

    return (
        <article className={`w-100 ${!isLast ? "mb-4" : ""}`}>
            <div className="d-flex flex-wrap justify-content-between align-items-center">
                <h2 className="fw-bold fs-4 mb-0" style={{ color: "#463689" }}>
                    {name} hahahah
                </h2>
                <div className="d-flex gap-3 align-items-center">
                    <button
                        type="button"
                        className="btn p-0 border-0"
                        onClick={handleCheckStatus}
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
                        className="btn p-0 border-0"
                        style={{
                            width: "30px",
                            height: "30px",
                            background: `url(https://cdn.builder.io/api/v1/image/assets/TEMP/dde7212dff190bebf24f53db88560aac3de6a595a609b29a2c8c95980fb22fc1) no-repeat center center`,
                            backgroundSize: "cover",
                        }}
                        aria-label="Car action button"
                    />
                </div>
            </div>
            <hr className="mt-3 border-dark" />
        </article>
    );
};

// ✅ **إضافة PropTypes للتأكد من صحة القيم**
CarItem.propTypes = {
    name: PropTypes.string.isRequired,
    uuid: PropTypes.string.isRequired,
    isLast: PropTypes.bool.isRequired,
};

export default CarList;
