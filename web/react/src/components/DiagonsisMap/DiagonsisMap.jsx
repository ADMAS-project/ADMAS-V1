import React from "react";

export const DiagnosisMap = () => {
    return (
        <section className="container text-center my-4">
            <div className="row justify-content-center">
                <div className="col-12 col-md-8 col-lg-6">
                    <img
                        loading="lazy"
                        src="https://cdn.builder.io/api/v1/image/assets/TEMP/0bd1db2508a89694a02c1d0b06d340a6d23779384a355f58de0338cc2df93554?placeholderIfAbsent=true&apiKey=6fca882fa0b4424e8afae39c209c0ca2"
                        alt="Emergency location map"
                        className="img-fluid rounded-3"
                    />
                </div>
            </div>
            <div className="row justify-content-center mt-2">
                <div className="col-12 col-md-8 col-lg-6 p-3 bg-white rounded-3 shadow border border-dark mx-auto " style={{ minHeight: "200px" }}>
                    <h2 className="fs-3 fw-bold">Preliminary Diagnosis:</h2>
                    <ul>
                        <li className="fs-4">Uncontrolled hypertension.</li>
                        <li className="fs-4">Possible myocardial infarction (heart attack).</li>
                    </ul>
                </div>

            </div>
        </section>
    );
};
