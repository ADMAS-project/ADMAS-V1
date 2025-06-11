"use client";

import React from "react";

export const DoneButton = () => {
    const handleDone = () => {
        console.log("Done clicked");
    };

    return (
        <div className="d-flex justify-content-center mt-5">
            <button
                onClick={handleDone}
                className="btn btn-primary fw-bold text-white px-5 py-3 fs-4 rounded-pill"
                style={{ maxWidth: "550px", width: "100%", backgroundColor: "#6f42c1", borderColor: "#6f42c1" }}
            >
                Done
            </button>
        </div>
    );
};
