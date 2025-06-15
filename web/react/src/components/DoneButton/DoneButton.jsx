"use client";

import React, { useEffect } from 'react';

export const DoneButton = ({carUUID}) => {
    const handleDone = async () => {
            try {
                const response = await fetch("http://localhost:5000/cars/served", {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({uuid: carUUID, served: true})
                })
                const result = await response.json();
                console.log('Response:', result);
            } catch (error) {
                console.error('Fetch error:', error);
            }
        
        console.log("Done clicked");
    };

    return (
        <div className="d-flex justify-content-center mt-1">
            <button
                onClick={handleDone}
                className="btn btn-primary fw-bold text-white px-3 py-1 fs-5 rounded-pill"
                style={{ maxWidth: "550px", width: "100%", backgroundColor: "#6f42c1", borderColor: "#6f42c1" }}
            >
                Done
            </button>
        </div>
    );
};
