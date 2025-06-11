"use client";

import React from 'react';
import { useNavigate } from 'react-router-dom';  // استيراد useNavigate
import logoHeader from "../../assets/logoHeader.png";

const LoginValidationBox = () => {
    const navigate = useNavigate();  // تعريف useNavigate

    const handleValidate = () => {
        console.log('Validating token...');
        navigate("/emergency");  // الانتقال إلى صفحة Emergency بعد التحقق
    };

    return (
        <section className="row mt-5 bg-white rounded-3 p-4 w-100 w-lg-75">
            <article className="col-lg-4 col-md-8 col-sm-10 border border-dark rounded-3 py-5 px-4 mx-auto">
                <div className="text-center mb-4">
                    <p className="text-dark fs-5">
                        Use your secure token to confirm login
                    </p>
                </div>
                <button
                    onClick={handleValidate}
                    className="btn btn-dark w-100 py-2 fw-bold"
                >
                    Validate
                </button>
            </article>
        </section>
    );
};

const Validate = () => {
    return (
        <div>
            <header className="mb-4 mt-4 px-4 text-start fs-4">
                <img src={logoHeader} alt="Logo Header" />
            </header>
            <main className="d-flex flex-column align-items-start bg-light min-vh-100">
                <div className="w-100 w-lg-75 mx-auto">
                    <LoginValidationBox />
                </div>
            </main>
        </div>
    );
};

export default Validate;
