"use client";

import React from "react";
import { useNavigate } from "react-router-dom";  // استيراد useNavigate
import logoHeader from '../../assets/logoHeader.png';

const ForGotPassword = () => {
    const navigate = useNavigate();  // تعريف useNavigate

    const handleBackToLogin = () => {
        navigate("/");  // الانتقال إلى صفحة تسجيل الدخول
    };

    return (
        <div>
            <header className="mb-4 mt-4 px-4 text-start fs-4">
                <img src={logoHeader} alt="Logo Header" />
            </header>
            <main className="container py-5">
                <div className="row justify-content-center">
                    <div className="col-12 col-md-10 col-lg-4">
                        <section className="card p-4 mb-4">
                            <div className="card-body">
                                <div className="text-center">
                                    <header className="d-flex align-items-center justify-content-center mb-4 text-primary">
                                        <img
                                            loading="lazy"
                                            src="https://cdn.builder.io/api/v1/image/assets/TEMP/cad4e9c70f5675453783138b16c0d56ed9d878200efd26edaf3f93a035be19ff?placeholderIfAbsent=true&apiKey=6fca882fa0b4424e8afae39c209c0ca2"
                                            className="img-fluid me-3"
                                            alt="Forgot password icon"
                                            style={{ width: "40px", height: "40px" }}
                                        />
                                        <h1 style={{ color: "#AA2DC7" }} className="h5 mb-0">Forget your password?</h1>
                                    </header>
                                    <p className="mb-4 text-muted">
                                        We're sorry, password reset
                                        <br /> is not available online.
                                        <br />
                                        Please visit the administration office
                                        <br /> to get a new password.
                                    </p>
                                    <button
                                        onClick={handleBackToLogin}
                                        className="btn w-100 text-white border-0 rounded-pill"
                                        style={{
                                            background: "linear-gradient(90deg, #0D3B66 0%, #AA2DC7 100%)",
                                        }}
                                    >
                                        Back to login
                                    </button>
                                </div>
                            </div>
                        </section>
                    </div>
                </div>
            </main>
        </div>
    );
};

export default ForGotPassword;
