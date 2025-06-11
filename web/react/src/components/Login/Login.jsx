"use client";

import React, { useState } from "react";
import { useNavigate } from "react-router-dom";
import logoHeader from "../../assets/logoHeader.png";

const LoginInput = ({ label, type, value, onChange, placeholder }) => (
    <div className="mb-3">
        <label className="form-label fs-6 text-dark">{label}</label>
        <input
            type={type}
            value={value}
            onChange={onChange}
            placeholder={placeholder}
            className="form-control"
        />
    </div>
);

const Login = () => {
    const [username, setUsername] = useState("");
    const [password, setPassword] = useState("");
    const [error, setError] = useState("");  // لتخزين رسالة الخطأ
    const navigate = useNavigate();

    const handleSubmit = async (e) => {
        e.preventDefault();

        try {
            const response = await fetch("http://localhost:5000/user/login", {
                method: "POST",
                headers: {
                    "Content-Type": "application/json",
                },
                body: JSON.stringify({
                    username: username,
                    password: password,
                }),
            });

            const resp = response;
            const data = await resp.json();
            document.cookie = `session=${data.token}`

            if (data.message === "success") {
                navigate("/emergency"); // إعادة التوجيه إلى صفحة الطوارئ
            } else if (data.status === "error") {
                setError(data.message); // عرض رسالة الخطأ القادمة من السيرفر
            }
        } catch (error) {
            console.log(error)
            setError("Something went wrong. Please try again."); // خطأ عام في حال فشل الطلب
        }
    };

    return (
        <div>
            <header className="mb-4 mt-4 px-4 text-start fs-4">
                <img src={logoHeader} alt="Logo Header" />
            </header>
            <main className="bg-light">
                <div className="container d-flex flex-column align-items-center">
                    <section className="container my-5">
                        <div className="row justify-content-center">
                            <div className="col-12 col-sm-10 col-md-8 col-lg-4">
                                <div className="card p-4 rounded-3 shadow-sm">
                                    <h2 className="text-center mb-4 fw-bold" style={{ color: "#5F28AE" }}>
                                        Welcome, please login
                                    </h2>
                                    <form onSubmit={handleSubmit}>
                                        <LoginInput
                                            label="User Name"
                                            type="text"
                                            value={username}
                                            onChange={(e) => setUsername(e.target.value)}
                                            placeholder="Enter your name"
                                        />
                                        <LoginInput
                                            label="Password"
                                            type="password"
                                            value={password}
                                            onChange={(e) => setPassword(e.target.value)}
                                            placeholder="Enter your password"
                                        />
                                        {error && (
                                            <div className="alert alert-danger mt-3" role="alert">
                                                {error}
                                            </div>
                                        )}
                                        <button
                                            type="button"
                                            className="btn btn-link p-0 float-end text-decoration-none"
                                            style={{ color: "#5F28AE" }}
                                            onClick={() => navigate("/forgot-password")}
                                        >
                                            Forget Password?
                                        </button>
                                        <button
                                            type="submit"
                                            className="btn btn-primary w-100 mt-3 rounded-pill"
                                            style={{ background: "linear-gradient(90deg, #0D3B66 0%, #AA2DC7 100%)" }}
                                        >
                                            Login
                                        </button>
                                    </form>
                                </div>
                            </div>
                        </div>
                    </section>
                </div>
            </main>
        </div>
    );
};

export default Login;
