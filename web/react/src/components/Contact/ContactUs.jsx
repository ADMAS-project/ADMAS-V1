"use client";

import React from "react";
import "./contact.css"
import Header from "../Header/Header";
// مكون نموذج الاتصال
const ContactForm = () => {
  return (
    <form className="container mt-2 p-2 bg-white rounded-3">
      <fieldset className="border border-dark rounded-3 p-2">
        <div className="row mb-4">
          <div className="col-12">
            <input
              type="email"
              placeholder="Email"
              className="form-control border-dark rounded-pill"
              required
            />
          </div>
        </div>
        <div className="row mb-4">
          <div className="col-12">
            <input
              type="password"
              placeholder="Password"
              className="form-control border-dark rounded-pill"
              required
            />
          </div>
        </div>
        <div className="row mb-4">
          <div className="col-12">
            <textarea
              placeholder="Enter your message"
              className="form-control border-dark rounded-3"
              rows="6"
              required
            ></textarea>
          </div>
        </div>
        <div className="d-flex flex-wrap gap-3 justify-content-center">
          <button type="submit" className=" rounded-pill px-4"
            style={{
              background: "linear-gradient(90deg, #0D3B66 0%, #AA2DC7 100%)",
              color: "#fff"
            }}
          >
            Send
          </button>
          <button type="button" className=" rounded-pill px-4">
            Cancel
          </button>
        </div>
      </fieldset>
    </form>
  );
};

// مكون التنقل

// المكون الرئيسي
const ContactUs = () => {
  return (
    <div>
      <Header />
      <main className="container  w-100">

        <section className="row justify-content-center section">
          <div className="col-12 col-md-10 col-lg-5">
            <ContactForm />
          </div>
        </section>
      </main>
    </div>
  );
};

export default ContactUs;
