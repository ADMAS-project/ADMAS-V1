import React from "react";
import { Link } from "react-router-dom";  // استيراد Link من React Router
import logoHeader from "../../assets/logoHeader.png";
import "./Header.css";

const Header = () => {
  return (
    <header className="container">
      <div className="row align-items-center">
        <div className="col-12 col-md-4 text-md-start text-center mb-3 mb-md-0">
          <Link to="/emergency">
            <img src={logoHeader} alt="Logo Header" style={{ cursor: "pointer" }} />
          </Link>
        </div>
        <nav className="col-12 col-md-8 d-flex justify-content-center justify-content-md-end gap-4">
          <Link to="/emergency" className="custom-link fw-bold">
            Emergency
          </Link>
          <Link to="/history" className="custom-link">
            History
          </Link>
          <Link to="/contact" className="custom-link">
            Contact us
          </Link>
        </nav>
      </div>
    </header>
  );
};

export default Header;
