import React from "react";
import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import "bootstrap/dist/css/bootstrap.min.css";  // استيراد Bootstrap
import Login from "./components/Login/Login.jsx"
import Validate from "./components/EmergencyServiceLogin/Validate";
import History from "./components/History/History.jsx";
import Emergency from "./components/EmergencyService/Emergency.jsx";
import ContactUs from "./components/Contact/ContactUs";
import ForGotPassword from "./components/forgotPassword/ForGotPassword.jsx";

function App() {
  return (
    <Router>
      <Routes>
        <Route path="/" element={<Login />} />
        <Route path="/validate" element={<Validate />} />
        <Route path="/emergency" element={<Emergency />} />
        <Route path="/history" element={<History />} />
        <Route path="/contact" element={<ContactUs />} />
        <Route path="/forgot-password" element={<ForGotPassword />} />
      </Routes>
    </Router>
  );
}

export default App;
