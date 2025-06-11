import { useState } from "react";
import "./App.css";
import { BrowserRouter, Routes, Route } from "react-router-dom";
import Home from "./containers/home";
import Log from "./containers/log";
import ReqNum from "./containers/reqNum";
import { EmbeddedProvider } from "./hooks/useEmbedded";

function App() {
  return (
    <BrowserRouter>
      <EmbeddedProvider>
        <Routes>
          <Route path="/" element={<Home />} />
          <Route path="/log" element={<Log />} />
          <Route path="/reqNum" element={<ReqNum />} />
        </Routes>
      </EmbeddedProvider>
    </BrowserRouter>
  );
}

export default App;
