import { useContext, createContext, useState, useEffect, useRef, useCallback } from "react";
import { useNavigate } from "react-router";

const EmbeddedContext = createContext({
  pending: false,
  reset: () => {},
  getlog: () => {},
  trySetOut: () => {},
  isIn: false,
  sendNum: (num) => {},
  logData: [],
});
const EmbeddedProvider = (props) => {
  const navigate = useNavigate();
  const [pending, setPending] = useState(false); // check if the request is pending
  const [isIn, setIsIn] = useState(false); // check if the user is in the system
  const [logData, setLogData] = useState([]); // store log data
  const ws = useRef(null);
  const reset = useCallback(() => {
    console.log("reset");
    setPending(false);
    navigate("/");
    setLogData([]);
  }, [navigate]);

  useEffect(() => {
    ws.current = new WebSocket("ws://localhost:4000");

    ws.current.onopen = () => {
      console.log("WebSocket connected");
    };

    ws.current.onmessage = (event) => {
      const { data } = event;
      const { type, payload } = JSON.parse(data);
      console.log("Received:", type, payload);
      switch (type) {
        case "getLog":
          // handle log data
          console.log("Log data received:", payload);
          let payload2 = payload.slice().reverse(); // reverse the log data for display
          setLogData(payload2);
          setPending(false);
          navigate("/log");
          break;
        case "setOut":
          // handle set out response
          console.log("Set out response received:", payload);
          if (payload.success) {
            setIsIn(false);
            console.log("Set out successfully");
            navigate("/reqNum");
          } else {
            setIsIn(true);
          }
          setPending(false);
          break;
        case "setIn":
          console.log(payload);
          if (payload.start) {
            setIsIn(true);
            console.log("Set in start");
          } else {
            setIsIn(false);
            alert("Storing end");
            console.log("Set in end");
          }
          break;
        case "endOut":
          alert("Send out end");
          break;
        case "updateLog":
          // handle log update
          console.log("Log updated:", payload);
          setLogData((prevData) => [payload, ...prevData]);
          break;
        default:
          console.warn("Unknown message type:", type);
      }
    };

    ws.current.onclose = () => {
      console.log("WebSocket disconnected");
    };

    return () => {
      ws.current?.close();
    };
  }, []);

  const sendData = (payload) => {
    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      ws.current.send(JSON.stringify(payload));
    } else {
      console.warn("WebSocket is not open");
    }
  };

  const getlog = useCallback(() => {
    // get log
    console.log("get log");
    //setPending(true);
    navigate("/log");
    sendData({ type: "getLog" });
  }, [navigate]);

  const trySetOut = useCallback(() => {
    // try to set mode to out
    console.log("try set mode to out");
    setPending(true);
    sendData({ type: "setOut" });
  }, [navigate]);

  const sendNum = useCallback((num) => {
    // send the req number to backend
    console.log("send number:", num);
    sendData({ type: "sendNum", payload: { num: num } });
    alert(`Send out start`);
    sendData({ type: "getLog" });
    navigate("/log");
  }, []);

  return (
    <EmbeddedContext.Provider
      value={{
        pending,
        reset,
        getlog,
        trySetOut,
        isIn,
        sendNum,
        logData,
      }}
      {...props}
    />
  );
};
const useEmbedded = () => useContext(EmbeddedContext);
export { EmbeddedProvider, useEmbedded };
