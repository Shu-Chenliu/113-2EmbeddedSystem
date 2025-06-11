import React, { useEffect } from "react";
import styled from "styled-components";
import Box from "@mui/material/Box";
import { useNavigate } from "react-router";
import Button from "@mui/material/Button";
import Stack from "@mui/material/Stack";
import Typography from "@mui/material/Typography";
import { useEmbedded } from "../hooks/useEmbedded";
import CircularProgress from "@mui/material/CircularProgress";
import SearchBar from "./searchBar";

const Wrapper = styled.div`
  height: 50%;
  padding: 0px;
  &:hover {
    background-color: darkblue; /* Background color on hover */
    color: white; /* Text color on hover */
  }
`;
const hoverStyle = {
  backgroundColor: "#c5e1a5", // 滑鼠懸停時的背景色
  cursor: "pointer",
};

export default function Home(props) {
  // console.log(import.meta.env.VITE_Mapbox_API_Token);
  const [task, setTask] = React.useState(""); // default task
  const { pending, getlog, trySetOut, isIn } = useEmbedded();

  useEffect(() => {
    console.log("Task set to:", task);
    switch (task) {
      case "log":
        getlog();
        break;
      case "out":
        trySetOut();
        break;
    }
  }, [task]);

  return (
    <Box sx={{ width: "100%", padding: "0px", height: "100%" }}>
      <SearchBar />
      <Box
        sx={{ padding: "0" }}
        display="flex"
        justifyContent="center"
        alignItems="center"
        height="80vh"
      >
        {pending ? (
          <CircularProgress />
        ) : (
          <>
            <Box
              spacing={2}
              sx={{
                width: 1 / 2,
                display: "flex",
                justifyContent: "center",
                alignItems: "center",
                height: "100%",
                borderRight: "4px #1976D2 solid",
                borderBottom: "4px #1976D2 solid",
                "&:hover": hoverStyle,
              }}
              onClick={() => setTask("log")}
            >
              <Typography variant="h4" component="div" sx={{ flexGrow: 1 }}>
                進出貨記錄
              </Typography>
            </Box>
            <Box
              spacing={2}
              sx={{
                width: 1 / 2,
                display: "flex",
                justifyContent: "center",
                alignItems: "center",
                height: "100%",
                borderRight: "4px #1976D2 solid",
                borderBottom: "4px #1976D2 solid",
                "&:hover": isIn ? undefined : hoverStyle,
              }}
              onClick={isIn ? undefined : () => setTask("out")}
            >
              <Typography variant="h4" component="div" sx={{ flexGrow: 1 }}>
                {isIn ? "正在進貨中..." : "出貨"}
              </Typography>
            </Box>
          </>
        )}
      </Box>
    </Box>
  );
}
