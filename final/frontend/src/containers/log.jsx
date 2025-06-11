import Box from "@mui/material/Box";
import { DataGridPro } from "@mui/x-data-grid-pro";
import { useDemoData } from "@mui/x-data-grid-generator";
import CircularProgress from "@mui/material/CircularProgress";
import SearchBar from "./searchBar";
import { useEmbedded } from "../hooks/useEmbedded";
import { useEffect, useState } from "react";
import { Hidden } from "@mui/material";
const col = [
  { field: "time", headerName: "Time stamp", width: 250 },
  { field: "uid", headerName: "ID", width: 180 },
  { field: "type", headerName: "Type", width: 110 },
  { field: "action", headerName: "Action", width: 110 },
];
function getRowId(row) {
  return row.id;
}

export default function DataGridProDemo() {
  const { pending, logData } = useEmbedded();
  const { data, loading } = useDemoData({
    dataSet: "Commodity",
    rowLength: 100000,
    editable: true,
  });
  console.log("logData", data);
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
          <Box sx={{ height: "100%", width: "100%" }}>
            <DataGridPro
              columns={col}
              rows={logData}
              getRowId={getRowId}
              loading={false}
              rowHeight={38}
              //checkboxSelection
              disableRowSelectionOnClick
            />
          </Box>
        )}
      </Box>
    </Box>
  );
}
