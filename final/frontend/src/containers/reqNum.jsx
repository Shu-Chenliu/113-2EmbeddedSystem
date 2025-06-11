import { Box, Container, Paper, TextField, Typography, Button } from "@mui/material";
import { useEmbedded } from "../hooks/useEmbedded";
import CircularProgress from "@mui/material/CircularProgress";
export default function ReqNum(props) {
  const { pending, sendNum } = useEmbedded();
  const handleClick = (num) => {
    if (!num || num <= 0) {
      alert("Please enter a valid quantity.");
      return;
    }
    sendNum(num);
  };
  return (
    <>
      {pending ? (
        <CircularProgress />
      ) : (
        <Container maxWidth="xs">
          <Paper elevation={10} sx={{ padding: 2 }}>
            <Box sx={{ padding: 2 }}>
              <Typography component="h1" variant="h5" sx={{ textAlign: "center" }}>
                {/* 棧板資訊 */} Estimated Shipment Quantity
              </Typography>
            </Box>
            <Box
              component="form"
              noValidate
              sx={{ display: "flex", flexDirection: "column", alignItems: "center" }}
            >
              <TextField
                id="number-required"
                label="Quantity"
                variant="standard"
                type="number"
                onChange={(e) => {}}
                sx={{ width: "60%" }}
              />
            </Box>
            <Box sx={{ padding: 3 }}>
              <Button
                type="submit"
                variant="contained"
                fullWidth
                sx={{ mt: 1, width: "60%" }}
                onClick={() => handleClick(document.getElementById("number-required").value)}
              >
                Confirm
              </Button>
            </Box>
          </Paper>
        </Container>
      )}
    </>
  );
}
