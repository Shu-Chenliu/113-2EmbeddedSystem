import AppBar from "@mui/material/AppBar";
import Box from "@mui/material/Box";
import Toolbar from "@mui/material/Toolbar";
import Typography from "@mui/material/Typography";
import IconButton from "@mui/material/IconButton";
import AccountCircle from "@mui/icons-material/AccountCircle";
import { useNavigate } from "react-router";
import { useEmbedded } from "../hooks/useEmbedded";
export default function SearchBar() {
  const { reset } = useEmbedded();
  const navigate = useNavigate();
  return (
    <Box sx={{ flexGrow: 1 }} onClick={() => navigate("/")}>
      <AppBar position="static">
        <Toolbar>
          <Box
            sx={{
              display: "flex",
              flexDirection: "row",
            }}
          >
            <IconButton
              sx={{ padding: "0", mr: "1rem" }}
              size="large"
              aria-label="account of current user"
              aria-controls="menu-appbar"
              aria-haspopup="true"
              color="inherit"
            >
              <AccountCircle />
            </IconButton>
          </Box>
          <Typography variant="h5" component="div" sx={{ flexGrow: 1 }}>
            Warehouse Management System
          </Typography>
        </Toolbar>
      </AppBar>
    </Box>
  );
}
