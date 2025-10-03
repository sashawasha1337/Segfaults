import * as React from "react";
import { useNavigate } from "react-router-dom";

import { 
  Box,
  Container,
  Paper,
  Stack,
  Typography,
} from "@mui/material";
import BackButton from "../components/BackButton";
import EventTable from "../components/EventTable";
import { db } from "../firebaseConfig";
import { useAuth } from "../ContextForAuth";
import{
  collection,
  query,
  where,
  orderBy,
  limit,
  getDocs
} from "firebase/firestore"

function ActivityLogPage() {
  const navigate = useNavigate();
  const { currentUser } = useAuth();
  console.log("Current user object:", currentUser);
  const [events, setEvents] = React.useState([]);
  const [loading, setLoading] = React.useState(true);
  const [err, setErr] = React.useState("");

  const [sortField, setSortField] = React.useState("timeMS");
  const [sortDirection, setSortDirection] = React.useState("desc");

  const handleRequestSort = (field) => {
    setSortDirection((prev) => (prev === "asc" ? "desc" : "asc"));
    setSortField(field);
  };
  

React.useEffect(() => {
    const fetchEvents = async () => {
      if (!currentUser) {
        setLoading(false);
        return;
      }
      try {
        const q = query(
          collection(db, "events"),
          where("users", "array-contains", currentUser.uid),
          orderBy("time", "desc"),
          limit(200)
        );
        const snap = await getDocs(q);

        const rows = snap.docs.map((d) => {
          const data = d.data();
          const t = 
            data.time?.toDate?.() instanceof Date
              ? data.time.toDate()
              :typeof data.time === "string"
              ? new Date(data.time)
              : null;
          const formatted = 
            t && !isNaN(t)
              ? t.toLocaleString("en-US", {
                  year: "numeric",
                  month: "2-digit",
                  day: "2-digit",
                  hour: "2-digit",
                  minute: "2-digit",
                  hour12: false,
                }).replace(",", "")
              : "";
          return  {
            eventId: d.id,
            robotId: data.robotId ?? "Unknown",
            category: data.category ?? "Unknown",
            location: data.location ?? "Unknown",
            time: formatted,
            //timeMS to make sure sorting by time works correctly in event table
            timeMS: t instanceof Date ? t.getTime() : null,
          };
        });

        setEvents(rows);
              } catch (e) {
        console.error("Error fetching events:", e);
        setErr(e?.message || "Failed to load activity log.");
      } finally {
        setLoading(false);
      }
    };

    fetchEvents();
  }, [currentUser]);

  //here is the actual function where data is sorted based on set order
  const sortedEvents = React.useMemo(() => {
    const dir= sortDirection === "asc" ? 1 : -1;
    const get = (e) => {
      if (sortField === "timeMS") return e.timeMS ?? 0;
      if (sortField === "eventId") return e.eventId ?? "";
      if (sortField === "robotId") return e.robotId ?? "";
      if (sortField === "category") return e.category ?? "";
      if (sortField === "location") return e.location ?? "";
      return "";
    }
    //pass this return into sorted events
    return [...events].sort((a, b) => {
      //these interior returns tell the sort functin how to order the events
      const aValue = get(a);
      const bValue = get(b);
      if (aValue < bValue) return -1 * dir;
      if (aValue > bValue) return 1 * dir;
      return 0;
    });
  }, [events, sortField, sortDirection]);

  return ( //TODO: make this look nicer with MUI components probably
    <>  
      <Container maxWidth="lg" sx={{ pt: 10, pb: 6 }}>
        <BackButton path="/HomePage" />
        <Stack spacing={3} alignItems="center">
          <Typography variant="h3" fontWeight={800} textAlign="center">
            Activity Log
          </Typography>

          <Paper
            elevation={2}
            sx={{
              width: "100%",
              px: 3,       // horizontal padding
              py: 2,       // vertical padding
              borderRadius: 3
            }}
          >
            {loading ? (
              <Typography variant="body1">Loading…</Typography>
            ) : err ? (
              <Typography sx={{ color: "crimson", whiteSpace: "pre-wrap" }}>
                {err}
              </Typography>
            ) : events.length === 0 ? (
              <Typography>No activity found.</Typography>
            ) : (
              <EventTable
                events={sortedEvents}
                sortField={sortField}
                sortDirection={sortDirection}
                onRequestSort={handleRequestSort}
                sx={{
                  "& th, & td": { py: 1.5, px: 2 }, // extra cell padding
                }}
              />
            )}
          </Paper>
        </Stack>
      </Container>
    </>
  );
}

export default ActivityLogPage;
