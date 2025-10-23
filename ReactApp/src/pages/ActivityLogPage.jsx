import * as React from "react";
import { useNavigate } from "react-router-dom";
import IconButton from "@mui/material/IconButton";
import { 
  Box,
  Button,
  Container,
  Paper,
  Stack,
  Typography,
  Tooltip,
} from "@mui/material";
import RefreshIcon from '@mui/icons-material/Refresh';
import BackButton from "../components/BackButton";
import EventTable from "../components/EventTable";
import { db } from "../firebaseConfig";
import { useAuth } from "../ContextForAuth";
import{
  collection,
  query,
  doc,
  where,
  orderBy,
  limit,
  getDocs,
  getDoc
} from "firebase/firestore"

function ActivityLogPage() {
  const navigate = useNavigate();
  const { currentUser } = useAuth();
  const [events, setEvents] = React.useState([]);
  const [loading, setLoading] = React.useState(true);
  const [err, setErr] = React.useState("");
  const [sortField, setSortField] = React.useState("timeMS");
  const [sortDirection, setSortDirection] = React.useState("desc");
  const [lastRefresh, setLastRefresh] = React.useState(new Date()); 

  console.log("Current user object:", currentUser);

  const pageSize = 15; // number of events per page
  const [pageIndex, setPageIndex] = React.useState(0); // which page we are on

  // Helper function to get robot IDs for a user
  const getUserRobotIDs = async (email) => {
    if (!email) return [];
    const profileRef = doc(db, "profiles", email);
    const profileSnap = await getDoc(profileRef);
    if (!profileSnap.exists()) return [];
    const profileData = profileSnap.data();
    const ids = profileData?.robots || profileData.robotIds || [];
    if (!Array.isArray(ids))
      throw new Error("Invalid robot IDs format in user profile, check how robot is sending events.");
    return ids.filter(Boolean);
  };

  //splits array into chunks
  const chunkArray = (array, n = 10) => {
    const out = [];
    for (let i = 0; i < array.length; i += n) {
      out.push(array.slice(i, i + n));
    }
    return out;
  };


  // Add fetchEvents function outside useEffect so it can be reused
  const fetchEvents = async (resetPage = false) => {
    if (!currentUser) {
      setLoading(false);
      return;
    }
    try {
      setLoading(true);
      if (resetPage) setPageIndex(0); // Reset to first page on new fetch

      const robotIds = await getUserRobotIDs(currentUser.email);

      if (robotIds.length === 0) {
        setEvents([]);
        setErr("You don't have any robots associated with your account.");
        setLoading(false);
        console.log("No robots found for user.");
        return;
      }

      const idChunks = chunkArray(robotIds, 10); // Firestore 'in' queries support up to 10 values
      const all = [];

      for (const IDs of idChunks) {
        const q = query(
          collection(db, "events"),
          where("robotId", "in", IDs),
          orderBy("time", "desc"),
          limit(pageSize * (pageIndex + 1)) // fetch only enough for current page
        );
        const snap = await getDocs(q);
  
        const rows = snap.docs.map((d) => {
            const data = d.data();
            const imageUrl = data.image_url ?? null;
            const t = 
              data.time?.toDate?.() instanceof Date
                ? data.time.toDate()
                :typeof data.time === "string"
                ? new Date(data.time)
                : null;
                const isValid = t instanceof Date && !Number.isNaN(t.getTime());
                const formatted =   isValid
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
              //gonna comment out eventID for now since it looks ugly tbh, we should probably redesign how events are identified later
              //eventId: d.id,
              robotId: data.robotId ?? "Unknown",
              category: data.category ?? "Unknown",
              location: data.location ?? "Unknown",
              time: formatted,
              //timeMS to make sure sorting by time works correctly in event table
              timeMS: isValid ? t.getTime() : null,
              imageUrl,
            };
          });
  
          all.push(...rows);
        }
  
        setEvents(all);
        setLastRefresh(new Date());
      } catch (e) {
        console.error("Error fetching events:", e);
        setErr(e?.message || "Failed to load activity log.");
      } finally {
        setLoading(false);
      }
    };

  React.useEffect(() => {
    fetchEvents();

    // Set up auto-refresh every 30 seconds
    const intervalId = setInterval(() => {
      fetchEvents();
    }, 30000); // 30000 ms = 30 seconds

    return () => clearInterval(intervalId);
  }, [currentUser?.email, pageIndex]);

  const handleRequestSort = (field) => {
    setSortDirection((prev) => (prev === "asc" ? "desc" : "asc"));
    setSortField(field);
  };
  

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

  const start = pageIndex * pageSize;
  const end = start + pageSize;
  const pagedEvents = React.useMemo(
    () => sortedEvents.slice(start, end),
  [sortedEvents, start, end]
);
// simple flags for the pager buttons
  const hasPrev = pageIndex > 0;
  const hasNext = end < sortedEvents.length;

  return ( //TODO: make this look nicer with MUI components probably
    <>  
      <Container maxWidth="lg" sx={{ pt: 10, pb: 6 }}>
        <BackButton path="/HomePage" />
        <Stack spacing={3} alignItems="center">
          <Typography variant="h3" fontWeight={800} textAlign="center">
            Activity Log
          </Typography>

          <Box sx={{ width: '100%', display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
            <Tooltip title="Last updated at">
              <Typography variant="body2" color="text.secondary">
                Last refresh: {lastRefresh.toLocaleTimeString()}
              </Typography>
            </Tooltip>
            <IconButton 
              onClick={fetchEvents}
              disabled={loading}
              color="primary"
            >
              <RefreshIcon />
            </IconButton>
          </Box>

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
              <Box sx={{ textAlign: "center", py: 4 }}>
                <Typography variant="body1" color="text.secondary">
                  No events yet. Try refreshing or verifying your robot is publishing.
                </Typography>
              <Button sx={{ mt: 2 }} variant="outlined" onClick={() => fetchEvents(true)} disabled={loading}>
                Refresh
              </Button>
            </Box>
            ) : (
              <>
                <EventTable
                  events={pagedEvents}
                  sortField={sortField}
                  sortDirection={sortDirection}
                  onRequestSort={handleRequestSort}
                  sx={{
                    "& th, & td": { py: 1.5, px: 2 }, // extra cell padding
                  }}
                />
                <Box sx={{ display: "flex", justifyContent: "space-between", alignItems: "center", mt: 2 }}>
                  <Button
                    variant="outlined"
                    disabled={!hasPrev || loading}
                    onClick={() => setPageIndex((i) => Math.max(0, i - 1))}
                  >
                    Previous
                    </Button>

                <Typography variant="body2" color="text.secondary">
                  Page {pageIndex + 1} {sortedEvents.length > 0 ? `of ${Math.ceil(sortedEvents.length / pageSize)}` : ""}
                </Typography>

                <Button
                 variant="outlined"
                  disabled={!hasNext || loading}
                  onClick={() => setPageIndex((i) => i + 1)}
                >
                  Next
                </Button>
              </Box>
            </>
            )}
            </Paper>
          </Stack>
      </Container>
    </>
  );
}

export default ActivityLogPage;
