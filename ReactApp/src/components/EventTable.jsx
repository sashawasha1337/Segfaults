import React from "react";
import { useNavigate } from "react-router-dom";

const EventTable = ({ events, sortField, sortDirection, onRequestSort }) => {
  const navigate = useNavigate();

  const handleViewDetails = (event) => {
    navigate("/TrashViewPage", {
      state: {
        eventId: event.id ?? event.eventId,
        confidence: event.confidence ?? null,
        imageUrl: event.imageUrl ?? null,
        robotId: event.robotId ?? "Unknown",
        category: event.category ?? "Unknown",
        location: event.location ?? "Unknown",
        timeMS: event.timeMS ?? null,
      },
    });
  };

  const SortingButton = ({ field, label }) => {
    const active = field === sortField;
    const dir = active ? sortDirection : "asc";
    const arrow = active ? (dir === "asc" ? "↑" : "↓") : "";

    return (
      <th
        onClick={() => onRequestSort(field)}
        aria-sort={active ? dir : "none"}
        style={{
          padding: "14px 18px",
          background: "#f3f3f3",
          fontWeight: 600,
          cursor: "pointer",
          textAlign: "center",
          borderRadius: "6px",
          fontSize: "0.95rem",
        }}
      >
        {label} {arrow}
      </th>
    );
  };

  return (
    <table
      style={{
        width: "100%",
        borderCollapse: "separate",
        borderSpacing: "0 12px",
        fontSize: "0.95rem",
      }}
    >
      <thead>
        <tr>
          <SortingButton field="robotId" label="Robot ID" />
          <SortingButton field="category" label="Category" />
          <SortingButton field="location" label="Location" />
          <SortingButton field="timeMS" label="Time" />
          <th
            style={{
              padding: "14px 18px",
              background: "#f3f3f3",
              fontWeight: 600,
              borderRadius: "6px",
              textAlign: "left",
            }}
          >
            Details
          </th>
        </tr>
      </thead>

      <tbody>
        {events.map((event) => (
          <tr
            key={event.eventId}
            style={{
              background: "white",
              boxShadow: "0 2px 6px rgba(0,0,0,0.08)",
              borderRadius: "8px",
            }}
          >
            <td style={{ padding: "14px 18px" }}>{event.robotId}</td>
            <td style={{ padding: "14px 18px" }}>{event.category}</td>
            <td style={{ padding: "14px 18px" }}>{event.location}</td>
            <td style={{ padding: "14px 18px" }}>{event.time}</td>

            <td style={{ padding: "14px 18px" }}>
              <button
                onClick={() => handleViewDetails(event)}
                style={{
                  background: "black",
                  color: "white",
                  border: "none",
                  padding: "8px 14px",
                  borderRadius: "6px",
                  cursor: "pointer",
                }}
              >
                View Details
              </button>
            </td>
          </tr>
        ))}
      </tbody>
    </table>
  );
};

export default EventTable;