import React, { Children } from 'react';
import { useNavigate } from "react-router-dom";

//const [sortField, setSortField] = React.useState<"timeMs" | "eventId" | "robotId" | "category" | "location">("timeMS");
//const [sortDirection, setSortDirection] = React.useState<"asc" | "desc">("desc");



const EventTable = ({events, sortField, sortDirection, onRequestSort, }) => {
  const navigate=useNavigate();
  const handleViewDetails = (event) => {
    navigate('/TrashViewPage', {
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
    }
  // this defines labels which manage arrows indicating sorting direction and field 
  const SortingButton=({field, label})=>{
    const activeField=field===sortField;
    const direction=activeField?sortDirection:"asc";
    const arrow = activeField ? (direction==="asc"?"↑":"↓") : "";
    return(
      <th
        onClick={()=> onRequestSort(field)}
        aria-sort={activeField? direction : "none"}
        title={`Sort by ${label}`}
        style={{cursor:"pointer"}}
      >
        {label} {arrow}
      </th>
    );
  };

  return (
    //TODO: this needs to look nicer, probably with MUI components
    <table>
      <thead>
        <tr>
          <SortingButton field="robotId" label="Robot ID" />
          <SortingButton field="category" label="Category" />
          <SortingButton field="location" label="Location" />
          <SortingButton field="timeMS" label="Time" />
          <th>Details</th>
        </tr>
      </thead>
      <tbody>
        {events.map((event) => (
          <tr key={event.eventId}>
            <td>{event.robotId}</td>
            <td>{event.category}</td>
            <td>{event.location}</td>
            <td>{event.time}</td>
            <td>
              <button
                onClick={() => handleViewDetails(event)}
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