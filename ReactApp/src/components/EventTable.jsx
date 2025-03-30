import React from 'react';
import { useNavigate } from "react-router-dom";

const EventTable = ({events}) => {
  const navigate=useNavigate();
  const handleViewDetails = (eventId) => {
    //commented out navigation to specific events 
    //navigate(`/EventDetails/${eventId}`);
    navigate('/TrashViewPage');
  }

  return (
    <table>
      <thead>
        <tr>
          <th>Event ID</th>
          <th>Robot ID</th>
          <th>Object Category</th>
          <th>Event Location</th>
          <th>Event Time</th>
          <th>Event Details</th>
        </tr>
      </thead>
      <tbody>
        {events.map((event) => (
          <tr key={event.eventId}>
            <td>{event.eventId}</td>
            <td>{event.robotId}</td>
            <td>{event.category}</td>
            <td>{event.location}</td>
            <td>{event.time}</td>
            <td>
              <button
                onClick={() => handleViewDetails(event.eventId)}
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