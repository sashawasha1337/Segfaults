import React from 'react';
import { useNavigate } from "react-router-dom";

const EventTable = ({events}) => {
    const navigate=useNavigate();
    const handleViewDetails = (eventId) => {
        //navigate(`/EventDetails/${eventId}`);
        navigate(`/Trash_view_page`);
    }
    return (
        <table>
            <thead>
                <tr>
                    <th>Event Type</th>
                    <th>Event Location</th>
                    <th>Event Time</th>
                    <th>Event Details</th>
                </tr>
            </thead>
            <tbody>
                {events.map((event) => (
                    <tr key={event.eventId}>
                        <td>{event.type}</td>
                        <td>{event.location}</td>
                        <td>{event.time}</td>
                        <td><button onClick={() => handleViewDetails(event.eventId)}>View Details</button></td>
                    </tr>
                ))}
            </tbody>
        </table>

    );
    

}



export default EventTable;