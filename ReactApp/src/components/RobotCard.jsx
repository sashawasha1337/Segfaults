import React from 'react';
import "../styles/RobotCard.css";

export const RobotCard = ({
  imgSrc,
  imgAlt,
  title,
  description,
  admin,
  buttonText,
  link,
  onDelete,
  isAdmin,
  onManageUsers
}) => {
  console.log('RobotCard component is rendering');

  return (
    <div className="robot-card-container">
      {imgSrc && <img src={imgSrc} alt={imgAlt} className='robot-card-img'/>}
      {title && <h1 className="robot-card-title">{title}</h1>}
      {description && <p className="robot-card-description">{description}</p>}
      {admin && <p className="robot-card-admin">Owner: {admin}</p>}

      {/* Main FPV/Control button */}
      {link && buttonText && (
        <a href={link} className="robot-card-btn">{buttonText}</a>
      )}

      {/* --- Admin-only buttons --- */}
      {isAdmin && (
        <div style={{ marginTop: "10px", display: "flex", flexDirection: "column", gap: "10px" }}>
          <button
            className="robot-card-btn"
            style={{ backgroundColor: "green" }}
            onClick={onManageUsers}
          >
            Manage Users
          </button>

          <button
            className="robot-card-btn delete-btn"
            onClick={onDelete}
          >
            Delete Robot
          </button>
        </div>
      )}

      {/* Non-admin delete (remove themselves) - ONLY if admin buttons not shown */}
      {!isAdmin && onDelete && (
        <button className="robot-card-btn delete-btn" onClick={onDelete}>
          Remove From My List
        </button>
      )}
    </div>
  );
};
