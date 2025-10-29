import React from 'react';
import "../styles/RobotCard.css";

export const RobotCard = ({ imgSrc, imgAlt, title, description, admin, buttonText, link, onDelete }) => {
  console.log('RobotCard component is rendering');

  return (
    <div className="robot-card-container">
      {imgSrc && <img src={imgSrc} alt={imgAlt} className='robot-card-img'/>}
      {title && <h1 className="robot-card-title">{title}</h1>}
      {description && <p className="robot-card-description">{description}</p>}
      {admin && <p className="robot-card-admin">Owner: {admin}</p>}
      {link && buttonText && (
        <>
          <a href={link} className="robot-card-btn">{buttonText}</a>
          {onDelete && (
            <button className="robot-card-btn delete-btn" onClick={onDelete}>
              Delete Robot
            </button>
          )}
        </>
      )}
    </div>
  );
};