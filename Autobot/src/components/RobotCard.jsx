import React from 'react'
import "../styles/RobotCard.css";

export const RobotCard = ({ imgSrc, imgAlt, title, description, buttonText, link }) => {
    console.log('RobotCard component is rendering');

return (
    <div className="robot-card-container">
            {imgSrc && <img src={imgSrc} alt={imgAlt} className='robot-card-img'/>}
            {title && <h1 className="robot-card-title">{title}</h1>}
            {description && <p className="robot-card-description">{description}</p>}
            {link && buttonText && <a href={link} className="robot-card-btn">{buttonText}</a>}
    </div>
)
}
