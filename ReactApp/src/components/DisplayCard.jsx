import React from "react";
import { Box, Card, CardContent, CardMedia, Typography } from "@mui/material";

/* 
 Card component to display the detected object and its metadata.
    The image, metadata, and labels are passed as props.
    The metadata is stored as key-value pairs in an object.
*/

const DisplayCard = ({title, image, metadata, labels }) => {
  return (
    <Card>
      {image && (
        <CardMedia
          component="img"
          image={image || "https://via.placeholder.com/500"} // Default placeholder
          alt="Detected Object"
        />
      )}
      <CardContent>

        <Typography variant="h6" component="div">
          {title}
        </Typography>

        <Box sx={{ mt: 1 }}>
          {metadata ? (
            Object.entries(metadata).map(([key, value]) => (
              <Typography key={key} variant="body2" color="text.secondary">
                <strong>{labels[key]}:</strong> {value}
              </Typography>
            ))
          ) : (
            <Typography variant="body2" color="text.secondary">
              No metadata available.
            </Typography>
          )}
        </Box>
      </CardContent>
    </Card>
  );
};

export default DisplayCard;