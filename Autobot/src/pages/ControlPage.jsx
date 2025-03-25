import {Button, Card, CardMedia, Grid ,Typography} from "@mui/material";
import * as React from "react";
import { ArrowUpward,ArrowDownward,ArrowBack,ArrowForward } from "@mui/icons-material";
const adjustRobotDirection = (direction) => { 
    //This function will be used to send the direction to the robot
}

const ControlPage =()   => {
    return (
   
        

        <div style ={{ padding: "20px",display: "flex", flexDirection: "column", justifyContent: "center", alignItems: "center"}}>
            <div style={{position:"fixed", top: "0", left: "50%",transform: "translateX(-50%)"}}>
                <Typography variant="h3" style={{textAlign: "center"}}>Robot Control</Typography>
            </div>
           <Card style={{ width: "100%", margin: "20px", border: "1px solid black",  }}>
                <CardMedia style={{height: "150",width: "250", objectFit: "cover" }}
                    component="img"
                   
                    image="https://via.placeholder.com/150"
                    alt="video feed"
                    
                    //instead of an image here this can be adjusted to be an mp4. 
                    //the type of video feed would have to be "progressive download"
                    //for live streaming we would need a tool like hls to convert
                    //ex usage :image="https://via.placeholder.com/150" would be  src="https://your-video-source-url.mp4"
                    // and component would be "video" not "img"
                    
                />
            </Card>
            <Grid container direction="column" justifyContent="space-between" alignItems="center" >
                <Grid item >
                    <Button onClick={() => adjustRobotDirection("forward")} variant="contained" style={{display: "flex", justifyContent:"center", alignItems: "center"}} >
                        <ArrowUpward style={{ fontSize: 50 }} />
                    </Button>
                </Grid>

                <Grid container spacing={2} justifyContent="space-between" alignItems="center">
                    <Grid item xs={6}>
                        <Button onClick={() => adjustRobotDirection("left")} variant="contained"style={{display: "flex"}} >
                            <ArrowBack style={{ fontSize: 50 }} />
                        </Button>
                    </Grid>
                    <Grid item xs={6}>
                        <Button onClick={() => adjustRobotDirection("right")} variant="contained" style={{display: "flex"}}>
                            <ArrowForward style={{ fontSize: 50 }} />
                        </Button>
                    </Grid>
                </Grid>

                <Grid item>
                    <Button onClick={() => adjustRobotDirection("back")} variant="contained" style={{display: "flex", justifyContent:"center", alignItems: "center"}}>
                        <ArrowDownward style={{ fontSize: 50 }} />
                    </Button>
                </Grid>
             
            </Grid>
        </div>
        
        
    );
};
export default ControlPage;
