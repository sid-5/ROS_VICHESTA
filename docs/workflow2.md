# Project Workflow Detailing

### **Workflow**

> <img src="../images/workflow.png" style="vertical-align:middle; padding:25px 25px 25px 25px" width="700">

### **Explanation**

> - Now as the bot has counted the number of balls in zone 1 and zone 2 so the first half of the problem is done.
> 
> > **Aruco Markers**
> <img src="../images/image6.jpeg" style="vertical-align:middle; > padding:25px 25px 25px 25px" width="700">
> 
> - The bot will navigate in the room and detection and reading of  that `aruco` markers will take place. 
> After detection of the ID's of the `aruco` markers now the bot will locate all the balls in zone 3 and add  them to the total number of balls found in zone 1 and zone 2.
> - Now as per the problem statement let the number of balls be `X`. Now we will compute the remainder of `X % 5`. Let's take that as `Y`.
>
> - Now after the computation, our bot will navigate to the room of  the map where the `aruco` marker are placed.
> 
> 
> - As mentioned we have to find the `aruco` marker with `ID = Y`.
> 
> - After detection of that `aruco` marker, we need to detect the  color of the box which is kept on the top of that particular  `aruco` marker. Let the color be `C`.
>
> - Now after the detection of the color the the robot needs to  navigate to the rooms which has `colored doors` to the finish line. 
>
> > **Colored Doors**
> <img src="../images/image3.jpeg" style="vertical-align:middlepadding:25px 25px 25px 25px" width="700">
>
> - Next task of the bot is to detect the same color as that of box  that of the door. Whichever door which has the same color, the bot needs to pass through that particular door and cross the finish line in order to accomplish the task. 
> 
> - After doing so the task will get over.