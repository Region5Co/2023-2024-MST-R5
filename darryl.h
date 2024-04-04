//A-D-H-F-B-G-E-C-A
  {

inlcude "Encoder.h"
Encoder encoder();

switch(stage){ //make stage variable
    case 0: 
    {
      Serial.println("headed to D");
      delay(2000);
      //turn 45 degrees CW
      moveUntilWithServo(Forward, 8.49, true)2
      delay(1000);
      stage = 1;
      break;
    }
    case 1:
    {
      Serial.println("headed to H");
      delay(2000);
      //turn 45 degrees CW
      moveUntilWithServo(FORWARD, 8.94,false);
      //turn 135f

      stage = 2;
      break;
    }
    case 2:
    {
      Serial.println("headed to F");
      delay(2000);
      //turn 71.5651 degrees CW
      moveUntilWithServo(FORWARD, 6.33, false);
      //turn 108.44
      stage = 3;

      break;
    }
    case 3: 
    {
      Serial.println("headed to B");
      delay(2000);
      //turn 63.4349
      moveUntilWithServo(FORWARD, 8.94, false); /* will need to test this value. Likely should use the encoders to verify*/
      ///106.56      
stage = 4;
      break;
    }
    case 4: 
    {
      Serial.println("headed to G");
      delay(2000);
      //turn 71.5651
      moveUntilWithServo(FORWARD, 8.453, false);
      delay(2000);
      //turn 108.44
      stage = 5;
      break;

    case 5: 
    {
      Serial.println("headed to E");
      delay(2000);
      //turn 63.4349
      moveUntilWithServo(FORWARD, 6.33, false);
      delay(2000);
      //turn 116.56
      stage = 6;
      break;
    case 6: 
    {
      Serial.println("headed to C");
      delay(2000);
      //turn 71.5651
      moveUntilWithServo(FORWARD, 6.33, false);
      delay(2000);
      //turn
      stage = 7;
      break;
    case 7: 
    {
      Serial.println("headed to A");
      delay(2000);
      //turn 18.4349
      moveUntilWithServo(FORWARD, 6.33, false);
      delay(2000);
      //turn 161.57
      stage = 0;
      break;
/*void moveUntilLt(moveDirection dir, int targetDist) {
  while (vl53.distance() == -1){
    Serial.println(vl53.distance());
    Serial.print("stuck in -1");
    robot.drive(dir, 100);
    delay(50);
  }

  while (vl53.distance() > targetDist) {
    Serial.println(vl53.distance());
    Serial.print("moveUntilLt");
    robot.drive(dir, 100);
    delay(50);
  }
  robot.stop();
}*/
void moveForward(moveDirection dir, int targetDist,bool ISLEFT){//julian verify
  int current_pos = encoder.getEncDist();
  int goal = targetDist - current_pos;
  i=0;
  while(encoder.getEncDist()<goal){
    robot.drive(FORWARD, 60, 1000);
    checkObstruction(forward, 200,ISLEFT);
}
}
int revisedDist(int dist)   {  
    if (dist==-1){
        dist=1000;
    return dist
}
}
void evade(bool ISLEFT){
    if (ISLEFT){
      //move certain amount left
      //mve
}
    else{
      //move certain amount right
    
}
}

  
  }
void checkObstruction(moveDirection dir, int watchArea, ISLEFT) { 

  if (revisedDist(vl53.distance()) < watchArea ) {
    evade(ISLEFT)
    }
    Serial.println(revisedDist(vl53.distance()));
    Serial.print("moveUntilLt");
    robot.drive(dir, 100);
    delay(50);
  }

void moveUntilWithServo(moveDirection dir, int targetDist, bool ISLEFT) { // 90 degree rotation on this lad lmao
  if (dir == FORWARD) {
    myservo.write(20);
  } else {
    myservo.write(150);
  }
  delay(2000);
  moveForward(dir,targetDist,ISLEFT);
  if (isLt) {
    moveUntilLt(dir, targetDist);
  } else {
  moveUntilGt(dir, targetDist);
  }
}

