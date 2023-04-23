import processing.serial.*;
import java.util.LinkedList;
import java.util.Queue;



Serial sensorPort;
static String raw; //raw values from serial
static String[] vals;

long inf = 9000000000000000000L;
int arr_len = 2000;
float min_thresh = 0.1; 
float is_peak = 0.6; 
float d_peak = 0.005;
float filt_val = 0.2;

int calib_count = 0;
long[] chan1_calib;
long[] chan2_calib;
long[] chan3_calib;
long chan1_min = inf, chan1_max;
long chan2_min = inf, chan2_max;
long chan3_min = inf, chan3_max;

float[] chan1, chan1_feat, d_chan1;
float[] chan2, chan2_feat, d_chan2;
float[] chan3, chan3_feat, d_chan3;
int[] chan1_feat_cnt, chan2_feat_cnt, chan3_feat_cnt;
int[] state, feat_idx;

// Directions: true = clockwise, false = counter-clockwise
boolean lost = true, direction = true;
int last_state = 0;

int calib_sec = 25; //calibration time, can be adjusted 
int calib_start = millis();

//angular position
double curr_pos = 0, last_pos = 0, curr_pos_1 = 0, curr_pos_2 = 0, curr_pos_3 = 0;
double[] pos_1_hist, pos_2_hist, pos_3_hist;
//number of rotatations
float total_rot = 0, last_rot = 0, rpm = 0, last_rpm = 0;
int last_t = 0;
int rot_count = 0;

boolean run_system = false;
boolean calibrating = false;
boolean draw_waveforms = false; //set to true to see waveforms during calibration 


int window_center;



//----- Fishing game (start)-----// 

PImage img;
PImage bar1; 
int widthBar1= displayWidth; 
PImage rod; 
PImage fish; 
int canvasHeight= 900; 

float step_f = 1; 
float r_f = 0; //speed meter bar length rate change (depends on RPM)
float i_f=0; 
boolean finishedBar= false; 
int numberRots_f= 0; //number of rotatations
int fishyCaught=0;  //number of fish caught
boolean fishingGame= false; //change to Trrue to initiate game





//----- Fishing game speed Meter (start)-----// 

void speedMeter(double increment) {

 // Display the RPM rectangle
 finishedBar=false;
  fill(42, 69, 100);
  stroke(0);
  rect(1042, displayHeight*0.8, 30, -1*(20+r_f), 8,8,8,8);

  // Increase the rectangle size
  if (increment > 1) {
    r_f+=increment*0.1; // r_f value determined by RPM (ie how fast our increments change)
    //println("current rpm:", increment);
  }

  // Start rectangle over 
  if (r_f > 580) { //580 represents max length of rectangle
    r_f = 0;
    finishedBar=true;
  }

}

//----- Fishing game speed Meter (end) -----// 

//--- Refresh Fishing Rod elements (start)----// 
void refreshBg() {

  //img = loadImage("assets/game-bg.png"); 
  //size(displayWidth, 900); 
  //img.resize(displayWidth, 900); 
  background(img);
}

//--- Refresh Fishing Rod elements (end)----// 

//--- Rotate rod handle (start) ---// 

void rotateRod (float angle, float x, float y){
  
   imageMode(CENTER);
   pushMatrix();
   translate(x,y);
   rotate(-1*radians(angle));
   image(rod,0,0);
   popMatrix(); 
   imageMode(CORNER);

}
//--- Rotate rod handle (end) ---// 

// ---- Translate Fish (start)--- // 

void fishMove() {

  image(fish,0,-250*numberRots_f);// each fish needs to reach position 400; 
  
  if (finishedBar) {
  numberRots_f++;
  }

  if (numberRots_f>=2){
    numberRots_f=0;
    fishyCaught++;
    
  }
  
}


// ---- Translate Fish (end)--- // 

//--- Update Fishgame text (start) --- //

void updateFishyText() {

textSize(40);
fill(42, 0, 100);
text(fishyCaught, displayHeight*0.34, displayHeight*0.07); 


}

//--- Update Fishgame text (end) --- //



//----- Fishing game (end)-----// 




void setup()
{
  
  //load fish game images
  img = loadImage("assets/game-bg.png");
  size(displayWidth, 900);
  img.resize(displayWidth, 900);
  
  fish= loadImage("assets/bar-2.png");
  fish.resize(displayWidth, 900);
  
  imageMode(CENTER);
  rod= loadImage("assets/rod.png");
  rod.resize(150, 150);
  
  // Automatically find the serial device
  // TODO: Works on Mac only right now. Just need to change the string.
  String[] devices = Serial.list();
  String portName = "";
  for (int i = 0; i < devices.length; i++) {
    if (devices[i].indexOf("usbserial") > 0 | devices[i].indexOf("usbmodem") > 0) {
      portName = devices[i];
      break;
    }
    
  }
  println(portName);
  
  // Try connecting
  try {
    sensorPort = new Serial(this, portName, 115200);
  } catch (RuntimeException e) {
    println("No Devices Found");
    exit();
  }

  chan1_calib = new long[arr_len];
  chan2_calib = new long[arr_len];
  chan3_calib = new long[arr_len];
  chan1 = new float[arr_len];
  chan2 = new float[arr_len];
  chan3 = new float[arr_len];
  d_chan1 = new float[arr_len];
  d_chan2 = new float[arr_len];
  d_chan3 = new float[arr_len];
  
  pos_1_hist = new double[arr_len];
  pos_2_hist = new double[arr_len];
  pos_3_hist = new double[arr_len];
  
  // Feature Array Structure
  // 0: Minimum of Off-State Transition (Local Min)
  // 1: Maximum of First Off-State Transition (Local Max)
  // 2: Maximum of Second Off-State Transition (Local Max)
  // One of the last two should technically be 1
  chan1_feat = new float[6];
  chan2_feat = new float[6];
  chan3_feat = new float[6];
  chan1_feat_cnt = new int[6];
  chan2_feat_cnt = new int[6];
  chan3_feat_cnt = new int[6];
  
  state = new int[arr_len];
  feat_idx = new int[arr_len];
  
  //size(1080, 720);
  size (displayWidth, 900); 
  //fullScreen();
  colorMode(HSB, 360, 100, 100);
  
  window_center = 260+(width-260)/2;
}

/*
* The drawing loop has the following flow:
* 1. Shift old data
* 2. Parse new data and add to arrays
* 3. If in calibration, update mins & maxes
* 4. Calculations
* 5. Draw new frames
*/
void draw()
{  
    
  

  // Check if calibration is complete
  if (millis()-calib_start > calib_sec*1000 & calibrating) {
    calibrating = false;
    calibrationFinished();
  }
  
  // Shift Data
  for (int i = 1; i < arr_len; i++) { 
    chan1[i-1] = chan1[i]; 
    chan2[i-1] = chan2[i];
    chan3[i-1] = chan3[i];
    d_chan1[i-1] = d_chan1[i];
    d_chan2[i-1] = d_chan2[i];
    d_chan3[i-1] = d_chan3[i];
    state[i-1] = state[i];
    feat_idx[i-1] = feat_idx[i];
    pos_1_hist[i-1] = pos_1_hist[i];
    pos_2_hist[i-1] = pos_2_hist[i];
    pos_3_hist[i-1] = pos_3_hist[i];
  }
  
  // Ingest Data
  // Note: If data is coming in faster than the framerate, the buffer will fill and this will need to be switched to a while loop
  
  //println(sensorPort.available());
  if (sensorPort.available() > 0) {
    try {
      raw = sensorPort.readStringUntil('\n'); 
      
      vals = split(raw, ',');
    } catch (Exception e) {
      System.err.println(e);
      e.printStackTrace();
    }
    
    if (calibrating && raw != null) {
      //the raw data from the library doesn't need the '.00' hence why we convert to double
      
      chan1_calib[calib_count] = (long)Double.parseDouble(vals[0].trim()); //Original was 0, 
      chan2_calib[calib_count] = (long)Double.parseDouble(vals[1].trim());//Original was 1, Long.valueOf(vals[3].trim())
      chan3_calib[calib_count] = (long)Double.parseDouble(vals[2].trim()); //Original was 2

      
      chan1_min = long_min(chan1_min, chan1_calib[calib_count]);
      chan1_max = long_max(chan1_max, chan1_calib[calib_count]);
      chan2_min = long_min(chan2_min, chan2_calib[calib_count]);
      chan2_max = long_max(chan2_max, chan2_calib[calib_count]);
      chan3_min = long_min(chan3_min, chan3_calib[calib_count]);
      chan3_max = long_max(chan3_max, chan3_calib[calib_count]);
      calib_count++;
    } else if (run_system & !calibrating & vals!=null) {      
      chan1[arr_len-1] = map((long)Double.parseDouble(vals[0].trim()), chan1_min, chan1_max, 0, 1);
      chan2[arr_len-1] = map((long)Double.parseDouble(vals[1].trim()), chan2_min, chan2_max, 0, 1);
      chan3[arr_len-1] = map((long)Double.parseDouble(vals[2].trim()), chan3_min, chan3_max, 0, 1);
      
      chan1[arr_len-1] = chan1[arr_len-1]*filt_val + chan1[arr_len-2]*(1-filt_val);
      chan2[arr_len-1] = chan2[arr_len-1]*filt_val + chan2[arr_len-2]*(1-filt_val);
      chan3[arr_len-1] = chan3[arr_len-1]*filt_val + chan3[arr_len-2]*(1-filt_val);
      
      d_chan1[arr_len-1] = chan1[arr_len-1] - chan1[arr_len-2];
      d_chan2[arr_len-1] = chan2[arr_len-1] - chan2[arr_len-2];
      d_chan3[arr_len-1] = chan3[arr_len-1] - chan3[arr_len-2];
    }
  }

  // Calculate State
  state[arr_len-1] = calculateState(chan1[arr_len-1], chan2[arr_len-1], chan3[arr_len-1]);
  // If not valid state, pull forward last state
  if (state[arr_len-1] != 1 & state[arr_len-1] != 2 & state[arr_len-1] != 4) {
    state[arr_len-1] = state[arr_len-2];
  }
  
  // Featuring Updating
  // Look for "Good Sandwiches" where one state is adjacent to two different states
  // If state change, decrement through array
  if (state[arr_len-1] != last_state) {
    float[] max_vals = new float[3];
    int i = arr_len-2;
    // While in the previous state, look for max
    while (i > 0 & state[i] == last_state) {
      max_vals[0] = max(chan1[i], max_vals[0]);
      max_vals[1] = max(chan2[i], max_vals[1]);
      max_vals[2] = max(chan3[i], max_vals[2]);
      i--;
    }
    // If newest state is different from 2nd to last state, good sandwich
    if (state[arr_len-1] != state[i] & state[i] != 0) {
      if (last_state == 1) {
        //println("Updating Peak 1: CH2: ",chan2_feat[1],",",max_vals[1]," CH3: ",chan3_feat[1],",",max_vals[2]);
        chan2_feat[1] = max_vals[1];
        chan3_feat[1] = max_vals[2];
      } else if (last_state == 2) {
        //println("Updating Peak 2: CH1: ",chan1_feat[3],",",max_vals[0]," CH3: ",chan3_feat[3],",",max_vals[2]);
        chan1_feat[3] = max_vals[0];
        chan3_feat[3] = max_vals[2];
      } else if (last_state == 4) {
        //println("Updating Peak 4: CH1: ",chan1_feat[5],",",max_vals[0]," CH2: ",chan2_feat[5],",",max_vals[1]);
        chan1_feat[5] = max_vals[0];
        chan2_feat[5] = max_vals[1];
      }
    }
  }
  
  // Feature Tracking
  // First check for transitions, these are our most informative features as they give us absolute position and direction
  // feat_idx variable tracks current minimum being compared to
  if (state[arr_len-1] != last_state ) {
    // 2-4 Transition
    if ((last_state == 2 & state[arr_len-1] == 4)|(last_state == 4 & state[arr_len-1] == 2)) {
      direction = (state[arr_len-1] == 4);
      feat_idx[arr_len-1] = 4;
    // 4-1 Transition
    } else if ((last_state == 4 & state[arr_len-1] == 1)|(last_state == 1 & state[arr_len-1] == 4)) {
      direction = (state[arr_len-1] == 1);
      feat_idx[arr_len-1] = 0;
      if (last_state == 4) { rot_count += 1; } else { rot_count -= 1; }
    // 1-2 Transition
    } else if ((last_state == 1 & state[arr_len-1] == 2)|(last_state == 2 & state[arr_len-1] == 1)) {
      direction = (state[arr_len-1] == 2);
      feat_idx[arr_len-1] = 2;
    }
  // If not at a transition, need known peak to determine position
  // Requires that we fix direction near the peak
  // Near the peak is defined as within 5% of the peak feature for that channel
  } else {
    if (state[arr_len-1] == 1) {
      if ((chan2[arr_len-1] > is_peak*chan2_feat[1] & abs(d_chan2[arr_len-1]) < d_peak) & (chan3[arr_len-1] > is_peak*chan3_feat[1] & abs(d_chan3[arr_len-1]) < d_peak)) {
        if (direction) {feat_idx[arr_len-1] = 2;} else {feat_idx[arr_len-1] = 0;}
      }
    } else if (state[arr_len-1] == 2) {
      if ((chan1[arr_len-1] > is_peak*chan1_feat[3] & abs(d_chan1[arr_len-1]) < d_peak) & (chan3[arr_len-1] > is_peak*chan3_feat[3] & abs(d_chan3[arr_len-1]) < d_peak)) {
        if (direction) {feat_idx[arr_len-1] = 4;} else {feat_idx[arr_len-1] = 2;}
      }
    } else if (state[arr_len-1] == 4) {
      if ((chan1[arr_len-1] > is_peak*chan1_feat[5] & abs(d_chan1[arr_len-1]) < d_peak) & (chan2[arr_len-1] > is_peak*chan2_feat[5] & abs(d_chan2[arr_len-1]) < d_peak)) {
        if (direction) {feat_idx[arr_len-1] = 0;} else {feat_idx[arr_len-1] = 4;}
      }
    }
  }
  last_state = state[arr_len-1];
  
  // Calculate Angle
  if (state[arr_len-1] == 1) {
    curr_pos_2 = cap2pos(map(chan2[arr_len-1], chan2_feat[feat_idx[arr_len-1]], chan2_feat[1], 0, 1));
    curr_pos_3 = cap2pos(map(chan3[arr_len-1], chan3_feat[feat_idx[arr_len-1]], chan3_feat[1], 0, 1));
    if (0 < curr_pos_2 && curr_pos_2 < 60 && 0 < curr_pos_3 && curr_pos_3 < 60) {
      if (feat_idx[arr_len-1] == 0) { // curr_pos = curr_pos_2 ;
        curr_pos = curr_pos_2;
        if (curr_pos < 5) {
          curr_pos = 5;
        } else if (curr_pos > 55) {
          curr_pos = 55;
        }
        curr_pos = map((float)curr_pos, 5, 55, 0, 60);
        println("state 1, the angle is:", curr_pos);
      } else {
        curr_pos = curr_pos_2;
        if (curr_pos < 5) {
          curr_pos = 5;
        } else if (curr_pos > 55) {
          curr_pos = 55;
        }
        curr_pos = 120 - map((float)curr_pos, 5, 55, 0, 60);
        println("state 2, the angle is:", curr_pos);
      }
    }
  } else if (state[arr_len-1] == 2) {
    curr_pos_1 = cap2pos(map(chan1[arr_len-1], chan1_feat[feat_idx[arr_len-1]], chan1_feat[3], 0, 1));
    curr_pos_3 = cap2pos(map(chan3[arr_len-1], chan3_feat[feat_idx[arr_len-1]], chan3_feat[3], 0, 1));
    if (0 < curr_pos_1 && curr_pos_1 < 60 && 0 < curr_pos_3 && curr_pos_3 < 60) {
      if (feat_idx[arr_len-1] == 2) { 
        curr_pos = curr_pos_1;
        if (curr_pos < 10) {
          curr_pos = 10;
        } else if (curr_pos > 50) {
          curr_pos = 50;
        }
        curr_pos = 120 + map((float)curr_pos, 10, 50, 0, 60);
        println("state 3, the angle is:", curr_pos);
      } else { 
        curr_pos = curr_pos_1;
        if (curr_pos < 10) {
          curr_pos = 10;
        } else if (curr_pos > 50) {
          curr_pos = 50;
        }
        curr_pos = 240 - map((float)curr_pos, 10, 50, 0, 60);
        println("state 4, the angle is:", curr_pos);
      }
    }
    ////curr_pos = 120 + curr_pos_1;
    //if (0 < curr_pos_1 && curr_pos_1 < 60 && 0 < curr_pos_3 && curr_pos_3 < 60) {
    //if (feat_idx[arr_len-1] == 2) { curr_pos = 120 + curr_pos_1 ;
    //println("state 3, the angle is:", (curr_pos_1 + curr_pos_3)/2);
  
    ////curr_pos = 120 + (curr_pos_1 + curr_pos_3)/2;
    
    //}
    //    else { curr_pos = 240 - curr_pos_1 ; 
    //    println("state 4, the angle is:", (curr_pos_1 + curr_pos_3)/2);
    ////curr_pos = 240 - (curr_pos_1 + curr_pos_3)/2;  
    
    //}
    //}
  } else if (state[arr_len-1] == 4) {
    curr_pos_1 = cap2pos(map(chan1[arr_len-1], chan1_feat[feat_idx[arr_len-1]], chan1_feat[5], 0, 1));
    curr_pos_2 = cap2pos(map(chan2[arr_len-1], chan2_feat[feat_idx[arr_len-1]], chan2_feat[5], 0, 1));
    if (0 < curr_pos_1 && curr_pos_1 < 60 && 0 < curr_pos_2 && curr_pos_2 < 60) {
      if (feat_idx[arr_len-1] == 4) { // (curr_pos_1 + curr_pos_2)/2
        curr_pos = curr_pos_2;
        if (curr_pos < 5) {
          curr_pos = 5;
        } else if (curr_pos > 55) {
          curr_pos = 55;
        }
        curr_pos = 240 + map((float)curr_pos, 5, 55, 0, 60);
        println("state 5, the angle is:", curr_pos);
      } else { 
        curr_pos = curr_pos_2;
        if (curr_pos < 5) {
          curr_pos = 5;
        } else if (curr_pos > 55) {
          curr_pos = 55;
        }
        curr_pos = 360 - map((float)curr_pos, 5, 55, 0, 60);
        println("state 6, the angle is:", curr_pos);
      }
    }
    
    //if (0 < curr_pos_1 && curr_pos_1 < 60 && 0 < curr_pos_2 && curr_pos_2 < 60) {
    //  if (feat_idx[arr_len-1] == 4) { curr_pos = 240 + curr_pos_1 ;
    //      println("state 5, the angle is:", (curr_pos_1 + curr_pos_2)/2);
    //    //curr_pos = 240 + (curr_pos_1 + curr_pos_2)/2;
      
    //  }
    //      else { curr_pos = 360 - curr_pos_1;
    //      println("state 6, the angle is:", (curr_pos_1 + curr_pos_2)/2);
        
    //  //curr_pos = 360 - (curr_pos_1 + curr_pos_2)/2;
      
    //  }
    //}
  }
  // IIR Filter on Postion
  //curr_pos = 0.1*curr_pos + 0.9*last_pos;
  pos_1_hist[arr_len-1] = curr_pos_1;
  pos_2_hist[arr_len-1] = curr_pos_2;
  pos_3_hist[arr_len-1] = curr_pos_3;
  //println(curr_pos);
  //println(curr_pos_1, curr_pos_2, curr_pos_3);
  
  // Update Position
  if (run_system) {
    total_rot = (float)(rot_count + curr_pos/360);
  }
  
  // Calculate RPM
  rpm = -(total_rot - last_rot)/(millis() - last_t)*60000;
  rpm = 0.1*rpm + 0.9*last_rpm;
  last_rot = total_rot;
  last_t = millis();
  last_rpm = rpm;
  
  // Update Last Position
  last_pos = curr_pos;
  
  
  // Draw Background
  background(0, 0, 100);

  // Draw Header
  noStroke();
  rectMode(CORNER);
  fill(52, 100, 96);
  rect(0, 0, width, 60);
  fill(0);
  textSize(30);
  textAlign(LEFT);
  text("MechSense", 20, 40);
  
  // Draw Frame Rate
  //text(frameRate, width-110, 40);
  
  // Draw Buttons
  stroke(0);
  textSize(20);
  textAlign(CENTER);
  line(260, 60, 260, height);
  fill(147, 99, 99);
  rect(30, 90, 200, 50, 10);
  //fill(175, 99, 95);
  //rect(30, 170, 200, 50, 10);
  fill(324, 99, 66);
  rect(30, 250, 200, 50, 10);
  fill(0);
  text("Start Calibration", 130, 122);
  //text("Redo Calibration", 130, 202);
  fill(0, 0, 100);
  text("Waveforms", 130, 282);

 fill(254, 200, 100);
 
 //rect(30, 350, 200, 50, 10);

 // fill(0, 0, 100);
 // text("Fishing Game", 130, 385);
  
  
  // UI State Machine
  // If not running
  if(!run_system) {
    fill(0);
    textSize(60);
    text("Press Start Calibration", window_center, 400);
  // If calibrating and draw_waveforms is not selected
  } else if (calibrating & !draw_waveforms) {
    fill(0);
    textSize(60);
    text("Calibration Started", window_center, 425);
    //text("Rotate the mechanism for "+calib_sec+" seconds", window_center, 500);
    text("Rotate the mechanism at least 3 times", window_center, 500);
    noStroke();
    fill(175, 99, 95);
    rectMode(CORNER);
    rect(-190+(width-260)/2, 560, (millis()-calib_start)/1000.0/calib_sec*900, 120);
    noFill();
    stroke(0);
    rectMode(CENTER);
    rect(window_center, 620, 900, 120);
  // If draw_waveforms is selected during calibration
  } else if(calibrating & draw_waveforms) {
    fill(0);
    textSize(30);
    text("Calibration Started", window_center, 125);
    text("Rotate the mechanism for "+calib_sec+" seconds", window_center, 175);
    noStroke();
    fill(175, 99, 95);
    rectMode(CORNER);
    rect(-40+(width-260)/2, 210, (millis()-calib_start)/1000.0/calib_sec*600, 60);
    noFill();
    stroke(0);
    rectMode(CENTER);
    rect(window_center, 240, 600, 60);
    for(int i = 0; i < calib_count; i++) {
      stroke(1);
      strokeWeight(2);

 
      point(map(i, 0, calib_count, 260, width), map(chan1_calib[i], chan1_min, chan1_max, 310, 310+(height-390)/3));
      point(map(i, 0, calib_count, 260, width), map(chan2_calib[i], chan2_min, chan2_max, 330+(height-390)/3, 330+(height-390)*2/3));
      point(map(i, 0, calib_count, 260, width), map(chan3_calib[i], chan3_min, chan3_max, 350+(height-390)*2/3, height-40));
      strokeWeight(0);
    }
  // If system is running but not drawing waveforms
  } else if (run_system && !draw_waveforms && !fishingGame) {
    noFill();
    strokeWeight(3);
    stroke(0);
    circle(window_center, 425, 450);
    line(window_center, 425, window_center + 300*cos((float)-curr_pos*2*PI/360), 425 + 300*sin((float)-curr_pos*2*PI/360));
    //line(window_center, 300, window_center + 150*sin(total_rot%(2*PI)), 300 - 150*cos(total_rot%(2*PI)));
    circle(window_center + 300*cos((float)-curr_pos*2*PI/360), 425 + 300*sin((float)-curr_pos*2*PI/360), 20);
    strokeWeight(0);
    //circle(window_center + 150*sin(total_rot%(2*PI)), 300 - 150*cos(total_rot%(2*PI)), 10);
    fill(0);
    textSize(60);
    //text("Angle: " + (int)curr_pos + "°", window_center, 820);
    if (rot_count >= 0) {
      text("Distance: " + (int)((rot_count+curr_pos/360)*34.6) + " cm", window_center, 820);
    } else {
      text("Distance: " + (int)((rot_count-curr_pos/360)*34.6) + " cm", window_center, 820);
    }
    //text("Speed: " + nfs(rpm, 3, 1) + " rpm", window_center, 895);
    if (rpm > 0) {text("Direction of Rotation: Clockwise", window_center, 970);}
    else {text("Direction of Rotation: Counter-Clockwise", window_center, 970);}
  // If system is running and draw waveforms is selected
  } else if (run_system && draw_waveforms && !fishingGame) {
    noStroke();
    fill(350);
    rect(260, 60+(height-60)/3, width, (height-60)/3);
    for(int i = 0; i < arr_len; i++) {
      strokeWeight(3);
      //stroke(map(state[i], 0, 7, 0, 360), 100, 100);
      if (feat_idx[i] > state[i] | (state[i] == 4 & feat_idx[i] == 0)) {
        stroke(map(state[i], 0, 7, 0, 360), 100, 50);
      } else { 
        stroke(map(state[i], 0, 7, 0, 360), 100, 100);
      }
      //stroke(0);
      
      //stroke(0, map((float)curr_pos_1, 0, 60, 20, 100), 100);
      point(map(i, 0, arr_len, 260, width), map(chan1[i], 0, 1, 60+(height-90)/3, 70));
      //point(map(i, 0, arr_len, 260, width), map(d_chan1[i], -0.025, 0.025, 60+(height-90)/3, 70));
      //point(map(i, 0, arr_len, 260, width), map((float)pos_1_hist[i], 0, 60, 60+(height-90)/3, 70));
      
      //stroke(0, map((float)curr_pos_2, 0, 60, 20, 100), 100);
      point(map(i, 0, arr_len, 260, width), map(chan2[i], 0, 1, 70+(height-90)*2/3, 80+(height-90)/3));
      //point(map(i, 0, arr_len, 260, width), map(d_chan2[i], -0.025, 0.025, 70+(height-90)*2/3, 80+(height-90)/3));
      //point(map(i, 0, arr_len, 260, width), map((float)pos_2_hist[i], 0, 60, 70+(height-90)*2/3, 80+(height-90)/3));
      
      //stroke(0, map((float)curr_pos_3, 0, 60, 20, 100), 100);
      point(map(i, 0, arr_len, 260, width), map(chan3[i], 0, 1, height-10, 90+(height-90)*2/3));
      //point(map(i, 0, arr_len, 260, width), map(d_chan3[i], -0.025, 0.025, height-10, 90+(height-90)*2/3));
      //point(map(i, 0, arr_len, 260, width), map((float)pos_3_hist[i], 0, 60, height-10, 90+(height-90)*2/3));
      
      strokeWeight(1);
    }
  } else if (run_system && fishingGame) {
  
    refreshBg();
    speedMeter(rpm);
    
    rotateRod((float) curr_pos, 830, displayHeight*0.685);
    
    fishMove();
    updateFishyText();

 }
}

// Button registration
void mouseClicked() {
  // Start Calibration Button
  if (mouseX > 30 && mouseX < 230 && mouseY > 90 && mouseY < 140) {
    run_system = true;
    calibrating = true;
    total_rot = 0;
    calib_count = 0;
    calib_start = millis();

  // Waveforms Button
  } else if (mouseX > 30 && mouseX < 230 && mouseY > 250 && mouseY < 300) {
    draw_waveforms = !draw_waveforms;
  } else if (mouseX > 30 && mouseX < 230 && mouseY > 350 && mouseY < 400) {
    fishingGame = true;
    println(fishingGame);
  }
}

void calibrationFinished() {
  
 
  int[] calib_states = new int[calib_count];
  int[] tmp = new int[calib_count];
  
  // Calculate state of calibration data
  for (int i = 0; i < calib_count; i++) {
    calib_states[i] = calculateState(
      map(chan1_calib[i], chan1_min, chan1_max, 0, 1), 
      map(chan2_calib[i], chan2_min, chan2_max, 0, 1),
      map(chan3_calib[i], chan3_min, chan3_max, 0, 1));

  }
 


  // Fill unknown states with closest known state
  for (int i = 0; i < calib_count; i++) { 

    if (calib_states[i] != 1 | calib_states[i] != 2 | calib_states[i] != 4) {
      int j = 0;
      
   
      while (j < calib_count) {
        
        if (j+i > calib_count-1) {
          break;
        } 
        
        if (
        
   
          calib_states[i+j] == 1 |
          calib_states[i+j] == 2 |
          calib_states[i+j] == 4
        ) {
          
          //  println("state-1");
          tmp[i] = calib_states[i+j];
          break;
        } else if (
          calib_states[i-min(i,j)] == 1 |
          calib_states[i-min(i,j)] == 2 |
          calib_states[i-min(i,j)] == 4
        ) {
          
          //  println("state-2");
          tmp[i] = calib_states[i-j];
          break;
        } else {
          
          //println("state-3");
          j++;
        }
      }
    } else {
      tmp[i] = calib_states[i];
    }
  }
  calib_states = tmp;
  
  // First look for transition points, these are local minima for a specific channel
  int last_state = calib_states[0];
  for (int i = 0; i < calib_count; i++) {
    // 2-4 Transition
    if ((last_state == 2 & calib_states[i] == 4)|(last_state == 4 & calib_states[i] == 2)) {
      chan1_feat[4] = map(chan1_calib[i], chan1_min, chan1_max, 0, 1);
      chan1_feat_cnt[4]++;
    // 4-1 Transition
    } else if ((last_state == 4 & calib_states[i] == 1)|(last_state == 1 & calib_states[i] == 4)) {
      chan2_feat[0] = map(chan2_calib[i], chan2_min, chan2_max, 0, 1);
      chan2_feat_cnt[0]++;
    // 1-2 Transition
    } else if ((last_state == 1 & calib_states[i] == 2)|(last_state == 2 & calib_states[i] == 1)) {
      chan3_feat[2] = map(chan3_calib[i], chan3_min, chan3_max, 0, 1);
      chan3_feat_cnt[2]++;
    }
    last_state = calib_states[i];
  }
  
  // Next look for local maxima. Mask out the first and last states to prevent partial peaks.
  int first_state = calib_states[0], final_state = calib_states[calib_count-1];
  tmp = calib_states;
  for (int i = 0; i < calib_count; i++) {
    if (calib_states[i] == first_state) { tmp[i] = -1;} 
    else { break; }
  }
  for (int i = calib_count-1; i > -1; i--) {
    if (tmp[i] == final_state) { tmp[i] = -1; }
    else { break; }
  }
  for (int i = 0; i < calib_count; i++) {
    if (calib_states[i] == 1) {
        chan2_feat[1] = max(chan2_feat[1], map(chan2_calib[i], chan2_min, chan2_max, 0, 1));
        chan3_feat[1] = max(chan3_feat[1], map(chan3_calib[i], chan3_min, chan3_max, 0, 1));
    } else if (calib_states[i] == 2) {
        chan1_feat[3] = max(chan1_feat[3], map(chan1_calib[i], chan1_min, chan1_max, 0, 1));
        chan3_feat[3] = max(chan3_feat[3], map(chan3_calib[i], chan3_min, chan3_max, 0, 1));
    } else if (calib_states[i] == 4) {
        chan1_feat[5] = max(chan1_feat[5], map(chan1_calib[i], chan1_min, chan1_max, 0, 1));        
        chan2_feat[5] = max(chan2_feat[5], map(chan2_calib[i], chan2_min, chan2_max, 0, 1));
    }
  }
}

int calculateState(float c1, float c2, float c3) {
  int s = 0;
  if (c1 < min_thresh) {s= 1;} // was s=s|0b001;
  if (c2 < min_thresh) {s= 2;} // was s|0b010
  if (c3 < min_thresh) {s= 4;} //was s=s|0b100
  return s;
}

long long_min(long a, long b) {
  if (a > b) return b;
  else return a;
}

long long_max(long a, long b) {
  if (a > b) return a;
  else return b;
}
