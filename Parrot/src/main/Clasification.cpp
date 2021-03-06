/*Implementation of clasification stage in vision process
Elaborated by: Ramiro Campos 
Date:04-13-2016 Time:11:00 AM
Description: In this module we implement the parts that correspond to clasification and training. The user will select
             which one he wants to work with. 
*/
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include "common.h"
#include "Clasification.h"
#include "Path_Planning.h"

#define TiempoCiclo 40000
#define t_angulo 1500000
#define t_angulo_b 1000000

using namespace std;
using namespace cv;

extern CHeli *heli;

ifstream fileLeftBack;
ifstream fileLeftBackBack ;
ifstream fileRightBack ;
ifstream fileRightBackBack;

void LeftBackRoutine( ){
   int joypadPitch=0, joypadRoll=0, joypadYaw=0, joypadVerticalSpeed=0, hover=0;

    string line;
    string v1, v2, v3, v4, v5 ;
    fileLeftBack.open("Routines/LeftBack.txt");
      if (fileLeftBack.is_open())
      {
          
          while(!fileLeftBack.eof())
          {
              getline(fileLeftBack, line);
              stringstream   linestream(line);
              getline(linestream, v1, ',' );
              getline(linestream, v2, ',' );
              getline(linestream, v3, ',' );
              getline(linestream, v4, ',' );
              getline(linestream, v5, ',' );

              stringstream convert0(v1);
              convert0 >> joypadPitch;

              stringstream convert1(v2);
              convert1 >> joypadRoll;

              stringstream convert2(v3);
              convert2 >> joypadYaw;

              stringstream convert3(v4);
              convert3 >> joypadVerticalSpeed;

              stringstream convert4(v5);
              convert4 >> hover;


              heli->setAngles(joypadPitch, joypadRoll, joypadYaw, joypadVerticalSpeed, hover);
              usleep(TiempoCiclo);

          }

          fileLeftBack.close();
    
      }
      else {
          cout << "Unable to open file";
      } 
}

void LeftBackBackRoutine( ){
   int joypadPitch=0, joypadRoll=0, joypadYaw=0, joypadVerticalSpeed=0, hover=0;

    string line;
    string v1, v2, v3, v4, v5 ;
    fileLeftBackBack.open("Routines/LeftBackBack.txt");
      if (fileLeftBackBack.is_open())
      {
          
          while(!fileLeftBackBack.eof())
          {
              getline(fileLeftBackBack, line);
              stringstream   linestream(line);
              getline(linestream, v1, ',' );
              getline(linestream, v2, ',' );
              getline(linestream, v3, ',' );
              getline(linestream, v4, ',' );
              getline(linestream, v5, ',' );

              stringstream convert0(v1);
              convert0 >> joypadPitch;

              stringstream convert1(v2);
              convert1 >> joypadRoll;

              stringstream convert2(v3);
              convert2 >> joypadYaw;

              stringstream convert3(v4);
              convert3 >> joypadVerticalSpeed;

              stringstream convert4(v5);
              convert4 >> hover;


              heli->setAngles(joypadPitch, joypadRoll, joypadYaw, joypadVerticalSpeed, hover);
              usleep(TiempoCiclo);

          }

          fileLeftBackBack.close();
    
      }
      else {
          cout << "Unable to open file";
      } 
}

void RightBackRoutine( ){
   int joypadPitch=0, joypadRoll=0, joypadYaw=0, joypadVerticalSpeed=0, hover=0;

    string line;
    string v1, v2, v3, v4, v5 ;
    fileRightBack.open("Routines/RightBack.txt");
      if (fileRightBack.is_open())
      {
          
          while(!fileRightBack.eof())
          {
              getline(fileRightBack, line);
              stringstream   linestream(line);
              getline(linestream, v1, ',' );
              getline(linestream, v2, ',' );
              getline(linestream, v3, ',' );
              getline(linestream, v4, ',' );
              getline(linestream, v5, ',' );

              stringstream convert0(v1);
              convert0 >> joypadPitch;

              stringstream convert1(v2);
              convert1 >> joypadRoll;

              stringstream convert2(v3);
              convert2 >> joypadYaw;

              stringstream convert3(v4);
              convert3 >> joypadVerticalSpeed;

              stringstream convert4(v5);
              convert4 >> hover;


              heli->setAngles(joypadPitch, joypadRoll, joypadYaw, joypadVerticalSpeed, hover);
              usleep(TiempoCiclo);

          }

          fileRightBack.close();
    
      }
      else {
          cout << "Unable to open file";
      } 
}

void RightBackBackRoutine( ){
   int joypadPitch=0, joypadRoll=0, joypadYaw=0, joypadVerticalSpeed=0, hover=0;

    string line;
    string v1, v2, v3, v4, v5 ;
    fileRightBackBack.open("Routines/RightBackBack.txt");
      if (fileRightBackBack.is_open())
      {
          
          while(!fileRightBackBack.eof())
          {
              getline(fileRightBackBack, line);
              stringstream   linestream(line);
              getline(linestream, v1, ',' );
              getline(linestream, v2, ',' );
              getline(linestream, v3, ',' );
              getline(linestream, v4, ',' );
              getline(linestream, v5, ',' );

              stringstream convert0(v1);
              convert0 >> joypadPitch;

              stringstream convert1(v2);
              convert1 >> joypadRoll;

              stringstream convert2(v3);
              convert2 >> joypadYaw;

              stringstream convert3(v4);
              convert3 >> joypadVerticalSpeed;

              stringstream convert4(v5);
              convert4 >> hover;


              heli->setAngles(joypadPitch, joypadRoll, joypadYaw, joypadVerticalSpeed, hover);
              usleep(TiempoCiclo);

          }

          fileRightBackBack.close();
    
      }
      else {
          cout << "Unable to open file";
      } 
}



/*UNUSED*/
/*Training main function
This function is in charge of the training process. Instead of the importance of this part for the proper working of the
system, the code involved is quite simple. In this function the training process is carried out in real time.
*/
int training(const std::vector<region_info>& regions_table,int iterator,std::vector<model_entry>& model_table)
{
  static double hu_mean[2];
  static double hu_variance[2];
  static double hu_stdDev[2];
  double hu_sample_1,hu_sample_2;
  static std::vector<double> hu_data_1,hu_data_2;
  
  //It is very important to work onl with a region at a time. If not, verify filters
  if(regions_table.size() != 1){
    //Tell the system training can't continue until filters are verified
    //printf("Regions table size in training function: %d\n",regions_table.size());
    return BAD_FILTER;
  }
  
  //First verify if iterations are completed.
  if(iterator > (TRAINING_ITERATIONS-1)){
    //If so, it is time to add the region to the model
    //First it is important to verify that model table maximum size has not been reached
    if(model_table.size() < MAXIMUM_MODEL_TABLE_SIZE){
      //Now that we know there is space, compute the parameters used to recognize a region
      hu_mean[fi_1] = hu_mean[fi_1]/hu_data_1.size();
      hu_variance[fi_1] = getVariance(hu_mean[fi_1],hu_data_1);
      hu_stdDev[fi_1] =  sqrt(hu_variance[fi_1]);
      hu_mean[fi_2] = hu_mean[fi_2]/hu_data_2.size();
      hu_variance[fi_2] = getVariance(hu_mean[fi_2],hu_data_2);
      hu_stdDev[fi_2] =  sqrt(hu_variance[fi_2]);
      //Before pushing the parameters to the table, it its important to verify that a condition of homogeneity is met

      //If condition is met,now it's time to push the parameters in the model table
      model_entry region_parameters;
      region_parameters.hu_mean[fi_1] = hu_mean[fi_1];
      region_parameters.hu_variance[fi_1] = hu_variance[fi_1];
      region_parameters.hu_stdDev[fi_1] = hu_stdDev[fi_1];
      region_parameters.hu_mean[fi_2] = hu_mean[fi_2];
      region_parameters.hu_variance[fi_2] = hu_variance[fi_2];
      region_parameters.hu_stdDev[fi_2] = hu_stdDev[fi_2];
   
      //printf("Adding object to model table. Paramters are:\n  Mean_1: %f variance_1: %f standard_deviation_1: %f Mean_2: %f Variance_2: %f Standard_Deviation_2: %f\n",hu_mean[fi_1],hu_variance[fi_1],hu_stdDev[fi_1],hu_mean[fi_2],hu_variance[fi_2],hu_stdDev[fi_2]);
      model_table.push_back(region_parameters);
    }
   
    else{
        //Tell the system model table is full
        return TABLE_FULL;
    }
  }
  else{
    //If iterations are not completed, collect samples
    //First verify if this is the first iteration. If so, reset parameters
    if(iterator == 0){
      hu_mean[fi_1] = 0;
      hu_mean[fi_2] = 0;
      hu_variance[fi_1] = 0;
      hu_variance[fi_2] = 0;
      hu_stdDev[fi_1] = 0;
      hu_stdDev[fi_2] = 0;
      hu_data_1.clear();
      hu_data_2.clear();
    }
    hu_sample_1 = regions_table[0].hu_moments[fi_1];
    hu_sample_2 = regions_table[0].hu_moments[fi_2];
    hu_mean[fi_1] += hu_sample_1;
    hu_mean[fi_2] += hu_sample_2; 
    hu_data_1.push_back(hu_sample_1);
    hu_data_2.push_back(hu_sample_2);
  }
  return SUCCESS;
}

double getVariance(double mean,const std::vector<double>& data)
{
  double temp = 0;
  for(int i=0;i<(int)data.size();i++){
    temp = (mean-data[i])*(mean-data[i]);
  }
  return temp/data.size();
}

/*
 Sometimes it's not possible to carry out an online and real time trainning, instead a static training is the option. This is a simple code in which the values are manually statically laoded by the user in the table.
*/
void static_training(std::vector<model_entry>& model_table){
  //Declaration of entries
  model_entry region_1,region_2,region_3,region_4;

  //Data Filling

  //For region 1 - GLASS: Moves to the right 
  region_1.hu_mean[0] = 0.2113663;
  region_1.hu_mean[1] = 0.0176753;
  region_1.hu_variance[0]= 0;
  region_1.hu_variance[1] = 0;
  region_1.hu_stdDev[0] = 0;
  region_1.hu_stdDev[1] = 0;
  region_1.hu_min[0] = 0.200418;
  region_1.hu_max[0] = 0.218501;
  region_1.hu_min[1] = 0.0132873;
  region_1.hu_max[1] = 0.0207098;
  region_1.move_type = X_MOVE;
  region_1.move_value = RIGHT;

  //For region 2-CUP: Moves backward 
  region_2.hu_mean[0] = 0.2465586;
  region_2.hu_mean[1] = 0.0249729;
  region_2.hu_variance[0]= 0;
  region_2.hu_variance[1] = 0;
  region_2.hu_stdDev[0] = 0;
  region_2.hu_stdDev[1] = 0;
  region_2.hu_min[0] = 0.229524;
  region_2.hu_max[0] = 0.266273;
  region_2.hu_min[1] = 0.019455;
  region_2.hu_max[1] = 0.032304;  
  region_2.move_type = Y_MOVE;
  region_2.move_value = BACK;

  //For region 3 - SPOON: Moves forward
  region_3.hu_mean[0] = 0.4984367;
  region_3.hu_mean[1] = 0.2062653;
  region_3.hu_variance[0]= 0;
  region_3.hu_variance[1] = 0;
  region_3.hu_stdDev[0] = 0;
  region_3.hu_stdDev[1] = 0;
  region_3.hu_min[0] = 0.438063;
  region_3.hu_max[0] = 0.645624;
  region_3.hu_min[1] = 0.154659;
  region_3.hu_max[1] = 0.301342;  
  region_3.move_type = Y_MOVE;
  region_3.move_value = FRONT;

  //For region 2 - PLATE: Moves left
  region_4.hu_mean[0] = 0.184922;
  region_4.hu_mean[1] = 0.002872064;
  region_4.hu_variance[0]= 0;
  region_4.hu_variance[1] = 0;
  region_4.hu_stdDev[0] = 0;
  region_4.hu_stdDev[1] = 0;
  region_4.hu_min[0] = 0.170513;
  region_4.hu_max[0] = 0.187963;
  region_4.hu_min[1] = 0.001627;
  region_4.hu_max[1] = 0.00446638;  
  region_4.move_type = X_MOVE;
  region_4.move_value = LEFT;

  model_table.push_back(region_1);
  model_table.push_back(region_2);
  model_table.push_back(region_3);
  model_table.push_back(region_4);

}

void print_model_table(const std::vector<model_entry>& model_table)
{
  printf("========== MODEL TABLE ============\n");
  for(int i=0;i<(int)model_table.size();i++){
    printf("Entry: %d .Paramters are:\n  Mean_1: %f variance_1: %f standard_deviation_1: %f Mean_2: %f Variance_2: %f Standard_Deviation_2: %f\n",i,model_table[i].hu_mean[fi_1],model_table[i].hu_variance[fi_1],model_table[i].hu_stdDev[fi_1],model_table[i].hu_mean[fi_2],model_table[i].hu_variance[fi_2],model_table[i].hu_stdDev[fi_2]);
  }
  return;
}

/*
This function is in charge of implementing the selection machine in order to determine the number of matches in recognition mode. It's supposed that a model table exists and also that only up to two regions can be recognized.
*/
int recognition(const std::vector<region_info>& regions_table,const std::vector<model_entry>& model_table,const float yaw, const float height_float,CHeli *heli,Mat& obstacles)
{
 double minimum_distance,distance_i;
 bool first_match;
 int minimum_index;
 int letsGo;
 
 double hu1_tolerance_min,hu1_tolerance_max;
 double hu2_tolerance_min,hu2_tolerance_max;
 double x_hu1,x_hu2,w_hu1,w_hu2;
 std::vector<uchar> hits_w; hits_w.clear();
 std::vector<uchar> hits_x(2,0);
 float angle;
  /*
  The selection machine is in charge of determining if a detected region matches an object in the model. The comparison 
  process is quite simple. It's based in considering the region of acceptance as the region between the minimum and maximum.
  If the object falls into acceptance with more than one object, the minimum distance to the average will be the selection
  criteria. 
  */
  
  //If there are no region to analyze do not bother the function trying to compute something, run away and tell this to the
  // system
  if((regions_table.size() < 1) || (regions_table.size()>2)){
    return  NO_MATCH; 
  }
  //If there are regions to analyze, iterate over the whole table in order to apply the process of comparison for each
  //detected region
  for(int region_i=0;region_i<(int)regions_table.size();region_i++){
    x_hu1 = regions_table[region_i].hu_moments[fi_1];
    x_hu2 = regions_table[region_i].hu_moments[fi_2];    
    /*printf("Region: %d hu_1: %f hu_2: %f\n",region_i,x_hu1,x_hu2);
    waitKey();*/
    //This part is the process of comparison with each element in the model
    first_match = true;
    for(int model_i=0;model_i<(int)model_table.size();model_i++){
      //Got average hu moments
      w_hu1 = model_table[model_i].hu_mean[fi_1];
      w_hu2 = model_table[model_i].hu_mean[fi_2];
      //Now get tolerance limits
      hu1_tolerance_min = model_table[model_i].hu_min[0];
      hu1_tolerance_max = model_table[model_i].hu_max[0];
      hu2_tolerance_min = model_table[model_i].hu_min[1];
      hu2_tolerance_max = model_table[model_i].hu_max[1];
   
      //As explained above, the selection condition requires that both moments fall into the acceptace range
      //Conditions if    (hu1_tolerance_min < x_hu1 < hu1_tolerance_max) and (hu2_tolerance_min < x_hu1 < hu2_tolerance_max) 
      if((hu1_tolerance_min <= x_hu1) && (x_hu1 <= hu1_tolerance_max) && 
         (hu2_tolerance_min <= x_hu2) && (x_hu2 <= hu2_tolerance_max)){ 
        /*printf("HIT!!!!!\n");
        printf("Model entry: %d w1_min: %f w1_max: %f x1: %f \n w2_min: %f w2_max: %f x2: %f\n",model_i,hu1_tolerance_min,hu1_tolerance_max,x_hu1,hu2_tolerance_min,hu2_tolerance_max,x_hu2);
        waitKey();*/
        //If the condition is met, we have made a match, and now it's important to determine if it's the smallest one
        distance_i = (w_hu1-x_hu1)*(w_hu1-x_hu1) + (w_hu2-x_hu2)*(w_hu2-x_hu2);
        if(first_match){
        first_match = false;
        minimum_index = model_i;
        minimum_distance = distance_i;
        }
        else if(distance_i < minimum_distance){
          minimum_index = model_i;
          minimum_distance = distance_i;
        }
      }
    }
    //Now if we had a match, let the append it to the hits vector
    if(!first_match){
      hits_w.push_back(minimum_index);
      hits_x.push_back(region_i);
    }
  }
  //Selector machine has finished, so now it's time to validate matches
  if(hits_w.size() < 2){
    printf("Returning no match\n");
    //waitKey();
    return NO_MATCH;
  }
  //By definition our system is capable of detecting just two objects at a time, if more than 3 objects has been detected, something is wrong
  else if(hits_w.size() > 2){
    printf("Match overflow\n");
    //waitKey();
    return MATCH_OVERFLOW;
  }
  else if(hits_w.size() == 2){
    //Just left to get parameters of flying in this format
    /* X Axis corresponds to left or rigth
       Y Axis corresponds to front or back
       Heigth corresponds to heigth of flight(low,medimum,high)*/
    uchar left_rigth,front_back,heigth;
    //There two types of objects:
    //   1-. Objects that indicate if the Parrot will move left or rigth
    //   2-. Objects that indicate if the Parrot will move backward or forward and the heigth of fligth
    //So first ask about what type of obect is each one
    int x_move_obj,y_move_obj;
    //printf("Regions recognized : %d %d\n",hits_w[0],hits_w[1]);
    if(model_table[hits_w[0]].move_type == X_MOVE and model_table[hits_w[1]].move_type == Y_MOVE){
      x_move_obj = hits_w[0]; 
      y_move_obj = hits_w[1];
    }
    else if(model_table[hits_w[1]].move_type == X_MOVE and model_table[hits_w[0]].move_type == Y_MOVE){
      x_move_obj = hits_w[1];
      y_move_obj = hits_w[0];
    }
    else{
      printf("Objects are of the same type\n");
      //waitKey();
      return SAME_OBJECTS;
    }
    left_rigth = model_table[x_move_obj].move_value;
    front_back = model_table[y_move_obj].move_value;
    //Heigth of flight requires a small procesing in order to interpret it in terms of low, medium and high
    angle = cvRound(regions_table[hits_x[y_move_obj]].orientation * 180/3.14159);
    //printf("Angle of flight: %d\n",(int)angle);
    angle = angle < 0?(180-abs(angle)):angle;
    //printf("Angle of flight: %d\n",(int)angle);
    heigth = (int)angle / 60;

    //Now it's time to call the fligth routine
    letsGo = lets_go_parrot(left_rigth,front_back,heigth,yaw,height_float,heli,obstacles); 
  }
  return letsGo;
}

int lets_go_parrot(uchar left_rigth,uchar front_back, uchar angle,const float yaw,const float height,CHeli *heli,Mat& obstacles)
{

  printf("Dear Parrot, would you be so kind of flying with parameters : \n left_rigth: %s front_back: %s heigth: %s\n",left_rigth==LEFT?"LEFT":"RIGTH",front_back==BACK?"BACK":"FRONT",angle==0?"LOW":(angle==1?"MEDIUM":"HIGH"));
  printf("Come on parrot, fucking get up!!!!!!!!!\n");
  //Now it's time to determine what functions to call according to the receive parameters. Remember that
  //the specifications requires that each time there should be one X move and one Y move objects.
  //CASE 1 GLASS-CUP : Fly to the rigth and backward
  if(left_rigth == RIGHT && front_back == BACK){
    printf("GLASS and CUP have been detected\n");
    path_planning(obstacles,RIGHT,BACK);
    imshow("Road map",obstacles);
    switch (angle) {
      case 0: heli->setAngles(0, 0, 0, -6553, 1); usleep(t_angulo);break;
      case 1: break;
      case 2: heli->setAngles(0, 0, 0, 6553, 1);usleep(t_angulo_b);break;
          }
    return 1;
    RightBackRoutine();
    //atrasDerecha(yaw,height,heli);
    //waitKey();
    heli->land();
  }
  //CASE 2 GLASS-SPOON : Fly to the rigth and forward
  else if(left_rigth == RIGHT && front_back == FRONT){
    printf("GLASS and SPOON have been detected\n ");
    path_planning(obstacles,RIGHT,FRONT);
    imshow("Road map",obstacles);
    switch (angle) {
      case 0: heli->setAngles(0, 0, 0, -6553, 1); usleep(t_angulo);break;
      case 1: break;
      case 2: heli->setAngles(0, 0, 0, 6553, 1);usleep(t_angulo_b);break;
          }
    return 2;
    RightBackBackRoutine();
    
    //adelanteDerecha(yaw,height,heli);
    //waitKey();
    heli->land();
  }
  //CASE 3 PLATE-CUP : Fly to the left and backward
  else if(left_rigth == LEFT && front_back == BACK){
    printf("PLATE and CUP have been detected\n");
    path_planning(obstacles,LEFT,BACK);
    imshow("Road map",obstacles);
    switch (angle) {
      case 0: heli->setAngles(0, 0, 0, -6553, 1); usleep(t_angulo);break;
      case 1: break;
      case 2: heli->setAngles(0, 0, 0, 6553, 1);usleep(t_angulo_b);break;
          }
    return 3;
    LeftBackRoutine();
    
    //atrasIzquierda(yaw,height,heli);
    //waitKey();
    heli->land();
  }
  //CASE 4 PLATE-SPOON : Fly to the left and forward
  else if(left_rigth == LEFT && front_back == FRONT){
    printf("PLATE and SPOON have been detected\n");
    path_planning(obstacles,LEFT,FRONT);
    imshow("Road map",obstacles);
    switch (angle) {
      case 0: heli->setAngles(0, 0, 0, -6553, 1); usleep(t_angulo);break;
      case 1: break;
      case 2: heli->setAngles(0, 0, 0, 6553, 1);usleep(t_angulo_b);break;
          }
    return 4;
    LeftBackBackRoutine();
    
    //adelanteIzquierda(yaw,height,heli);    
    //waitKey();
    heli->land();
  }
  //waitKey();
  return 5;
}

