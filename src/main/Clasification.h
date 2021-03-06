#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include "common.h"

#ifndef CLASSIFICATION_H
#define CLASSIFICATION_H 

using namespace std;
using namespace cv;

#define SUCCESS 1
#define BAD_FILTER -1
#define TABLE_FULL -2
#define NO_MATCH -3
#define MATCH_OVERFLOW -4
#define SAME_OBJECTS -5

#define TRAINING_ITERATIONS 10
#define MAXIMUM_MODEL_TABLE_SIZE 4

//Structure to hold the parameters for a region in the model
typedef struct model_entry{
  double hu_mean[2];
  double hu_variance[2];
  double hu_stdDev[2];
  double hu_max[2];
  double hu_min[2];
  uchar move_type;
  uchar move_value;
}model_entry;

//Function prototypes
int training(const std::vector<region_info>& regions_table,int iterator,std::vector<model_entry>& model_table);
double getVariance(double mean,const std::vector<double>& data);
int recognition(const std::vector<region_info>& regions_table,const std::vector<model_entry>& model_table,const float yaw, const float height,CHeli *heli,Mat& obstacles);
int lets_go_parrot(uchar left_rigth,uchar front_back, uchar angle,const float yaw,const float height,CHeli *heli,Mat& obstacles);
void print_model_table(const std::vector<model_entry>& model_table);
void static_training(std::vector<model_entry>& model_table);
void adelanteIzquierda(const float yaw, const float height,CHeli *heli);
void adelanteDerecha(const float yaw, const float height,CHeli *heli);
void atrasIzquierda(const float yaw, const float height,CHeli *heli);
void atrasDerecha(const float yaw, const float height,CHeli *heli);
void LeftBackRoutine();
void LeftBackBackRoutine();
void RightBackRoutine();
void RightBackBackRoutine();

#endif // CLASSIFICATION_H
