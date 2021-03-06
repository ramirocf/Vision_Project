#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "SDL/SDL.h"
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <fstream>
#include <opencv/cv.h>
#include "common.h"
#include "Segmentation.h"
#include "Clasification.h"
#include "CHeli.h"
#include "Filter.h"
#include "Path_Planning.h"
 

#define TRAINING 1
#define RECOGNITION 2

using namespace std;
using namespace cv;

//**********VARIABLES ORIGINALES***********
bool stop = false;
CRawImage *image;
/*DEFINITION OF HELI OBJECT*/
CHeli *heli;
float pitch, roll, yaw, height;
int hover=0;
// Joystick related
SDL_Joystick* m_joystick;
bool useJoystick;
int joypadRoll, joypadPitch, joypadVerticalSpeed, joypadYaw;
bool navigatedWithJoystick, joypadTakeOff, joypadLand, joypadHover;
string ultimo = "init";

//**********MODO DE OPERACION**********
//0 = camara del Drone, 1 = Camara Web, 2 = Imagen local
int CamSelDrone=0;

//********ARCHIVOS*********f**
ifstream myfileR ;
static ifstream fileLeftBack0 ;
static ifstream fileLeftBackBack0;
static ifstream fileRightBack0 ;
static ifstream fileRightBackBack0;
ofstream myfileS ;
bool Start2Save = false, Start2Read = false;
int joypadSave = 0, joypadStopSave=0, joypadRead = 0, joypadLeftBack=0, joypadRightBack=0, joypadLeftBackBack=0, joypadRightBackBack=0;
bool goLeftBack= false, goRightBack = false, goLeftBackBack= false, goRightBackBack=false;

//*********VARIABLES MAIN****************
bool save=false,Calibrar=false;
time_t last =time(&last), timer ;
bool guardar=false,mas=false;
int counName=0,counNameLoad=1;
int mov=0;

//Tiempo
long long
timeval_diff(struct timeval *difference,struct timeval *end_time, struct timeval *start_time){
  struct timeval temp_diff;

  if(difference==NULL)
  {
    difference=&temp_diff;
  }

  difference->tv_sec =end_time->tv_sec -start_time->tv_sec ;
  difference->tv_usec=end_time->tv_usec-start_time->tv_usec;

  /* Using while instead of if below makes the code slightly more robust. */

  while(difference->tv_usec<0)
  {
    difference->tv_usec+=1000000;
    difference->tv_sec -=1;
  }

  return 1000000LL*difference->tv_sec+
                   difference->tv_usec;

} /* timeval_diff() */




struct timeval earlier;
struct timeval later;
long long conti =0;

//***************FUNCIONES ***********************
void delay(int seg){
    time(&last);
    bool activo = true;
    int diferencia=0;
    char x;
    while( diferencia< seg && activo){
        time(&timer);
        diferencia = difftime(timer,last);
        x =waitKey(50);
        if (x=='s'){
            activo=false;
        }
        cout << diferencia;
    }
}

// Convert CRawImage to Mat
void rawToMat( Mat &destImage, CRawImage* sourceImage)
{   
    uchar *pointerImage = destImage.ptr(0);
    
    for (int i = 0; i < 240*320; i++)
    {
        pointerImage[3*i] = sourceImage->data[3*i+2];
        pointerImage[3*i+1] = sourceImage->data[3*i+1];
        pointerImage[3*i+2] = sourceImage->data[3*i];
    }
}

void RunRoutine(){
    string line;
    string v1, v2, v3, v4, v5 ;

    if (myfileR.is_open())
    {
        
        if(!myfileR.eof())
        {
            getline(myfileR, line);
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
        }
        else{
            Start2Read = false;
            heli->land();
            myfileR.close();
        }

    }
    else {
        myfileR.open("Read.txt");
        cout << "Unable to open file";
    } 
}

void LeftBackR( ){
    string line;
    string v1, v2, v3, v4, v5 ;

    if (fileLeftBack0.is_open())
    {
        
        if(!fileLeftBack0.eof())
        {
            getline(fileLeftBack0, line);
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
        }
        else{
            goLeftBack = false;
            heli->land();
            fileLeftBack0.close();
        }

    }
    else {
        fileLeftBack0.open("Routines/LeftBack.txt");
        cout << "Unable to open file";
    } 

}

void LeftBackBackR( ){
    string line;
    string v1, v2, v3, v4, v5 ;

    if (fileLeftBackBack0.is_open())
    {
        
        if(!fileLeftBackBack0.eof())
        {
            getline(fileLeftBackBack0, line);
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
        }
        else{
            goLeftBackBack = false;
            heli->land();
            fileLeftBackBack0.close();
        }

    }
    else {
        fileLeftBackBack0.open("Routines/LeftBackBack.txt");
        cout << "Unable to open file";
    } 

}

void RightBackR( ){
    string line;
    string v1, v2, v3, v4, v5 ;

    if (fileRightBack0.is_open())
    {
        
        if(!fileRightBack0.eof())
        {
            getline(fileRightBack0, line);
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
        }
        else{
            goRightBack = false;
            heli->land();
            fileRightBack0.close();
        }

    }
    else {
        fileRightBack0.open("Routines/RightBack.txt");
        cout << "Unable to open file";
    } 

}

void RightBackBackR( ){
    string line;
    string v1, v2, v3, v4, v5 ;

    if (fileRightBackBack0.is_open())
    {
        
        if(!fileRightBackBack0.eof())
        {
            getline(fileRightBackBack0, line);
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
        }
        else{
            goRightBackBack = false;
            heli->land();
            fileRightBackBack0.close();
        }

    }
    else {
        fileRightBackBack0.open("Routines/RightBack.txt");
        cout << "Unable to open file";
    } 

}

int main(int argc,char* argv[])
{
    //Capturar imagen camara web
    VideoCapture camera = VideoCapture(0);
    //Establishing connection with the quadcopter
    heli = new CHeli();
    //This class holds the image from the drone 
    image = new CRawImage(320,240);
    // Initial values for control   
    pitch = roll = yaw = height = 0.0;
    joypadPitch = joypadRoll = joypadYaw = joypadVerticalSpeed = 0.0;

    //A R C H I V O S

    myfileS.open ("Saved.txt");

    //-------CREACION DE MATRICES------ 
    Mat imageRGB = Mat(240, 320, CV_8UC3);
    Mat LapCam = Mat(240, 320, CV_8UC3);//Utilizar web cam
    
    string imageName;
    string imageNameLoad("img/imagen1.jpg");
    Mat imageLocal; //= imread(imageName.c_str(), CV_LOAD_IMAGE_COLOR); // Read the file
    Mat imageBinarizada;
    Mat segmentedImage;
    Mat obstacles;
    std::vector<region_info> regions_table;
    std::vector<model_entry> model_table;

    //-------PANTALLAS------ 
    namedWindow ("ImagenRGB", CV_WINDOW_AUTOSIZE);
    namedWindow("ImagenBinarizada", CV_WINDOW_AUTOSIZE);
    namedWindow("Road Map",CV_WINDOW_AUTOSIZE);
    moveWindow("ImagenRGB", 800,10 );
    moveWindow("ImagenBinarizada", 800,350);
    moveWindow("Road Map", 800,700);

    //------Initialize Joystick
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);
    useJoystick = SDL_NumJoysticks() > 0;
    if (useJoystick)
    {
        SDL_JoystickClose(m_joystick);
        m_joystick = SDL_JoystickOpen(0);
    }

    bool recognize = false;
    int result;

    //Static training loading
    static_training(model_table);
    print_model_table(model_table);
    //waitKey();

    //**** WHILE PRINCIPAL *******


    while (stop == false)
    {
        gettimeofday(&earlier,NULL);
        model_table.clear();
        static_training(model_table);
        //print_model_table(model_table);
        //waitKey();
        //time(&timer);
        //conti=difftime(timer,last);
        // Clear the console
        printf("\033[2J\033[1;1H");

        if (useJoystick)
        {
            SDL_Event event;
            SDL_PollEvent(&event);

            joypadRoll = SDL_JoystickGetAxis(m_joystick, 2) /5;
            if(abs(joypadRoll) < 5461/2)joypadRoll=0;

            joypadPitch = SDL_JoystickGetAxis(m_joystick, 5) /5;
            if(abs(joypadPitch) < 5461/2)joypadPitch=0;

            joypadVerticalSpeed = SDL_JoystickGetAxis(m_joystick, 1) /5;
            if(abs(joypadVerticalSpeed) < 5461/2)joypadVerticalSpeed=0;

            joypadYaw = SDL_JoystickGetAxis(m_joystick, 0) /5;
            if(abs(joypadYaw) < 5461/2)joypadYaw=0;

            joypadTakeOff = SDL_JoystickGetButton(m_joystick, 1); 
            joypadLand = SDL_JoystickGetButton(m_joystick, 2);
            joypadHover = !SDL_JoystickGetButton(m_joystick, 3);

            joypadSave = SDL_JoystickGetButton(m_joystick,13);
            joypadRead = SDL_JoystickGetButton(m_joystick,9);
            joypadStopSave = SDL_JoystickGetButton(m_joystick,0);

            joypadLeftBack = SDL_JoystickGetButton(m_joystick,4);
            joypadRightBack = SDL_JoystickGetButton(m_joystick,5);
            joypadLeftBackBack = SDL_JoystickGetButton(m_joystick,6);
            joypadRightBackBack = SDL_JoystickGetButton(m_joystick,7);



            if (joypadSave == 1)
                Start2Save = true;
            if (joypadRead == 1)
                Start2Read = true;

            if (joypadLeftBack == 1)
                goLeftBack = true;
            if (joypadRightBack == 1)
                goRightBack = true;
            if (joypadLeftBackBack == 1)
                goLeftBackBack = true;
            if (joypadRightBackBack ==1)
                goRightBackBack= true;

            if (joypadStopSave == 1)
            {
                Start2Save= false;
                fileRightBack0.close();
                fileRightBackBack0.close();
                fileLeftBack0.close();
                fileLeftBackBack0.close();
            }

        }

        // prints the drone telemetric data, helidata struct contains drone angles, speeds and battery status
        printf("===================== Parrot Basic Example =====================\n\n");
        fprintf(stdout, "Angles  : %.2lf %.2lf %.2lf \n", helidata.phi, helidata.psi, helidata.theta);
        fprintf(stdout, "Speeds  : %.2lf %.2lf %.2lf \n", helidata.vx, helidata.vy, helidata.vz);
        fprintf(stdout, "Battery : %.0lf \n", helidata.battery);
        fprintf(stdout, "Hover   : %d \n", !joypadHover);
        fprintf(stdout, "Joypad  : %d \n", useJoystick ? 1 : 0);
        fprintf(stdout, "Roll    : %d \n", joypadRoll);
        fprintf(stdout, "Pitch   : %d \n", joypadPitch);
        fprintf(stdout, "Yaw     : %d \n", joypadYaw);
        fprintf(stdout, "V.S.    : %d \n", joypadVerticalSpeed);
        fprintf(stdout, "TakeOff : %d \n", joypadTakeOff);
        fprintf(stdout, "Land    : %d \n", joypadLand);
        fprintf(stdout, "Conti   : %lld \n", conti);
        //fprintf(stdout, "Navigating with Joystick: %d \n", navigatedWithJoystick ? 1 : 0);
        //cout<<"Pos X: "<<Px<<" Pos Y: "<<Py<<endl<<"Pos X1: "<<Px1<<" Pos Y1: "<<Py1<<endl<<"Valor RGB: ("<<vR<<","<<vG<<","<<vB<<")"<<endl;
        /*cout<<"Valor RGB: ("<<vV<<","<<vS<<","<<vH<<")"<<endl;
        cout<<" minH: "<<minH<<" minS: "<<minS<<" minV: "<<minV<<endl;
        cout<<" maxH: "<<maxH<<" maxS: "<<maxS<<" maxV: "<<maxV<<endl;
        cout<<" averH: "<<vH<<" averS: "<<vS<<" averV: "<<vV<<"   "<< "time" <<conti<<endl;*/
        //image is captured

        if(CamSelDrone==0)
        {
            //Trabajar con Drone
            heli->renewImage(image);
            rawToMat(imageRGB, image);
        }
        else if (CamSelDrone==1)
        {
            //Trabajar camara web
            camera.read(LapCam);
            imageRGB = LapCam;
        }
        else if (CamSelDrone==2)
        {
            //Trabajar imagen local
            imageLocal = imread(imageNameLoad.c_str(), CV_LOAD_IMAGE_COLOR);
            imageRGB = imageLocal;
        }
        if(!Calibrar)
        {
            imshow("ImagenRGB", imageRGB);
        }

        //***************ETAPA DE FILTRADO***************************
        filtrado(imageRGB, imageBinarizada);
        imshow("ImagenBinarizada", imageBinarizada);
        //*********************************************************

        //Acomodar Pantallas
         if (mov == 0)
        {
            moveWindow("imageBinarizada",400,710);
            mov=1;
        }

        //Show segmented image. This could be tought with training purposes
         if(save)
         {
           regions_table.clear();
           segmentedImage = Mat::zeros(imageBinarizada.rows,imageBinarizada.cols,CV_8UC3);
           segmentation_and_characterization(imageBinarizada,segmentedImage,regions_table);
           imshow("Segmented image",segmentedImage);
           moveWindow("Segmented image", 1200,10 );
           print_regions(regions_table);
           //waitKey();
           save=false;
         }

         if(guardar)//new
         {
                ostringstream convert;
                convert << counName;
                imageName="img/imagen";
                imageName.append(convert.str());
                imageName.append(".jpg");
            imwrite(imageName, imageRGB);
            guardar = false;
            counName++;
         }
         if(mas)
         {  
            counNameLoad++;
            ostringstream convertLoad;
            convertLoad << counNameLoad;
            imageNameLoad="img/imagen";
            imageNameLoad.append(convertLoad.str());
            imageNameLoad.append(".jpg");
            mas=false;
         }

         if (recognize == true){
           regions_table.clear();
           segmentedImage = Mat::zeros(imageBinarizada.rows,imageBinarizada.cols,CV_8UC3);
           segmentation_and_characterization(imageBinarizada,segmentedImage,regions_table);
           imshow("Segmented image",segmentedImage);
           result = recognition(regions_table,model_table,yaw,height,heli,obstacles);
           recognize = false;

           switch (result){
                case 1 : goRightBackBack=true; break;
                case 2 : goRightBack=true; break;
                case 3 : goLeftBackBack=true; break;
                case 4 : goLeftBack=true; break;
                case 5 : break;
                default : break;

           }

           /*
           if(result == SUCCESS) {
             //waitKey();
             recognize = false;
             heli->land();
             //stop = true;
           }*/
           recognize = false;

         }
        //*************CONTROL DRONE TECLADO************
        char key = waitKey(5);
        switch (key) {
            case 'a': yaw = -20000.0; break;
            case 'd': yaw = 20000.0; break;
            case 'w': height = -20000.0; break;
            case 's': height = 20000.0; break;
            case 'q': heli->takeoff(); break;
            case 'e': heli->land(); break;
            case 'z': heli->switchCamera(0); break;
            case 'x': heli->switchCamera(1); break;
            case 'c': heli->switchCamera(2); break;
            case 'v': heli->switchCamera(3); break;
            case 'j': roll = -20000.0; break;
            case 'l': roll = 20000.0; break;
            case 'i': pitch = -20000.0; break;
            case 'k': pitch = 20000.0; break;
            case 'h': hover = (hover + 1) % 2; break;
            case 'n': break;
            case 'b':save=true;break;
            case 'r': recognize = true;break;
            case 27: stop = true; break;
            case '1': Calibrar = true; mov=0; break;
            case '2': Calibrar = false; closeAw();break;
            case '3': delay(5); break;
            case '0': guardar=true;break;
            case '9':mas=true;break;
            default: pitch = roll = yaw = height = 0.0;
        }

        if (joypadTakeOff) {
            heli->takeoff();
        }
        if (joypadLand) {
            heli->land();
        }

        /*********LANZAR RUTINAS ********/
        //Lo que hacen estas funciones es obtener las variables de joypadRoll, joypadPitch, etc... y cambiar su valor global
        if (Start2Read)
            RunRoutine();
        if (goLeftBack)
            LeftBackR();
        if (goLeftBackBack)
            LeftBackBackR();
        if (goRightBack)
            RightBackR();
        if (goRightBackBack)
            RightBackBackR();
        /********************************/


        //setting the drone angles
        if (joypadRoll != 0 || joypadPitch != 0 || joypadVerticalSpeed != 0 || joypadYaw != 0)
        {
            heli->setAngles(joypadPitch, joypadRoll, joypadYaw, joypadVerticalSpeed, !joypadHover);
            navigatedWithJoystick = true;

            if (Start2Save)
            myfileS << joypadPitch << "," << joypadRoll << "," << joypadYaw << "," << joypadVerticalSpeed << "," << !joypadHover << "\n";
        }
        else
        {
            heli->setAngles(pitch, roll, yaw, height, hover);
            navigatedWithJoystick = false;

            if (Start2Save)
            myfileS << joypadPitch << "," << joypadRoll << "," << joypadYaw << "," << joypadVerticalSpeed << "," << !joypadHover << "\n";

        }


        

        usleep(15000);

        gettimeofday(&later,NULL);

        conti = timeval_diff(NULL,&later,&earlier);
        
    }
    
    //Finalizar
    heli->land();
    SDL_JoystickClose(m_joystick);
    delete heli;
    delete image;

    myfileS.close();
    fileRightBack0.close();
    fileRightBackBack0.close();
    fileLeftBack0.close();
    fileLeftBackBack0.close();
    return 0;
}
