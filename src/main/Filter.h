//Variables Filtros de los Trackbars
int gauss_value0=0, gauss_value=8;  //Gaussiano
int median_value=8, median_value2=4; //Mediano
int bn0=0,bn2=0; //Treshold
int morph_elem = 2,morph_size = 10, morph_operator = 1; //Morphologic


//Isolate y Average
int limitUp[3];     
int limitDown[3];
double vH = 175;
double vS = 255;
double vV = 170;
int boundH=3, boundS=20, boundV=0;   //HSV

//Average
int minV,minS,minH;
int maxV,maxS,maxH;

//Variables seleccionar recuadro   
int Px; 
int Py;
int Px1;
int Py1;

//Varias
extern int mov;
extern bool Calibrar;

Mat imagenClick; 

//Funcion que calcula promedio RGB, recibiendo cordenadas x y 
void average(int aPx, int aPx1,int aPy,int aPy1)
{
    minV=255;minS=255;minH=255;
    maxV=0;maxS=0;maxH=0;
    uchar* destination;
    int maxX,minX,maxY,minY;
    double counter=0;
    if(aPx >= aPx1)
    {
        maxX= aPx;
        minX= aPx1;
    }
    else
    {
        maxX= aPx1;
        minX= aPx;
    }
        if(Py >= Py1)
    {
        maxY= aPy;
        minY= aPy1;
    }
    else
    {
        maxY= aPy1;
        minY= aPy;
    }
    double tH,tS,tV;
    for(int i=minY; i<= maxY;i++)
    {
        for(int j=minX;j<=maxX;j++)
        {
            destination = (uchar*) imagenClick.ptr<uchar>(i);
            tH+=destination[j * 3];
            tS+=destination[j*3+1];
            tV+=destination[j*3+2];
            counter++;
            if(destination[j * 3] < minH) minH=destination[j*3];
            if(destination[j*3+1] < minS) minS=destination[j*3+1];
            if(destination[j*3+2] < minV) minV=destination[j*3+2];
            if(destination[j * 3] > maxH) maxH=destination[j*3];
            if(destination[j*3+1] > maxS) maxS=destination[j*3+1];
            if(destination[j*3+2] > maxV) maxV=destination[j*3+2];
        }
    }
    vH=floor(tH/counter);
    vV=floor(tV/counter);
    vS=floor(tS/counter);    
}

//Evento del Codigo del click en pantalla; captura el down y up para seleccionar un area
void mouseCoordinatesExampleCallback(int event, int x, int y, int flags, void* param)
{
 
    //uchar* destination;
    switch (event)
    {
        case CV_EVENT_LBUTTONDOWN:
            Px=x;
            Py=y;

            break;
        case CV_EVENT_MOUSEMOVE:

            break;
        case CV_EVENT_LBUTTONUP:
            Px1=x;
            Py1=y;
            //if(Calibrar)
            average(Px,Px1,Py,Py1);
            break;
        case CV_EVENT_RBUTTONDOWN:
        //flag=!flag;
            break;
        
    }
    
}

//Función del filtro, y aislar formas.
void isolate(Mat &out1, Mat inp)
{
    
    limitUp[2]=vV+vV*boundV/100;
    limitUp[1]=vS+vS*boundS/100;
    limitUp[0]=vH+vH*boundH/100;
    limitDown[2]=vV-vV*boundV/100;
    limitDown[1]=vS-vS*boundS/100;
    limitDown[0]=vH-vH*boundH/100;

    bool aver = 1;
    uchar v, s, h;
    int x=inp.rows;
    int y=inp.cols;

    Mat out(inp.rows, inp.cols, CV_8UC3);
    out1=inp.clone();
    //upper_white = np.array([limitUp[0],limitUp[1],limitUp[2]], dtype=np.uint8)
   // lower_white = np.array([imitDown[0],imitDown[1],imitDown[2]], dtype=np.uint8)
    for( int i=0; i< x; i++)
    {
        for (int j = 0; j < y; ++j)
        {
            v = out1.data[i*out1.step + j*3 + 2];
            s = out1.data[i*out1.step + j*3 + 1];
            h = out1.data[i*out1.step + j*3 ];/* code */
            if(aver)
            {
                if(h > limitDown[0] && h < limitUp[0])
                    {//color match
                        out1.data[i*out1.step + j*3 ] = vH;

                    if(s > limitDown[1] && s < limitUp[1])
                       {
                        out1.data[i*out1.step + j*3 + 1] = vS;

                        if(v > limitDown[2] && v < limitUp[2] )
                           {
                                out1.data[i*out1.step + j*3 + 2] = vV;
                           } 
                       }
                    }
                    else
                    {
                        out1.data[i*out1.step + j*3 + 2]=0;
                        out1.data[i*out1.step + j*3 + 1]=0;
                        out1.data[i*out1.step + j*3 ]=0;
                    }
            }
            else
            {
                //inRange(out,Scalar(limitUp[0],limitUp[1],limitUp[2]),Scalar(limitDown[0],limitDown[1],limitDown[2]),out1);

            }   
        }
    }

}

void isolateBin(Mat &out1, Mat inp)
{
    uchar h;
    int x=inp.rows;
    int y=inp.cols;

    Mat out(inp.rows, inp.cols, CV_8UC1);
    out1=inp.clone();
    for( int i=0; i< x; i++)
    {
        for (int j = 0; j < y; ++j)
        {   
            h = out1.data[i*out1.step + j ];/* code */
                if(h < 10)
                    {//color match
                        out1.data[i*out1.step + j ] = 0;
                    }
                    else
                    {
                        out1.data[i*out1.step + j ]=255;
                    }
  
        }
    }

}

void filtrado(Mat &imageRGB, Mat  &salida )
{

    Mat imageHSV, imageMorph;
    Mat imageBN, imageGaussian, imageGaussian0, imageHSVMedian;
    Mat imageGRAY;
    Mat imageRGB2,imageBin;
    Mat imageBNMedian;
    Mat filtroHSV;

    //TRACKBARS y WINDOW
    string Window_Calibrate = "Calibrar";
    string Window_Isolate = "Isolate";
    string Window_imageClick = "imageClick";
    setMouseCallback(Window_imageClick, mouseCoordinatesExampleCallback);

    if (mov == 0 && Calibrar) //Ejecutar solo 1 vez
    {

        namedWindow (Window_Calibrate, CV_WINDOW_AUTOSIZE);
        namedWindow (Window_Isolate, CV_WINDOW_AUTOSIZE);
        namedWindow(Window_imageClick);

        createTrackbar("H :\n", Window_Calibrate, &boundH, 100);
        createTrackbar("S :\n", Window_Calibrate, &boundS, 100);
        createTrackbar("V :\n", Window_Calibrate, &boundV, 100);
        createTrackbar("First Gaussian:\n", Window_Calibrate, &gauss_value0, 30);
        createTrackbar("First Mediano:\n", Window_Calibrate, &median_value, 30);
        createTrackbar("Threshold value :\n", Window_Calibrate, &bn0, 255);
        createTrackbar("Tipo Threshold :\n", Window_Calibrate, &bn2, 1);
        createTrackbar("Second Gaussian:\n", Window_Calibrate, &gauss_value, 30);
        createTrackbar("Second Mediano:\n", Window_Calibrate, &median_value2, 30);
        createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", Window_Calibrate, &morph_operator, 4);
        createTrackbar("Element:\n 0: Rect - 1: Cross - 2: Ellipse", Window_Calibrate, &morph_elem, 2);
        createTrackbar("Kernel size:\n 2n +1", Window_Calibrate, &morph_size, 21);


        moveWindow(Window_Calibrate,0,0);
        moveWindow(Window_imageClick,400,10);
        moveWindow(Window_Isolate, 400,350);
        mov=1;
    }

    //INICIA EL FILTRADO
    //Etapa 0 Cambiar RGB - HSV
    cvtColor(imageRGB, imageHSV, CV_BGR2HSV);
    imageHSV.copyTo(imagenClick);

    //Primera Etapa -- ISOLATE
    isolate(filtroHSV, imageHSV); //Es alreves los inputs AISLAR COLOR
    
    //Segunda Etapa - Filtro Gaussiano y Mediano
    GaussianBlur( filtroHSV, imageGaussian0, Size( gauss_value0+gauss_value0%2+1, gauss_value0+gauss_value0%2+1 ), 0, 0 );
    medianBlur( imageGaussian0, imageHSVMedian, median_value+median_value%2+1); 
    ////imshow("filtrosGausyMedian", imageHSVMedian);
    
    //Tercera Etapa - Convertir de RGB a GRIS y luego Binarizar
    cvtColor(imageHSVMedian, imageRGB2, CV_HSV2BGR);
    cvtColor(imageRGB2, imageGRAY, CV_BGR2GRAY);
    threshold(imageGRAY, imageBN, bn0, 255, bn2 );
    ////imshow("filtroBN", imageBN);
    
    //Cuarta Etapa
    Mat KernelMorph = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    morphologyEx( imageBN, imageMorph, morph_operator+2, KernelMorph );
    ////imshow("filtroMorhp", imageMorph);
    
    //Quinta Etapa- Segundo Mediano y Gaussiano
    GaussianBlur( imageMorph, imageGaussian, Size( gauss_value+gauss_value%2+1, gauss_value+gauss_value%2+1 ), 0, 0 );
    medianBlur( imageGaussian, imageBNMedian, median_value2+median_value2%2+1); 
    ////imshow("filtroGAus2", imageBNMedian);
    
    //Sexta Etapa - Aplicar nueva Binarizacion
    isolateBin(imageBin,imageBNMedian);

    salida=imageBin;

    //Mostrar Pantallas
    if (Calibrar){
        imshow(Window_imageClick, imageRGB);
        imshow(Window_Isolate, filtroHSV);
    }

}

void closeAw()
{
    destroyWindow("Calibrar");
    destroyWindow("Isolate");
    destroyWindow("imageClick");
    destroyWindow("imageBinarizada");
}


