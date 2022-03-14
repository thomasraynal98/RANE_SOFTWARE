#include <iostream>
#include <fstream>
#include <chrono>
#include <cstdlib>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>

//? The main purpose of this code is to test performance of differente openCV
//? optimization technique.

void test1()
{
  cv::setUseOptimized(false);
  std::cout << cv::useOptimized() << std::endl;

  std::string link = "../data_software/map/RB_1.png";
  int sample = 100;
  int64 t0 = cv::getTickCount();  
  for(int i = 0; i < sample; i++)
  {
    cv::Mat map_weighted = cv::imread(link, cv::IMREAD_GRAYSCALE);
  }
  int64 t1 = cv::getTickCount();
  double secs = (t1-t0)/cv::getTickFrequency()/sample;

  std::cout << "Temps moyen d'ouverture d'un PNG est de : " << secs << " secondes. (tentative:" << sample << ")" << std::endl;

  //! Normal    est de 0.015 secondes.
  //! Optimized est de 0.015 secondes.
}

void test2()
{
    cv::setUseOptimized(true);
    std::string link = "../data_software/map/RB_1.png";
    cv::Mat map_weighted = cv::imread(link, cv::IMREAD_GRAYSCALE);
    cv::Mat map_weighted_4Bts;

    int sample = 100;

    int64 t0 = cv::getTickCount();  
    for(int ii = 0; ii < sample; ii++)
    {
      // for(int i=0; i<map_weighted.rows; i++)
      // {
      //   for(int j=0; j<map_weighted.cols; j++)
      //   {
      //     map_weighted.at<uchar>(i,j) = 150;
      //   }
      // }
      typedef cv::Point_<uint8_t> Pixel;
      map_weighted.forEach<Pixel>([](Pixel& pixel, const int position[]) -> void {
        pixel.x = 100;
        pixel.y = 100;
      });
    }

    int64 t1 = cv::getTickCount();
    double secs = (t1-t0)/cv::getTickFrequency()/sample;
    std::cout << "Temps moyen de lecture de tout les pixel d'un PNG est de : " << secs << " secondes. (tentative:" << sample << ")" << std::endl;

    // Show Image inside a window with
    // the name provided
    cv::imshow("Window Name", map_weighted);
  
    // Wait for any keystroke
    cv::waitKey(0);

    //! LECTURE 100 NON OPTIMIZED 8 BITS  : 0.0037 secondes. (x1 img)
    //! LECTURE 100 NON OPTIMIZED 4 BITS  : 0.0037 secondes.
    //! LECTURE 100 OPTIMIZED 8 BITS      : 0.0037 secondes.
    //! ECRITURE 100 OPTIMIZED 8 BITS     : 0.0037 secondes.
    //! ECRITURE 100 NON OPTIMIZED 8 BITS : 0.0037 secondes.
    //! ECRITURE 100 FOREACH 8 BITS       : 0.0017 secondes.
}

void test3()
{
  std::string link = "../data_software/map/RB_1.png";
  cv::Mat map_weighted = cv::imread(link, cv::IMREAD_GRAYSCALE);
  cv::Mat map_weighted_bigger;
  double last1 = 0; double last2 = 0;

  for(double multiplicateur = 1; multiplicateur <= 64; multiplicateur *= 2)
  {
    cv::resize(map_weighted, map_weighted_bigger, cv::Size(0,0),multiplicateur,multiplicateur,6);

    int sample = 1;
    int64 t1 = cv::getTickCount();
    for(int test = 0; test < sample; test++)
    {
      for(int i = 0; i < map_weighted_bigger.cols; i++)
      {
        for(int j = 0; j < map_weighted_bigger.rows; j++)
        {
          map_weighted_bigger.at<uchar>(j,i) = 150;
        }
      }
    }
    int64 t2 = cv::getTickCount();
    typedef cv::Point_<uint8_t> Pixel;
    for(int test = 0; test < sample; test++)
    {
      map_weighted_bigger.forEach<Pixel>([](Pixel& pixel, const int position[]) -> void {
        pixel.x = 100;
        pixel.y = 100;
      });
    }

    int64 t3 = cv::getTickCount();

    std::cout << "size=" << map_weighted_bigger.rows << "x" << map_weighted_bigger.cols;
    double million_pixel = ((double)map_weighted_bigger.rows*map_weighted_bigger.cols) / 1000000;
    std::cout << " " << million_pixel << " Mpixel | M1 = " << (t2-t1)/cv::getTickFrequency()/sample << " secs (+" << (t2-t1)/cv::getTickFrequency()/sample * 100 / last1 << " %)";
    std::cout << "| M2 = " << (t3-t2)/cv::getTickFrequency()/sample << " secs (+" << (t3-t2)/cv::getTickFrequency()/sample * 100 / last1 << " %)" << std::endl;

    last1 = (t2-t1)/cv::getTickFrequency()/sample;
    last2 = (t3-t2)/cv::getTickFrequency()/sample;
    // std::cout << "méthode 1 temps moyen : " << (t2-t1)/cv::getTickFrequency()/sample << " secondes." << std::endl;
    // std::cout << "méthode 2 temps moyen : " << (t3-t2)/cv::getTickFrequency()/sample << " secondes." << std::endl;
  }
}

int main()
{
  //TODO: TEST 1 : importation time between 8 bits and 4 bits png, and optimization code.
  // test1();

  //TODO: TEST 2 : comparer le temps d'acces à toute l'image en fonction du type et de l'optimization.
  // test2();

  //TODO: TEST 3 : comparaison access a toute les values d'une matrix, naive vs foreach.
  test3();
  
  return 0;
}