#include "CImg.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>

#define TIME_QUANT 	1000
#define TIME_HOUR	3600
#define G 		6.6738399999999988e-11

using namespace std;

struct vect
{
  double coord[3];
  vect() { coord[0] = 0.0; coord[1] = 0.0; coord[2] = 0.0; };
  vect operator * (vect b) 
  {
    vect res;
    for (int i=0;i<3;i++)
      res[i] = coord[i] * b.coord[i];
    return res;
  }
  vect operator * (double b)
  {
    vect res;
    for (int i = 0; i<3; i++)
      res[i] = coord[i] * b;
    return res;
  }
  vect operator - (vect b) 
  {
    vect res;
    for (int i=0;i<3;i++)
      res[i] = coord[i] - b.coord[i];
    return res;
  }
  vect operator + (vect b)
  {
    vect res;
    for (int i = 0; i<3; i++)
      res[i] = coord[i] + b.coord[i];
    return res;
  }
  vect operator / (double b)
  {
    vect res;
    for (int i=0;i<3;i++)
      res[i] = coord[i] / b;
    return res;
  }
  double & operator [] (int i)
  {
    return coord[i];
  }
  bool operator == (vect & b)
  {
      return (coord[0] == b.coord[0] && coord[1] == b.coord[1] && coord[2] == b.coord[2]);
  }
  vect norm()
  {
    double a = coord[0] * coord[0] + coord[1] * coord[1] + coord[2] * coord[2];
    vect res = (*this) / (a*sqrt(a));
    return res;
  }
};

double mag_2(vect a)
{
  double res = 0;
  for (int i = 0; i<3; i++)
    res += a.coord[i] * a.coord[i];
  return res;
}

struct object
{
  vect position;
  vect velocity;
  object operator + (object & b)
  {
      object res;
      res.velocity = (*this).velocity + b.velocity;
      res.position = (*this).position + b.position;
      return res;
  };
  object operator - (object & b)
  {
      object res;
      res.velocity = (*this).velocity - b.velocity;
      res.position = (*this).position - b.position;
      return res;
  };
  object operator * (double b)
  {
      object res;
      res.velocity = (*this).velocity * b;
      res.position = (*this).position * b;
      return res;
  };
};

char OBJ_NUM;

struct objects
{
    vector<object> objs;
    objects() {
        objs.resize(OBJ_NUM);
    }
    int size()
    {
        return objs.size();
    };
    object & operator [] (int i)
    {
        return objs[i];
    };
    objects operator + (objects b)
    {
        objects res;
        for (int i = 0; i < (*this).size(); i++)
            res[i] = (*this)[i] + b[i];
        return res;
    }
    objects operator * (double b)
    {
        objects res;
        for (int i = 0; i < (*this).size(); i++)
            res[i] = (*this)[i] * b;
        return res;
    }
};

struct object_descr
{
  char name[16];
  double mass;
  vector<char> nearobj;
  char color[3];
};

struct {
  objects U;
  vector<object_descr> descrs;
} state;

using namespace std;

bool state_init ()
{
  FILE * f = fopen ("config.txt","r");

  if (!f)
  {
    cout << "Failed to find config file please specify one\n"
	    "In the following format:\n"
            "Name:%s Mass:%f R:%d G:%d B:%d PosX:%f PosY:%f PosZ:%f VelX:%f VelY:%f VelZ:%f \n"
            "Example:\n"
            "NumberOfObjects:3\n"
            "Name:Sun Mass:1.98892e30 R:100 G:100 B:0 PosX:0.0 PosY:0.0 PosZ:0.0 VelX:0.0 VelY:0.0 VelZ:0.0\n"
            "Name:Earth Mass:5.972e24 R:0 G:0 B:100 PosX:0.0 PosY:149600000000 PosZ:0.0 VelX:-29783.000 VelY:0.0 VelZ:0.0\n"
            "Name:Moon Mass:7.36e22 R:50 G:50 B:50 PosX:0.0 PosY:149215600000 PosZ:0.0 VelX:-29781.978 VelY:0.0 VelZ:0.0\n"
	    "\n";
    return false;
  }

  fscanf (f, "NumberOfObjects:%d\n", &OBJ_NUM);

  state.descrs.resize(OBJ_NUM);
  state.U.objs.resize(OBJ_NUM);
  for (int i = 0; i < OBJ_NUM; i++)
  {
    vect pos, vel;
    int res = fscanf(f, "Name:%s Mass:%lf R:%d G:%d B:%d PosX:%lf PosY:%lf PosZ:%lf VelX:%lf VelY:%lf VelZ:%lf\n",
                     state.descrs[i].name, &state.descrs[i].mass,
                     &state.descrs[i].color[0], &state.descrs[i].color[1], &state.descrs[i].color[2],
                     &pos[0], &pos[1], &pos[2], &vel[0], &vel[1], &vel[2]);
    state.U[i].position = pos;
    state.U[i].velocity = vel;
    if (!res)
    {
      cout << "file error on: " << i << " line please be carefull\n";
      return false;
    }
  }

  fclose (f);

  for (int i = 0; i < OBJ_NUM; i++)
  {
    char num = 0;
    state.descrs[i].nearobj.resize(OBJ_NUM - 1);
    for (char j = 0; j < OBJ_NUM; j++)
      if (i != j)
        state.descrs[i].nearobj[num++] = j;
  }
  return true;
}

objects F(objects U)
{
    double dt = TIME_QUANT;
    objects res;
    for (int i = 0; i < OBJ_NUM; i++)
    {
        vect a;
        vect v = U[i].velocity;
        vect S0 = U[i].position;

        for (int j = 0; j < OBJ_NUM - 1; j++)
        {
            char num = state.descrs[i].nearobj[j];
            vect R = U[num].position - S0;
            a = a + R.norm() * (state.descrs[num].mass * G);
        }

        res[i].velocity = a;
        res[i].position = v;
    }
    return res;
}

void evaluate()
{
    objects k1, k2, k3, k4;
    double dt = TIME_QUANT;
    k1 = F(state.U) * dt;
    k2 = F(state.U + k1 * 0.5) * dt;
    k3 = F(state.U + k2 * 0.5) * dt;
    k4 = F(state.U + k3);
    state.U = state.U + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (1.0 / 6.0);
}

//Mapping part

#define SIZE 		900
#define X_0 		SIZE/2
#define Y_0 		X_0
#define BINSCALE 	29

#define sn 0.087155742748
#define cs 0.996194698092

//view matrix

double VIEW[4][4] = {
    { 1, 0, 0, 1 },
    { 0, 1, 0, 1 },
    { 0, 0, 1, 1 }, 
    { 0, 0, 0, 1 } };

//Rotates X

double XRR[4][4] = { 
    { 1, 0, 0, 0 },
    { 0,cs,sn, 0 },
    { 0,-sn,cs, 0 },
    { 0, 0, 0, 1 } };

double XRL[4][4] = {
    { 1, 0, 0, 0 },
    { 0,cs,-sn, 0 },
    { 0,sn,cs, 0 },
    { 0, 0, 0, 1 } };

//Rotate Y

double YRR[4][4] = { 
    { cs,0,sn, 0 },
    { 0, 1, 0, 0 },
    { -sn, 0,cs, 0 },
    { 0, 0, 0, 1 } };

double YRL[4][4] = {
    { cs,0,-sn, 0 },
    { 0, 1, 0, 0 },
    { sn, 0,cs, 0 },
    { 0, 0, 0, 1 } };

//Rotate Z

double ZRR[4][4] = {
    { cs,sn, 0, 0 },
    { -sn,cs, 0, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 } };

double ZRL[4][4] = {
    { cs,-sn, 0, 0 },
    { sn,cs, 0, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 } };

//Shift X

double XSR[4][4] = {
    { 1, 0, 0, 0},
    { 0, 1, 0, 0},
    { 0, 0, 1, 0},
    { 0, 0, 0, 1 } };

double XSL[4][4] = {
    { 1, 0, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 } };

//Shift Y

double YSR[4][4] = {
    { 1, 0, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 } };

double YSL[4][4] = {
    { 1, 0, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 } };

//Shift Z

double ZSR[4][4] = {
    { 1, 0, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 } };

double ZSL[4][4] = {
    { 1, 0, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 } };

double SCALE = 1000000000.0;

void mul(double M[4][4], double R[4][4])
{
    double t[4][4];
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            t[i][j] = 0;
            for (int k = 0; k < 4; k++)
                t[i][j] += M[i][k] * R[k][j];
        }
    }
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            R[i][j] = t[i][j];
}

void mul(double M[4][4], vect & v)
{
    double t[4];
    for (int i = 0; i < 4; i++)
    {
        t[i] = 0;
        for (int k = 0; k < 4; k++)
            t[i] += M[i][k] * ((k == 3) ? 1 : v[k]);
    }
    for (int i = 0; i < 4; i++)
        v[i] = t[i];
}

//hack to represent moon-like objects
vect moon_hack (vect obj)
{
    vect & E = state.U[1].position;
    return obj + ((obj - E) * 100);
}

void
position2disp(vect obj, int * x, int * y)
{
  //center is always the most heavy body
  vect & center = state.U[0].position;

  vect res = obj;// -center;

  mul(VIEW, res);

  //z elem is ignored
  long long obj_x = res[0] / SCALE;
  long long obj_y = res[1] / SCALE;

  *x = obj_x + X_0;
  *y = -obj_y + Y_0;
}

void update_shift()
{
    XSR[0][3] = SCALE*10;
    XSL[0][3] = -SCALE*10;
    YSR[1][3] = SCALE*10;
    YSL[1][3] = -SCALE*10;
    ZSR[2][3] = SCALE*10;
    ZSL[2][3] = -SCALE*10;
}

//Visulaizatin part.

using namespace cimg_library;

int main()
{
#if 0
  CImg<unsigned char> img(640,400,1,3); // Define a 640x400 color image with 8 bits per color component.
  img.fill(0); // Set pixel values to 0 (color : black)
  unsigned char purple[] = { 255,0,255 }; // Define a purple color
  img.draw_text(100,100,"Hello World",purple); // Draw a purple "Hello world" at coordinates (100,100).
  img.display("My first CImg code"); // Display the image in a display window.
#endif
  CImg<unsigned char> img(800,800,1,3);

  if (!state_init ())
    return -1;
  
  CImgDisplay disp(img);

  //remembering previos places
  vector<vector<vect>> dots; dots.resize (OBJ_NUM);
  short dot_idx = 0;

  unsigned t = 0;

  while (1)
  {
    img.fill(0);
    evaluate ();
    for (int i = 0; i < OBJ_NUM; i++)
    {
      unsigned char color[] = { state.descrs[i].color[0],
                                state.descrs[i].color[1],
                                state.descrs[i].color[2] };
      //draw dots
      if (dot_idx >= dots[i].size())
          dots[i].resize (dot_idx + 1);

      for (int j = 0; j < dots[i].size(); j++)
      {
          int xd, yd; position2disp (dots[i][j], &xd, &yd);
          img.draw_circle(xd, yd, 1, color);
      }

      //member new dots
      if (!(t & 0xff))
      {
          dots[i][dot_idx] = (i == 2) ? moon_hack(state.U[i].position) : state.U[i].position;
          if (++dot_idx > 100) dot_idx = 0;
      }

      //draw body
      int x, y; position2disp(i == 2 ? moon_hack(state.U[i].position) : state.U[i].position, &x, &y);
      img.draw_circle(x, y, 4 * (OBJ_NUM - i),
                      CImg<unsigned char>::vector(state.descrs[i].color[0],
                      state.descrs[i].color[1], state.descrs[i].color[2]).data());
      //draw name
      img.draw_text(x + 5, y + 5, state.descrs[i].name, color);
    }
    disp.flush();
    disp.display(img);
    
    //react on user input
    switch (disp.key())
    {
    //rotate 
    // X
    case cimg::keyW: mul(XRR, VIEW); break;
    case cimg::keyS: mul(XRL, VIEW); break;

    // Y
    case cimg::keyA: mul(YRR, VIEW); break;
    case cimg::keyD: mul(YRL, VIEW); break;

    // Z
    case cimg::keyQ: mul(ZRR, VIEW); break;
    case cimg::keyE: mul(ZRL, VIEW); break;

    // X
    case cimg::keyH: mul(XSR, VIEW); break;
    case cimg::keyF: mul(XSL, VIEW); break;

    // Y
    case cimg::keyT: mul(YSR, VIEW); break;
    case cimg::keyG: mul(YSL, VIEW); break;

    // Z
    case cimg::keyR: mul(ZSR, VIEW); break;
    case cimg::keyY: mul(ZSL, VIEW); break;

    //Scale
    case cimg::keyX: SCALE *= 1.2; update_shift(); break;
    case cimg::keyZ: SCALE /= 1.2; update_shift(); break;
    }
    //Sleep(1000);
    t = (t + 1) & 0xffffffff;
  }
  return 0;
}


#if 0
Compilation notes:
    Microsoft Visual C++ 6.0, Visual Studio.NET and Visual Express Edition : Use project files and solution files provided in the CImg Library package (directory 'compilation/') to see how it works.
    Intel ICL compiler : Use the following command to compile a CImg-based program with ICL :
    icl /Ox hello_world.cpp user32.lib gdi32.lib
    g++ (MingW windows version) : Use the following command to compile a CImg-based program with g++, on Windows :
    g++ -o hello_word.exe hello_word.cpp -O2 -lgdi32
    g++ (Linux version) : Use the following command to compile a CImg-based program with g++, on Linux :
    g++ -o hello_word.exe hello_world.cpp -O2 -L/usr/X11R6/lib -lm -lpthread -lX11
    g++ (Solaris version) : Use the following command to compile a CImg-based program with g++, on Solaris :
    g++ -o hello_word.exe hello_world.cpp -O2 -lm -lpthread -R/usr/X11R6/lib -lrt -lnsl -lsocket
    g++ (Mac OS X version) : Use the following command to compile a CImg-based program with g++, on Mac OS X :
    g++ -o hello_word.exe hello_world.cpp -O2 -lm -lpthread -I/usr/X11R6/include -L/usr/X11R6/lib -lm -lpthread -lX11
    Dev-Cpp : Use the project file provided in the CImg library package to see how it works.
#endif
