/*
   **********************************************************
   **********************************************************
   **      AA                          3/13/05      v1.0   **
   **     AAAA                                             **
   **    AA  AA    aaaa       rr  rr     oooo   nn  nn     **
   **   AA    AA  aaaaaa      rr rrrr   oooooo  nn nnnn    **
   **   AA    AA aa    aa     rrrr  rr oo    oo nnnn  nn   **
   **   AAAAAAAA aa    aa     rrr      oo    oo nnn   nn   **
   **   AAAAAAAA aa    aa     rr       oo    oo nn    nn   **
   **   AA    AA  aa  aa aa   rr        oooooo  nn    nn   **
   **   AA    AA   aaaa   aaa rr         oooo   nn    nn   **
   **********************************************************
   **********************************************************

   void bar(BITMAP *buffer,int x1,int y1,int x2,int y2,double att,double poss,int col1,int col2, int dir)
   int collision_detect(BITMAP *sprite1, int xmin1, int ymin1, BITMAP *sprite2, int xmin2, int ymin2)
   double obj_atn(double X1, double Y1, double X2, double Y2)
   void set_up_timer()
   void update_timeinfo()
   int rnd(int arg1, int arg2)
   double drnd(double arg1, double arg2)
   int cin_num(char *lab, int x, int y, int col)
   char *cin_str(char *lab, int x, int y, int col)
   void append_str(char *dest, char *source)
   void save_filename(const char *filename, int attrib, int param)
   void fill_filename_list(filename_list *&f_file, char *wildcard)

*/

#include <ctime>
#include <cmath>
#include <allegro.h>

class filename_list
{
   public:
   
   const char *name;
   filename_list *prev, *next;
   
   filename_list(){name = NULL; prev = NULL; next = NULL;}
};

/*Various Constants*/

BITMAP *buffer;

const int RED = 63488, GREEN = 2016, BLUE = 31;
const int YELLOW = 65504, PURPLE = 63551, TEAL = 2047;
const int ORANGE = 64480;
const int WHITE = 65535, BLACK = 0, TRANSPARENT = 63519;
const int ROT_NUM = 46603;

const int B_UP = 1, B_RIGHT = 2, B_DOWN = 3, B_LEFT = 4;

const double RTOD = 57.3248, DTOR = 0.017444;

volatile double elapsed_time = 0.0;
void __inc_elapsed_time(){elapsed_time += .001;}
END_OF_FUNCTION(__inc_elapsed_time);
double dt;

void install_timer2();

void bar(BITMAP *buffer,int x1,int y1,int x2,int y2,double att,double poss,int col1,int col2, int dir)
{
  rect(buffer,x1,y1,x2,y2,col1);
  switch(dir)
  {
    case 1:if(y2-((y2-y1)/poss*att)<y2-1)rectfill(buffer,x2-1,y2-1,x1+1,int(y2-((y2-y1)/poss*att)+1),col2);
           if(!att==poss)rectfill(buffer,x2-1,int(y2-((y2-y1)/poss*att)-1),x1+1,y1+1,BLACK);
           break;
    case 2:if((x2-x1)/poss*att+x1-1>x1+1)rectfill(buffer,x1+1,y1+1,int((x2-x1)/poss*att+x1-1),y2-1,col2);
           if(!att==poss)rectfill(buffer,int((x2-x1)/poss*att+x1+1),y1+1,x2-1,y2-1,BLACK);
           break;
    case 3:if((y2-y1)/poss*att+y1>y1+1)rectfill(buffer,x1+1,y1+1,x2-1,int((y2-y1)/poss*att+y1-1),col2);
           if(!att==poss)rectfill(buffer,x1+1,int((y2-y1)/poss*att+y1+1),x2-1,y2-1,BLACK);
           break;
    case 4:if(x2-((x2-x1)/poss*att)<x2-1)rectfill(buffer,x2-1,y1+1,int(x2-((x2-x1)/poss*att)+1),y2-1,col2);
           if(!att==poss)rectfill(buffer,int(x2-((x2-x1)/poss*att)-1),y1+1,x1+1,y2-1,BLACK);
           break;
  }
}

#define max(a, b) (((a) > (b)) ? (a): (b))
#define min(a, b) (((a) < (b)) ? (a): (b))

int collision_detect(BITMAP *sprite1, int xmin1, int ymin1, BITMAP *sprite2, int xmin2, int ymin2)
{
  int xmax1, ymax1, xmax2, ymax2;
  xmin1 -= sprite1->w/2; ymin1 -= sprite1->h/2;
  xmin2 -= sprite2->w/2; ymin2 -= sprite2->h/2;
  xmax1 = xmin1 + sprite1->w; ymax1 = ymin1 + sprite1->h;
  xmax2 = xmin2 + sprite2->w; ymax2 = ymin2 + sprite2->h;
  
  int xmin = max(xmin1, xmin2);
  int ymin = max(ymin1, ymin2);
  int xmax = min(xmax1, xmax2);
  int ymax = min(ymax1, ymax2);
  
  if (!(xmin1 <= xmax2 && xmax1 >= xmin2 && ymin1 <= ymax2 && ymax1 >= ymin2)) return 0; 

  for (int y = ymin; y < ymax; y++) 
  {
    for (int x = xmin; x < xmax; x++) 
    {
      int x1 = x - xmin1, y1 = y - ymin1;
      int x2 = x - xmin2, y2 = y - ymin2;
      int color1 = getpixel(sprite1, x1, y1);
      int color2 = getpixel(sprite2, x2, y2);
      if (color1 != TRANSPARENT && color2 != TRANSPARENT) { return 1; }
    }
  }
  return 0;
}

double obj_atn(double X1, double Y1, double X2, double Y2)
{
  if(X1 == X2 && Y1 == Y2) return 0;
  double rts = atan((Y2-Y1)/abs(int(X2-X1)));
  if(X1>X2) rts=3.14-rts;
  if(rts<0) rts = 6.28 + rts;
  return rts;
}

void set_up_timer()
{
  install_timer();
  srand(time(NULL));
  LOCK_VARIABLE(elapsed_time);
  LOCK_FUNCTION(__inc_elapsed_time);
  install_int_ex(__inc_elapsed_time, BPS_TO_TIMER(1000));
}

void update_timeinfo()
{
  static double last_time;
  dt = elapsed_time - last_time;
  last_time = elapsed_time;
}

int rnd(int arg1, int arg2)
{
   return rand() % (arg2 - arg1 + 1) + arg1;
}

double drnd(double arg1, double arg2)
{
   arg1 = (arg1 * 100); arg2 = (arg2 * 100);
   return (rand() % int(arg2 - arg1 + 1) + arg1) / 100;
}

int cin_num(char *lab, int x, int y, int col)
{
  clear_keybuf();
  char *st_edit = new char[101];
  st_edit[100] = char(NULL);
  char null_char = char(NULL);
  st_edit[0] = null_char;
  int st_pos = 0;
  while(!key[KEY_ENTER])
  {
    rectfill(screen,x,y,screen->w-1,y+10,BLACK);
    textprintf(screen,font,x,y,col,lab); textprintf(screen,font,x+text_length(font,lab),y,col,st_edit);
    st_edit[st_pos] = readkey() % 256;
    st_edit[st_pos+1] = null_char;
    if(!key[KEY_BACKSPACE]) st_pos++;
    else if(st_pos) {st_pos--; st_edit[st_pos] = null_char;}
    int char_num = int(st_edit[st_pos-1]);
    if((char_num < 48 || char_num > 57) && st_pos){st_pos--;st_edit[st_pos] = null_char;}
  }
  int ret_num = 0;
  for(int snum = 0; snum < st_pos; snum++)
    ret_num += int((int(st_edit[snum])-48) * pow(10,st_pos-snum-1));

  return ret_num;
}

char *cin_str(char *lab, int x, int y, int col)
{
  clear_keybuf();
  char *st_edit = new char[101];
  st_edit[100] = char(NULL);
  char null_char = char(NULL);
  st_edit[0] = null_char;
  int st_pos = 0;
  while(!key[KEY_ENTER])
  {
    rectfill(screen,x,y,screen->w-1,y+10,BLACK);
    textprintf(screen,font,x,y,col,lab); textprintf(screen,font,x+text_length(font,lab),y,col,st_edit);
    st_edit[st_pos] = readkey() % 256;
    st_edit[st_pos+1] = null_char;
    if(!key[KEY_BACKSPACE]) st_pos++;
    else if(st_pos) {st_pos--; st_edit[st_pos] = null_char;}
  }
  st_pos--; st_edit[st_pos] = null_char;
  return st_edit;
}

void append_str(char *dest, char *source)
{
   int dest_length = 0, source_length = 0;

   for(int a = 0; a < 999999; a++)
      if(!dest[a]){dest_length = a; a = 999999;}
   for(int a = 0; a < 999999; a++)
      if(!source[a]){source_length = a; a = 999999;}

   for(int a = 0; a < source_length; a++)
      {dest[dest_length] = source[a]; dest_length++;}
      
   dest[dest_length] = char(0);
}

void save_filename(const char *filename, int attrib, int param)
{
   int skip = 0;
   for(int a = 0; a < 1000; a++)
      if(!filename[a] && filename[a-1] == '.') skip = 1;
   
   if(!skip)
   {
      filename_list *&f_file = *(filename_list **)param;

      filename_list *n_file = new filename_list;
      if(f_file) f_file->prev = n_file;
      n_file->next = f_file;
      f_file = n_file;
      n_file->name = ustrdup(filename);
   }
}

void fill_filename_list(filename_list *&f_file, char *wildcard)
{
   for_each_file(wildcard,255,&save_filename,(int)&f_file);
}
