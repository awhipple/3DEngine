#include "aaron.cpp"
#include <allegro.h>
#include <cmath>
#include <stdio.h>
#include <string.h>

#define Z_BUF_OFFSET                1000
#define MAX_POINTS_PER_POLY         4
#define MAX_VERTICES_PER_OBJECT     80
#define MAX_POLYS_PER_OBJECT        40
#define MAX_OBJECTS                 32
#define MAX_POLYS_PER_FRAME         128

volatile double elapsed_time = 0.0;
void __inc_elapsed_time() { elapsed_time += .001; }
END_OF_FUNCTION(__inc_elapsed_time);
double dt, fps;

const int poly_clip_min_x = 0, poly_clip_min_y = 0, poly_clip_max_x = 640, poly_clip_max_y = 480;

const int VD = 250;                     //VD = Veiwing Distance

typedef float matrix_4x4[4][4];         //standard 4x4 homogenous matrix
typedef float matrix_1x4[4];            //a 1x4 matrix or row vector

typedef struct vector_3d_typ
{
  float x, y, z, w;
} point_3d, vector_3d, *point_3d_ptr, *vector_3d_ptr;

typedef struct dir_3d_typ
{
  int ang_x, ang_y, ang_z;   //angles relative to x, y, and z
} dir_3d, *dir_3d_ptr;

typedef struct polygon_typ
{
  int num_points;                        //number of points in a polygon
  int vertex_list[MAX_POINTS_PER_POLY];  //the index number of vertices
  int color;                             //color of polygon
  int shade;                             //the final shade of color after lighting
  int shading;                           //type of lighting, flat or constant shading
  int two_sided;                         //flags if the polygon is two sided
  int visible;                           //used to remove backfaces
  int active;                            //used to turn faces on and off
  int clipped;                           //flags that polygon has been clipped or removed
  float normal_length;                   //pre-computed magnitude of normal
} polygon2, *polygon2_ptr;

typedef struct facet_typ
{
  int num_points;                             //number of points in a polygon
  int color;                                  //color of polygon
  int shade;                                  //the final shade of color after lighting
  int shading;                                //type of lighting, flat or constant shading
  int two_sided;                              //flags if the polygon is two sided
  int visible;                                //used to remove backfaces
  int active;                                 //used to turn faces on and off
  int clipped;                                //flags that polygon has been clipped or removed
  point_3d vertex_list[MAX_POINTS_PER_POLY];  //the index number of vertices
  float normal_length;                        //pre-computed magnitude of normal
} facet, *facet_ptr;

typedef struct object_typ
{
  int id;                                            //identification number of object
  int num_vertices;                                  //total number of vertices in object
  point_3d vertices_local[MAX_VERTICES_PER_OBJECT];  //local vertices
  point_3d vertices_world[MAX_VERTICES_PER_OBJECT];  //world vertices
  point_3d vertices_camera[MAX_VERTICES_PER_OBJECT]; //camera vertices
  int num_polys;                                     //the number of polygons in the object
  polygon2 polys[MAX_POLYS_PER_OBJECT];              //the polygons of the object
  float radius;                                      //the average radius of the object
  int state;                                         //state of object
  point_3d world_pos;                                //position of object in world co-ordinates
} object, *object_ptr;

void Make_vector_3d(point_3d_ptr init, point_3d_ptr term, vector_3d_ptr result);
float Vector_Mag_3d(vector_3d_ptr v);
float Dot_Product_3d(vector_3d_ptr u, vector_3d_ptr v);
void Cross_Product_3d(vector_3d_ptr u, vector_3d_ptr v, vector_3d_ptr normal);
void Mat_Identity_4x4(matrix_4x4 a);
void Mat_Zero_4x4(matrix_4x4 a);
void Mat_Mul_4x4_4x4(matrix_4x4 a, matrix_4x4 b, matrix_4x4 result);
void Mat_Mul_1x4_4x4(matrix_1x4 a, matrix_4x4 b, matrix_1x4 result);
char *PLG_Get_Line(char *string, int max_length, FILE *fp);
int PLG_Load_Object(object_ptr the_object, char *filename, float scale);
int PGB_Load_Object(object_ptr the_object, char *filename, float scale);
object load_object(char *filename, float scale);
float Compute_Object_Radius(object_ptr the_object);
void Translate_Object(object_ptr the_object, int x_trans, int y_trans, int z_trans);
void Position_Object(object_ptr the_object, int x, int y, int z);
void Scale_Object(object_ptr the_object, float scale_factor);
void Rotate_Object(object_ptr the_object, int angle_x, int angle_y, int angle_z);
void Draw_Object_Wire(BITMAP *buffer, object_ptr the_object);
void Draw_Object_Solid(BITMAP *buffer, object_ptr the_object);
void Build_Look_Up_Tables();
void draw_triangle(BITMAP *tobuf, int x1, int y1, int x2, int y2, int x3, int y3, int color);
void draw_triangle_fb(BITMAP *tobuf, int x1, int y1, int x2, int y2, int x3, int y3, int color);
void draw_triangle_ft(BITMAP *tobuf, int x1, int y1, int x2, int y2, int x3, int y3, int color);
void Remove_Backfaces_And_Shade(object_ptr the_object);
void Draw_Tri_3D_Z(BITMAP *tobuf, int x1, int y1, int z1, int x2, int y2, int z2, int x3, int y3, int z3, int color);
void Draw_T_Tri_3D_Z(BITMAP *tobuf,int x1,int y1, int z1, int x2, int y2, int z2, int x3, int y3, int z3, int color);
void Draw_B_Tri_3D_Z(BITMAP *tobuf,int x1,int y1, int z1, int x2, int y2, int z2, int x3, int y3, int z3, int color);
int X3D(float x, float z);
int Y3D(float y, float z);
void set_obj_points(object &obj);
void update_timeinfo();

float cos_look[360+1], sin_look[360+1], gdp;

float z_buffer[640][480];


void Make_vector_3d(point_3d_ptr init, point_3d_ptr term, vector_3d_ptr result)
{
  result->x = term->x - init->x;
  result->y = term->y - init->y;
  result->z = term->z - init->z;    
}

float Vector_Mag_3d(vector_3d_ptr v)
{
  return((float)sqrt(v->x*v->x + v->y*v->y + v->z*v->z));
}

float Dot_Product_3d(vector_3d_ptr u, vector_3d_ptr v)
{
  return( (u->x * u->x) + (u->y * v->y) + (u->z * v->z));
}

void Cross_Product_3d(vector_3d_ptr u, vector_3d_ptr v, vector_3d_ptr normal)
{
  normal->x = (u->y*v->z - u->z*v->y);
  normal->y = (u->x*v->z - u->z*v->x);
  normal->z = (u->x*v->y - u->y*v->x);
}

void Mat_Identity_4x4(matrix_4x4 a)
{
  a[0][1] = a[0][2] = a[0][3] = 0;
  a[1][0] = a[1][2] = a[1][3] = 0;
  a[2][0] = a[2][1] = a[2][3] = 0;
  a[3][0] = a[3][1] = a[3][2] = 0;

  a[0][0] = a[1][1] = a[2][2] = a[3][3] = 1;
}

void Mat_Zero_4x4(matrix_4x4 a)
{
  a[0][0] = a[0][1] = a[0][2] = a[0][3] = 0;
  a[1][0] = a[1][1] = a[1][2] = a[1][3] = 0;
  a[2][0] = a[2][1] = a[2][2] = a[2][3] = 0;
  a[3][0] = a[3][1] = a[3][2] = a[3][3] = 0;
}

void Mat_Mul_4x4_4x4(matrix_4x4 a, matrix_4x4 b, matrix_4x4 result)
{
  int index_i, index_j, index_k;
  float sum;
  for(index_i = 0; index_i < 4; index_i++)
  {
    for(index_j = 0; index_j < 4; index_j++)
    {
      sum = 0;
      for(index_k = 0; index_k < 4; index_k++)
      {
        sum += a[index_i][index_k]*b[index_k][index_j];
      }
      result[index_i][index_j] = sum;
    }
  }
}

void Mat_Mul_1x4_4x4(matrix_1x4 a, matrix_4x4 b, matrix_1x4 result)
{
  int index_j, index_k;
  float sum;
  for(index_j = 0; index_j < 4; index_j++)
  {
    sum = 0;
    for(index_k = 0; index_k < 4; index_k++)
    {
      sum += a[index_k]*b[index_k][index_j];
    }
    result[index_j] = sum;
  }
}

char *PLG_Get_Line(char *string, int max_length, FILE *fp)
{
  char buffer[80];
  int length, index = 0, index_2 = 0, parsed = 0;
  while(1)
  {
    if(!fgets(buffer,max_length,fp)) return(NULL);
    length = strlen(buffer);
    buffer[length-1] = 0;
    index = 0;
    while(buffer[index]==' ') index++;
    parsed = 0;
    index_2 = 0;
    while(!parsed)
    {
      if(buffer[index] != '#' && buffer[index] != ';')
      {
        string[index_2] = buffer[index];
        if(string[index_2] == 0) parsed = 1;
        index++;
        index_2++;
      }
      else
      {
        string[index_2] = 0;
        parsed = 1;
      }
    }
    if(strlen(string)) return(string);
  }
}
  
int PLG_Load_Object(object_ptr the_object, char *filename, float scale)
{
  FILE *fp;
  static int id_number = 0;
  char buffer[80], object_name[32], *token;
  unsigned int total_vertices, total_polys, num_vertices, color_des, logical_color, shading, index, index_2,
               vertex_num, vertex_0, vertex_1, vertex_2;
  float x, y, z;
  vector_3d u, v, normal;
  if((fp=fopen(filename,"r"))==NULL)
  {
    printf("\nCouldn't open file %s",filename);
    return(0);
  }
  if(!PLG_Get_Line(buffer,80,fp))
  {
    printf("\n1Error with PLG file %s",filename);
    fclose(fp);
    return(0);
  }
  sscanf(buffer,"%s %d %d",object_name,&total_vertices,&total_polys);
  the_object->num_vertices = total_vertices;
  the_object->num_polys = total_polys;
  the_object->state = 1;
  the_object->world_pos.x = 0;
  the_object->world_pos.y = 0;
  the_object->world_pos.z = 0;    
  the_object->id = id_number++;
  for(index = 0; index < total_vertices; index++);
  {
    if(!PLG_Get_Line(buffer,80,fp))
    {
      printf("\n2Error with PLG file %s",filename);
      fclose(fp);
      return(0);
    }
    sscanf(buffer,"%f %f %f",&x,&y,&z);
    the_object->vertices_local[index].x = x*scale;
    the_object->vertices_local[index].y = y*scale;
    the_object->vertices_local[index].z = z*scale;        
  }
  for(index = 0; index < total_polys; index++)
  {
    if(!PLG_Get_Line(buffer,80,fp))
    {
      printf("\n3Error with PLG file %s",filename);
      fclose(fp);
      return(0);
    }
    if(!(token = strtok(buffer," ")))
    {
      printf("\n4Error with PLG file %s",filename);
      fclose(fp);
      return(0);
    }
    if(token[0]=='0' && (token[1]=='x' || token[1]=='X'))
    {
      sscanf(&token[2],"%x",&color_des);
    }
    else
    {
      color_des = atoi(token);
    }
    logical_color = color_des & 0x00ff;
    shading = color_des >> 12;
    if(!(token = strtok(NULL," ")))
    {
      printf("\n5Error with PLG file %s",filename);
      fclose(fp);
      return(0);
    }
    if((num_vertices = atoi(token))<=0)
    {
      printf("\n6Error with PLG file (number of vertices) %s",filename);
      fclose(fp);
      return(0);
    }
    the_object->polys[index].num_points = num_vertices;
    the_object->polys[index].color = logical_color;
    the_object->polys[index].shading = shading;
    the_object->polys[index].two_sided = 0;
    the_object->polys[index].visible = 1;
    the_object->polys[index].clipped = 0;
    the_object->polys[index].active = 1;
    for(index_2 = 0; index_2 < num_vertices; index_2++)
    {
      if(!(token = strtok(NULL," ")))
      {
        printf("\n7Error with PLG file %s",filename);
        fclose(fp);
        return(0);
      }
      vertex_num = atoi(token);
      the_object->polys[index].vertex_list[index_2] = vertex_num;
    }
    vertex_0 = the_object->polys[index].vertex_list[0];
    vertex_1 = the_object->polys[index].vertex_list[1];
    vertex_2 = the_object->polys[index].vertex_list[2];
    Make_vector_3d((point_3d_ptr)&the_object->vertices_local[vertex_0],
                   (point_3d_ptr)&the_object->vertices_local[vertex_1],
                   (vector_3d_ptr)&u);
    Make_vector_3d((point_3d_ptr)&the_object->vertices_local[vertex_0],
                   (point_3d_ptr)&the_object->vertices_local[vertex_2],
                   (vector_3d_ptr)&v);
    Cross_Product_3d((vector_3d_ptr)&v,
                     (vector_3d_ptr)&u,
                     (vector_3d_ptr)&normal);
    the_object->polys[index].normal_length = Vector_Mag_3d((vector_3d_ptr)&normal);
  }
  fclose(fp);
  Compute_Object_Radius(the_object);
  return(1);
}

int PGB_Load_Object(object_ptr the_object, char *filename, float scale)
{
  BITMAP *file = load_bitmap(filename,0);
  static int id_number = 0;
  unsigned int total_vertices, total_polys, num_vertices, color_des, logical_color, shading, index, index_2,
               vertex_num, vertex_0, vertex_1, vertex_2;
  float x, y, z;
  vector_3d u, v, normal;
  total_vertices = col_to_num(getpixel(file,0,0));
  total_polys = col_to_num(getpixel(file,1,0));
  the_object->num_vertices = total_vertices;
  the_object->num_polys = total_polys;
  the_object->state = 1;
  the_object->world_pos.x = 0;
  the_object->world_pos.y = 0;
  the_object->world_pos.z = 0;    
  the_object->id = id_number++;

  for(index = 0; index < total_vertices; index++)
  {
    x = col_to_num(getpixel(file,1,index+1));
    y = col_to_num(getpixel(file,3,index+1));
    z = col_to_num(getpixel(file,5,index+1));
    if(!col_to_num(getpixel(file,0,index+1))) x *= -1;
    if(!col_to_num(getpixel(file,2,index+1))) y *= -1;
    if(!col_to_num(getpixel(file,4,index+1))) z *= -1;        
    the_object->vertices_local[index].x = x*scale;
    the_object->vertices_local[index].y = y*scale;
    the_object->vertices_local[index].z = z*scale;        
  }
  for(index = 0; index < total_polys; index++)
  {
    
    color_des = getpixel(file,0,total_vertices+index+1);
    num_vertices = col_to_num(getpixel(file,1,total_vertices+index+1));
    
    logical_color = color_des;
    shading = 0;
    the_object->polys[index].num_points = num_vertices;
    the_object->polys[index].color = logical_color;
    the_object->polys[index].shading = shading;
    the_object->polys[index].two_sided = 0;
    the_object->polys[index].visible = 1;
    the_object->polys[index].clipped = 0;
    the_object->polys[index].active = 1;
    for(index_2 = 0; index_2 < num_vertices; index_2++)
    {
      vertex_num = col_to_num(getpixel(file,index_2+2,total_vertices+index+1));
      the_object->polys[index].vertex_list[index_2] = vertex_num;
    }
    vertex_0 = the_object->polys[index].vertex_list[0];
    vertex_1 = the_object->polys[index].vertex_list[1];
    vertex_2 = the_object->polys[index].vertex_list[2];
    Make_vector_3d((point_3d_ptr)&the_object->vertices_local[vertex_0],
                   (point_3d_ptr)&the_object->vertices_local[vertex_1],
                   (vector_3d_ptr)&u);
    Make_vector_3d((point_3d_ptr)&the_object->vertices_local[vertex_0],
                   (point_3d_ptr)&the_object->vertices_local[vertex_2],
                   (vector_3d_ptr)&v);
    Cross_Product_3d((vector_3d_ptr)&v,
                     (vector_3d_ptr)&u,
                     (vector_3d_ptr)&normal);
    the_object->polys[index].normal_length = Vector_Mag_3d((vector_3d_ptr)&normal);
  }
  Compute_Object_Radius(the_object);
  return(1);
}

object load_object(char *filename, float scale)
{
  object tem;
  PGB_Load_Object(&tem,filename,scale);
  return tem;
}

float Compute_Object_Radius(object_ptr the_object)
{
  float new_radius, x, y, z;
  int index;
  the_object->radius = 0;
  for(index = 0; index < the_object->num_vertices; index++)
  {
    x = the_object->vertices_local[index].x;
    y = the_object->vertices_local[index].y;
    z = the_object->vertices_local[index].z;
    new_radius = (float)sqrt(x*x + y*y + z*z);
    if(new_radius > the_object->radius) the_object->radius = new_radius;
  }
  return(the_object->radius);
}

void Translate_Object(object_ptr the_object, int x_trans, int y_trans, int z_trans)
{
  the_object->world_pos.x += x_trans;
  the_object->world_pos.y += y_trans;
  the_object->world_pos.z += z_trans;
}

void Position_Object(object_ptr the_object, int x, int y, int z)
{
  the_object->world_pos.x = x;
  the_object->world_pos.y = y;
  the_object->world_pos.z = z;
}    

void Scale_Object(object_ptr the_object, float scale_factor)
{
  int curr_poly, curr_vertex;
  float scale_2;
  for(curr_vertex = 0; curr_vertex < the_object->num_vertices; curr_vertex++)
  {
    the_object->vertices_local[curr_vertex].x *= scale_factor;
    the_object->vertices_local[curr_vertex].y *= scale_factor;
    the_object->vertices_local[curr_vertex].z *= scale_factor;
  }
  scale_2 = scale_factor*scale_factor;
  for(curr_poly = 0; curr_poly < the_object->num_polys; curr_poly++)
  {
    the_object->polys[curr_poly].normal_length *= scale_2;
  }
  the_object->radius *= scale_factor;
}

void Rotate_Object(object_ptr the_object, int angle_x, int angle_y, int angle_z)
{
  int index, product = 0;
  matrix_4x4 rotate_x, rotate_y, rotate_z, rotate, temp;
  float temp_x, temp_y, temp_z;
  if(angle_x==0 && angle_y==0 && angle_z==0) return;
  Mat_Identity_4x4(rotate);
  if(angle_x)
  {
    Mat_Identity_4x4(rotate_x);
    rotate_x[1][1] = ( cos_look[angle_x]);
    rotate_x[1][2] = ( sin_look[angle_x]);
    rotate_x[2][1] = (-sin_look[angle_x]);
    rotate_x[2][2] = ( cos_look[angle_x]);
  }
  if(angle_y)
  {
    Mat_Identity_4x4(rotate_y);
    rotate_y[0][0] = ( cos_look[angle_y]);
    rotate_y[0][2] = (-sin_look[angle_y]);
    rotate_y[2][0] = ( sin_look[angle_y]);
    rotate_y[2][2] = ( cos_look[angle_y]);
  }
  if(angle_z)
  {
    Mat_Identity_4x4(rotate_z);
    rotate_z[0][0] = ( cos_look[angle_z]);
    rotate_z[0][1] = ( sin_look[angle_z]);
    rotate_z[1][0] = (-sin_look[angle_z]);
    rotate_z[1][1] = ( cos_look[angle_z]);
  }
  if(angle_x) product|=4;
  if(angle_y) product|=2;
  if(angle_z) product|=1;
  switch(product)
  {
    case 0: break;
    case 1: rotate = rotate_z;
            break;
    case 2: rotate = rotate_y;
            break;
    case 3: Mat_Mul_4x4_4x4(rotate_y,rotate_z,rotate);
            break;
    case 4: rotate = rotate_x;
            break;
    case 5: Mat_Mul_4x4_4x4(rotate_x,rotate_z,rotate);
            break;
    case 6: Mat_Mul_4x4_4x4(rotate_x,rotate_y,rotate);
            break;
    case 7: Mat_Mul_4x4_4x4(rotate_x,rotate_y,temp);
            Mat_Mul_4x4_4x4(temp,rotate_z,rotate);
            break;
  }
  for(index = 0; index < the_object->num_vertices; index++)
  {
    temp_x = the_object->vertices_local[index].x * rotate[0][0] +
             the_object->vertices_local[index].y * rotate[1][0] +
             the_object->vertices_local[index].z * rotate[2][0];
    temp_y = the_object->vertices_local[index].x * rotate[0][1] +
             the_object->vertices_local[index].y * rotate[1][1] +
             the_object->vertices_local[index].z * rotate[2][1];
    temp_z = the_object->vertices_local[index].x * rotate[0][2] +
             the_object->vertices_local[index].y * rotate[1][2] +
             the_object->vertices_local[index].z * rotate[2][2];
    the_object->vertices_local[index].x = temp_x;
    the_object->vertices_local[index].y = temp_y;
    the_object->vertices_local[index].z = temp_z;
  }
}

void Draw_Object_Wire(BITMAP *buffer, object_ptr the_object)
{
  int curr_poly, curr_vertex, vertex;
  float x1, y1, z1, x2, y2, z2;
  int ix1, iy1, ix2, iy2;
  for(curr_poly = 0; curr_poly < the_object->num_polys; curr_poly++)
  {
    if(the_object->polys[curr_poly].visible==0 ||
       the_object->polys[curr_poly].clipped )
      continue;
      
    for(curr_vertex = 0; curr_vertex < the_object->polys[curr_poly].num_points - 1; curr_vertex++)
    {
      vertex = the_object->polys[curr_poly].vertex_list[curr_vertex];
      x1 = the_object->vertices_camera[vertex].x;
      y1 = the_object->vertices_camera[vertex].y;
      z1 = the_object->vertices_camera[vertex].z;
      vertex = the_object->polys[curr_poly].vertex_list[curr_vertex+1];
      x2 = the_object->vertices_camera[vertex].x;
      y2 = the_object->vertices_camera[vertex].y;
      z2 = the_object->vertices_camera[vertex].z;
      x1 = (320 + x1 * VD / z1);
      y1 = (240 - y1 * VD / z1);
      x2 = (320 + x2 * VD / z2);
      y2 = (240 - y2 * VD / z2);
      ix1 = int(x1);
      iy1 = int(y1);
      ix2 = int(x2);
      iy2 = int(y2);
      line(buffer,ix1,iy1,ix2,iy2,the_object->polys[curr_poly].color);
    }
    ix1 = int(x2);
    iy1 = int(y2);
    vertex = the_object->polys[curr_poly].vertex_list[0];
    x2 = the_object->vertices_camera[vertex].x;
    y2 = the_object->vertices_camera[vertex].y;
    z2 = the_object->vertices_camera[vertex].z;
    x2 = (320 + x2 * VD / z2);
    y2 = (240 - y2 * VD / z2);
    ix2 = int(x2);
    iy2 = int(y2);
    line(buffer,ix1,iy1,ix2,iy2,the_object->polys[curr_poly].color);
  }
}

void Draw_Object_Solid(BITMAP *buffer, object_ptr the_object)
{
  int curr_poly, curr_vertex, vertex;
  float x1, y1, z1, x2, y2, z2;
  int ix1, iy1, ix2, iy2;
  for(curr_poly = 0; curr_poly < the_object->num_polys; curr_poly++)
  {
    if(the_object->polys[curr_poly].visible==0 ||
       the_object->polys[curr_poly].clipped )
      continue;
    
    int ix[3], iy[3], iz[3];  
    for(curr_vertex = 0; curr_vertex < 3; curr_vertex++)
    {
      vertex = the_object->polys[curr_poly].vertex_list[curr_vertex];
      x1 = the_object->vertices_camera[vertex].x;
      y1 = the_object->vertices_camera[vertex].y;
      z1 = the_object->vertices_camera[vertex].z;
      x1 = (320 + x1 * VD / z1);
      y1 = (240 - y1 * VD / z1);
      ix[curr_vertex] = int(x1);
      iy[curr_vertex] = int(y1);
      iz[curr_vertex] = int(z1);
    }
    Draw_Tri_3D_Z(buffer,ix[0],iy[0],iz[0],ix[1],iy[1],iz[1],ix[2],iy[2],iz[2],the_object->polys[curr_poly].color);
  }
}

void Build_Look_Up_Tables()
{
  int angle;
  float rad;
  for(angle = 0; angle <= 360; angle++)
  {
    rad = (float) (3.14159*(float)angle/(float)180);
    cos_look[angle] = (float)cos(rad);
    sin_look[angle] = (float)sin(rad);
  }
}

void draw_triangle_fb(BITMAP *tobuf, int x1, int y1, int x2, int y2, int x3, int y3, int color)
{
  int temp_x, temp_y;
  if(y2<y1)
  {
    temp_x = x2;
    temp_y = y2;
    x2     = x1;
    y2     = y1;
    x1     = temp_x;
    y1     = temp_y;
  }
  if(y3<y1)
  {
    temp_x = x3;
    temp_y = y3;
    x3     = x1;
    y3     = y1;
    x1     = temp_x;
    y1     = temp_y;
  }
  if(x2<x3)
  {
    temp_x = x2;
    temp_y = y2;
    x2     = x3;
    y2     = y3;
    x3     = temp_x;
    y3     = temp_y;
  }
  double height = y3 - y1;
  double xs = x1, xe = x1, sll = (x1 - x3) / height, slr = (x2 - x1) / height;
  for(int temp_y = y1; temp_y <= y2; temp_y++)
  {
    line(tobuf,int(xs),temp_y,int(xe),temp_y,color);
    xs -= sll; xe += slr;
  }
}

void draw_triangle_ft(BITMAP *tobuf, int x1, int y1, int x2, int y2, int x3, int y3, int color)
{
  int temp_x, temp_y;
  if(y2>y1)
  {
    temp_x = x2;
    temp_y = y2;
    x2     = x1;
    y2     = y1;
    x1     = temp_x;
    y1     = temp_y;
  }
  if(y3>y1)
  {
    temp_x = x3;
    temp_y = y3;
    x3     = x1;
    y3     = y1;
    x1     = temp_x;
    y1     = temp_y;
  }
  if(x2>x3)
  {
    temp_x = x2;
    temp_y = y2;
    x2     = x3;
    y2     = y3;
    x3     = temp_x;
    y3     = temp_y;
  }
  double height = y1 - y3;
  double xs = x1, xe = x1, sll = (x1 - x2) / height, slr = (x3 - x1) / height;
  for(int temp_y = y1; temp_y >= y2; temp_y--)
  {
    line(tobuf,int(xs),temp_y,int(xe),temp_y,color);
    xs -= sll; xe += slr;
  }
}

void draw_triangle(BITMAP *tobuf, int x1, int y1, int x2, int y2, int x3, int y3, int color)
{
  int t1x1, t1y1, t1x2, t1y2, t1x3, t1y3, t2x1, t2y1, t2x2, t2y2, t2x3, t2y3;
  int temp_x, temp_y;
  if(y2<y1)
  {
    temp_x = x2;
    temp_y = y2;
    x2     = x1;
    y2     = y1;
    x1     = temp_x;
    y1     = temp_y;
  }
  if(y3<y1)
  {
    temp_x = x3;
    temp_y = y3;
    x3     = x1;
    y3     = y1;
    x1     = temp_x;
    y1     = temp_y;
  }
  if(y3<y2)
  {
    temp_x = x3;
    temp_y = y3;
    x3     = x2;
    y3     = y2;
    x2     = temp_x;
    y2     = temp_y;
  }
  if(y1==y2)      draw_triangle_ft(tobuf,x1,y1,x2,y2,x3,y3,color);
  else if(y2==y3) draw_triangle_fb(tobuf,x1,y1,x2,y2,x3,y3,color);
  else
  {
    double height = y3 - y1;
    double sl = (x3-x1)/height;
    t1x1 = x1; t1y1 = y1; t1x2 = x2; t1y2 = y2; t1x3 = int(x1 + sl*(y2-y1)); t1y3 = y2;
    draw_triangle_fb(tobuf,t1x1,t1y1,t1x2,t1y2,t1x3,t1y3,color);
    draw_triangle_ft(tobuf,t1x2,t1y2,t1x3,t1y3,x3,y3,color);
  }
}

void Remove_Backfaces_And_Shade(object_ptr the_object)
{
  int vertex_0, vertex_1, vertex_2, curr_poly;
  float dp, intensity;
  vector_3d u, v, normal, sight, view_point;
  view_point.x = 0; view_point.y = 0; view_point.z = 0;
  for(curr_poly = 0; curr_poly < the_object->num_polys; curr_poly++)
  {
    vertex_0 = the_object->polys[curr_poly].vertex_list[0];
    vertex_1 = the_object->polys[curr_poly].vertex_list[1];
    vertex_2 = the_object->polys[curr_poly].vertex_list[2];
    Make_vector_3d((point_3d_ptr)&the_object->vertices_world[vertex_0],
                   (point_3d_ptr)&the_object->vertices_world[vertex_1],
                   (vector_3d_ptr)&u);
    Make_vector_3d((point_3d_ptr)&the_object->vertices_world[vertex_0],
                   (point_3d_ptr)&the_object->vertices_world[vertex_2],
                   (vector_3d_ptr)&v);
    Cross_Product_3d((vector_3d_ptr)&v,
                     (vector_3d_ptr)&u,
                     (vector_3d_ptr)&normal);
    sight.x = view_point.x-the_object->vertices_world[vertex_0].x;
    sight.y = view_point.y-the_object->vertices_world[vertex_0].y;
    sight.z = view_point.z-the_object->vertices_world[vertex_0].z;        
    dp = Dot_Product_3d((vector_3d_ptr)&normal,(vector_3d_ptr)&sight);
    gdp = dp;
    if(dp>0)
    {
      the_object->polys[curr_poly].visible = 1;
      
      if(the_object->polys[curr_poly].shading == 0)
      {
        //dp = Dot_Product_3d((vector_3d_ptr)&normal,
        //                    (vector_3d_ptr)&light_source);
        if(dp>0)
        {
          intensity = 0 + (15*dp/(the_object->polys[curr_poly].normal_length));
          if(intensity > 15) intensity = 15;
          //the_object->polys
        }
      }
    }
    else
      the_object->polys[curr_poly].visible = 0;
    if(key[KEY_N]) the_object->polys[curr_poly].visible = 1;
  }
}

void Draw_Tri_3D_Z(BITMAP *tobuf, int x1, int y1, int z1, int x2, int y2, int z2, int x3, int y3, int z3, int color)
{
  int temp_x, temp_y, temp_z, new_x, new_z;
  if((x1==x2 && x2==x3) || (y1==y2 && y2==y3)) return;
  if(y2<y1)
  {
    temp_x = x2;     temp_y = y2;     temp_z = z2;
    x2     = x1;     y2     = y1;     z2     = z1;
    x1     = temp_x; y1     = temp_y; z1     = temp_z;
  }
  if(y3<y1)
  {
    temp_x = x3;     temp_y = y3;     temp_z = z3;
    x3     = x1;     y3     = y1;     z3     = z1;
    x1     = temp_x; y1     = temp_y; z1     = temp_z;
  }
  if(y3<y2)
  {
    temp_x = x3;     temp_y = y3;     temp_z = z3;
    x3     = x2;     y3     = y2;     z3     = z2;
    x2     = temp_x; y2     = temp_y; z2     = temp_z;
  }
  if( y3<poly_clip_min_y || y1>poly_clip_max_y ||
     (x1<poly_clip_min_x && x2<poly_clip_min_x && x3<poly_clip_min_x) ||
     (x1>poly_clip_max_x && x2>poly_clip_max_x && x3>poly_clip_max_x) )
    return;
  
  if(y2==y3) Draw_B_Tri_3D_Z(tobuf,x1,y1,z1,x2,y2,z2,x3,y3,z3,color);
  else if(y1==y2) Draw_T_Tri_3D_Z(tobuf,x1,y1,z1,x2,y2,z2,x3,y3,z3,color);
  else
  {
    new_x = x1 + (int)((float)(y2-y1)*(float)(x3-x1)/(float)(y3-y1));
    new_z = z1 + (int)((float)(y2-y1)*(float)(z3-z1)/(float)(y3-y1));
    if(y2>=poly_clip_min_y && y1<poly_clip_max_y)
      Draw_B_Tri_3D_Z(tobuf,x1,y1,z1,new_x,y2,new_z,x2,y2,z2,color);
    if(y3>=poly_clip_min_y && y1<poly_clip_max_y)
      Draw_T_Tri_3D_Z(tobuf,x2,y2,z2,new_x,y2,new_z,x3,y3,z3,color);
  }
}

void Draw_T_Tri_3D_Z(BITMAP *tobuf, int x1, int y1, int z1, int x2, int y2, int z2, int x3, int y3, int z3, int color)
{
  int temp_x, temp_y, temp_z;
  if(y2>y1)
  {
    temp_x = x2;
    temp_y = y2;
    temp_z = z2;
    x2     = x1;
    y2     = y1;
    z2     = z1;
    x1     = temp_x;
    y1     = temp_y;
    z1     = temp_z;
  }
  if(y3>y1)
  {
    temp_x = x3;
    temp_y = y3;
    temp_z = z3;
    x3     = x1;
    y3     = y1;
    z3     = z1;
    x1     = temp_x;
    y1     = temp_y;
    z1     = temp_z;
  }
  if(x3<x2)
  {
    temp_x = x3;
    temp_y = y3;
    temp_z = z3;
    x3     = x2;
    y3     = y2;
    z3     = z2;
    x2     = temp_x;
    y2     = temp_y;
    z2     = temp_z;
  }
  float t_height = y1-y2;
  float zcy1 = (z2-z1)/t_height, zcy2 = (z3-z1)/t_height, zs = z1, ze = z1;
  float xcy1 = (x2-x1)/t_height, xcy2 = (x3-x1)/t_height, xs = x1, xe = x1; 
  float z_mid, zcx;
  for(int Y = y1; Y >= y2; Y--)
  {
    z_mid = zs;
    zcx = (zs-ze)/(xs-xe);
    for(int X = int(xs); X <= xe; X++)
    {
      if(X >= 0 && X <= 639 && Y >= 0 && Y <= 479 && z_mid < z_buffer[X][Y]+Z_BUF_OFFSET)
      {
        putpixel(tobuf,X,Y,color);
        z_buffer[X][Y] = z_mid-Z_BUF_OFFSET;
      }
      z_mid += zcx;
    }
    xs += xcy1; xe += xcy2;
    zs += zcy1; ze += zcy2;
  }
}         
void Draw_B_Tri_3D_Z(BITMAP *tobuf, int x1, int y1, int z1, int x2, int y2, int z2, int x3, int y3, int z3, int color)
{
  int temp_x, temp_y, temp_z;
  if(y2<y1)
  {
    temp_x = x2;
    temp_y = y2;
    temp_z = z2;
    x2     = x1;
    y2     = y1;
    z2     = z1;
    x1     = temp_x;
    y1     = temp_y;
    z1     = temp_z;
  }
  if(y3<y1)
  {
    temp_x = x3;
    temp_y = y3;
    temp_z = z3;
    x3     = x1;
    y3     = y1;
    z3     = z1;
    x1     = temp_x;
    y1     = temp_y;
    z1     = temp_z;
  }
  if(x3<x2)
  {
    temp_x = x3;
    temp_y = y3;
    temp_z = z3;
    x3     = x2;
    y3     = y2;
    z3     = z2;
    x2     = temp_x;
    y2     = temp_y;
    z2     = temp_z;
  }
  float t_height = y2-y1;
  float zcy1 = (z2-z1)/t_height, zcy2 = (z3-z1)/t_height, zs = z1, ze = z1;
  float xcy1 = (x2-x1)/t_height, xcy2 = (x3-x1)/t_height, xs = x1, xe = x1; 
  float z_mid, zcx;
  for(int Y = y1; Y <= y2; Y++)
  {
    z_mid = zs;
    zcx = (zs-ze)/(xs-xe);
    for(int X = int(xs); X <= xe; X++)
    {
      if(X >= 0 && X <= 639 && Y >= 0 && Y <= 479 && z_mid < z_buffer[X][Y]+Z_BUF_OFFSET)
      {
        putpixel(tobuf,X,Y,color);
        z_buffer[X][Y] = z_mid-Z_BUF_OFFSET;
      }
      z_mid += zcx;
    }
    xs += xcy1; xe += xcy2;
    zs += zcy1; ze += zcy2;
  }
}

int X3D(float x, float z)
{
  return int(320 + x * VD / z);
}

int Y3D(float y, float z)
{
  return int(240 - y * VD / z);
}

void set_object_points(object &obj)
{
  for(int a = 0; a < obj.num_vertices; a++)
  {
    obj.vertices_world[a].x = 
              obj.vertices_local[a].x+obj.world_pos.x;
    obj.vertices_world[a].y = 
              obj.vertices_local[a].y+obj.world_pos.y;
    obj.vertices_world[a].z = 
              obj.vertices_local[a].z+obj.world_pos.z;
    obj.vertices_camera[a].x = 
              obj.vertices_world[a].x;
    obj.vertices_camera[a].y = 
              obj.vertices_world[a].y;
    obj.vertices_camera[a].z = 
              obj.vertices_world[a].z;
  }
}

void update_timeinfo() {
  static double last_time, current_time=0, last_fps_time=0;
  static int frames=0;
  last_time = current_time;
  current_time = elapsed_time;
  dt = current_time - last_time;
  frames++;
  if (current_time - last_fps_time > 0.5) {
    fps = frames / (current_time - last_fps_time);
    last_fps_time = current_time;
    frames = 0;
  }
}
