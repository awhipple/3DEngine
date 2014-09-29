#include "aaron.cpp"

double VD = 250;

class vertex_class;

class vertexlink_class
{
   public:

   vertex_class *point;
   vertexlink_class *prev, *next;

   vertexlink_class(){point = NULL; prev = NULL; next = NULL;}
};

class vertex_class
{
   public:

   double x, y, z;
   vertexlink_class *f_link;
   vertex_class *prev, *next;

   vertex_class(){x = 0; y = 0; z = 0; f_link = NULL; prev = NULL; next = NULL;}
};

class object_class
{
   public:

   double x, y, z;
   vertex_class *f_vertex;
   object_class *prev, *next;

   object_class(){x = 0; y = 0; z = 0; f_vertex = NULL; prev = NULL; next = NULL;}
};

object_class *f_object = NULL;

void rotate_obj_x(object_class *obj, double rad);
void rotate_obj_y(object_class *obj, double rad);
void rotate_obj_z(object_class *obj, double rad);

int main()
{
   allegro_init();
   install_keyboard();
   set_up_timer();
   install_sound(DIGI_AUTODETECT,MIDI_AUTODETECT,0);

   set_color_depth(16);
   set_gfx_mode(GFX_AUTODETECT,800,600,0,0);

   DATAFILE *data = load_datafile("data.dat");

   BITMAP *buffer = create_bitmap(800,600);
   MIDI *mid = (MIDI *)data[1].dat;//load_midi("darkman.mid");
   play_midi(mid,1);

   object_class *n_object = new object_class;
   if(f_object) f_object->prev = n_object;
   n_object->next = f_object;
   f_object = n_object;

   double cx = 0, cy = 0, cz = 0, cxv = 0, cyv = 0, cs = 100, csv = -40;
   int new_tvect = 10;
   vertex_class *last_v1[8], *last_v2[8];
   bool first_rep = 1;
   for(int a = 0; a < 50; a++)
   {
      int p = 0;
      for(int c = 0; c < 8; c++) last_v2[c] = last_v1[c];
      for(double crad = 0; crad < 6.28; crad += 0.785)
      {
         vertex_class *n_vert = new vertex_class;
         if(f_object->f_vertex) f_object->f_vertex->prev = n_vert;
         n_vert->next = f_object->f_vertex;
         f_object->f_vertex = n_vert;

         n_vert->x = cx + cos(crad) * cs;
         n_vert->y = cy + sin(crad) * cs;
         n_vert->z = cz;

         last_v1[p] = n_vert;
         if(p)
         {
            vertexlink_class *n_vlist = new vertexlink_class;
            if(n_vert->f_link) n_vert->f_link->prev = n_vlist;
            n_vlist->next = n_vert->f_link;
            n_vert->f_link = n_vlist;
            n_vlist->point = last_v1[p-1];
         }
         if(!first_rep)
         {
            vertexlink_class *n_vlist = new vertexlink_class;
            if(n_vert->f_link) n_vert->f_link->prev = n_vlist;
            n_vlist->next = n_vert->f_link;
            n_vert->f_link = n_vlist;
            n_vlist->point = last_v2[p];
         }
         p++;
      }
      vertexlink_class *n_vlist = new vertexlink_class;
      if(last_v1[0]->f_link) last_v1[0]->f_link->prev = n_vlist;
      n_vlist->next = last_v1[0]->f_link;
      last_v1[0]->f_link = n_vlist;
      n_vlist->point = last_v1[7];

      cx += cxv;
      cy += cyv;
      cz += 50;
      cs += csv;
      if(cs < 20) cs = 20;
      new_tvect--;
      if(!new_tvect){cxv = drnd(-50,50); cyv = drnd(-50,50); csv = drnd(-20,20); new_tvect = rnd(1,10);}

      if(first_rep) cs = 500;
      first_rep = 0;
   }

   f_object->z += 1000;

   while(!key[KEY_ESC])
   {
      clear(buffer);

      object_class *cur_object = f_object;
      while(cur_object)
      {
         update_timeinfo();
         if(!key[KEY_SPACE])
         {
            //rotate_obj_x(cur_object,.01);
            //rotate_obj_y(cur_object,.02);
            rotate_obj_z(cur_object,.03 * ((cur_object->z+2500)/3000));
         }

         if(key[KEY_UP]) cur_object->y -= 300 * dt;
         if(key[KEY_DOWN]) cur_object->y += 300 * dt;

         if(key[KEY_LEFT]) cur_object->x += 300 * dt;
         if(key[KEY_RIGHT]) cur_object->x -= 300 * dt;

         /*if(key[KEY_A])*/ cur_object->z -= 100 * dt;
         /*if(key[KEY_Z]) cur_object->z += 100 * dt;*/

         vertex_class *cur_vertex = cur_object->f_vertex;
         while(cur_vertex)
         {
            if(key[KEY_R]) cur_vertex->y += rnd(-1,1);
            if(key[KEY_T])
            {
               if(cur_vertex->y > 0) cur_vertex->y -= 1;
               if(cur_vertex->y < 0) cur_vertex->y += 1;
            }

            vertexlink_class *cur_link = cur_vertex->f_link;
            while(cur_link)
            {
               int vx1 = int(cur_object->x + cur_vertex->x),
                   vy1 = int(cur_object->y + cur_vertex->y),
                   vz1 = int(cur_object->z + cur_vertex->z),
                   vx2 = int(cur_object->x + cur_link->point->x),
                   vy2 = int(cur_object->y + cur_link->point->y),
                   vz2 = int(cur_object->z + cur_link->point->z);
               int x1 = int(400 + vx1 * VD / vz1),
                   y1 = int(300 - vy1 * VD / vz1),
                   x2 = int(400 + vx2 * VD / vz2),
                   y2 = int(300 - vy2 * VD / vz2);

               bool draw_line = 1;
               if(vz1 < 0 || vz2 < 0) draw_line = 0;

               int col = 255 - int(vz1 * .3);// - cur_object->z;
               if(col < 0) col = 0; if(col > 255) col = 255;

               if(draw_line) line(buffer,x1,y1,x2,y2,makecol(col,col,col));

               cur_link = cur_link->next;
            }

            cur_vertex = cur_vertex->next;
         }

         cur_object = cur_object->next;
      }

      blit(buffer,screen,0,0,0,0,800,600);
   }

   allegro_exit();
   return 0;
}
END_OF_MAIN();

void rotate_obj_x(object_class *obj, double rad)
{
   vertex_class *cur_v = obj->f_vertex;
   while(cur_v)
   {
      double ny = cos(rad) * cur_v->y - sin(rad) * cur_v->z,
             nz = sin(rad) * cur_v->y + cos(rad) * cur_v->z;
      cur_v->y = ny; cur_v->z = nz;

      cur_v = cur_v->next;
   }
}

void rotate_obj_y(object_class *obj, double rad)
{
   vertex_class *cur_v = obj->f_vertex;
   while(cur_v)
   {
      double nx = cos(rad) * cur_v->x - sin(rad) * cur_v->z,
             nz = sin(rad) * cur_v->x + cos(rad) * cur_v->z;
      cur_v->x = nx; cur_v->z = nz;

      cur_v = cur_v->next;
   }
}

void rotate_obj_z(object_class *obj, double rad)
{
   vertex_class *cur_v = obj->f_vertex;
   while(cur_v)
   {
      double nx = cos(rad) * cur_v->x - sin(rad) * cur_v->y,
             ny = sin(rad) * cur_v->x + cos(rad) * cur_v->y;
      cur_v->x = nx; cur_v->y = ny;

      cur_v = cur_v->next;
   }
}


