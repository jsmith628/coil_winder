# include "queue.h"

#define MODIFIERLENGTH 5

// Gx A[f] B[f] S[f]

struct modifier {
  char c;
  float f;
};

struct command {
  char type[2];
  struct modifier modifiers[MODIFIERLENGTH];

};


struct command parse(String g){

  struct command c;
  for (int x = 0; x < MODIFIERLENGTH; x++){
      c.modifiers[x] = {0,-1};
  }
  g.trim();

  // Evaluate command

  if(g.charAt(0) == 'G' || g.charAt(0) == 'M'){
    c.type[0] = g.charAt(0);
    int x = 1;
    int y = 0;
    int index = 0;
    while (x<g.length()&&isDigit(g.charAt(x))){
      x++;
    }
    c.type[1] = (byte)g.substring(1, x).toInt();

    // Evaluate parameters

    while (x < g.length()){
      if (isAlpha(g.charAt(x))){
        struct modifier m;
        m.c = (char)g.charAt(x);
        y = ++x;
        while ((isDigit(g.charAt(y))||g.charAt(y)=='.'||g.charAt(y)=='-')&& y<g.length()){
          y++;
        }
        m.f = g.substring(x, y).toFloat();
        c.modifiers[index++] = m;
        // for (int z = 0; z < MODIFIERLENGTH; z++){
        //   Serial.print(c.modifiers[z].c);
        //   Serial.print(c.modifiers[z].f);
        //   Serial.print(" ");
        // }
        //Serial.println(c.modifiers[0].c);
        x = y;
      }else{
        x++;
      }
    }

    //Serial.println(c.modifiers[0].c);
    return c;
  } else {
      c.type[0] = 0;
      c.type[1] = 0;
      return c;
  }
}

bool interpret_gcode(struct command){
// ToDo


return true;
}



struct command com;
Queue<struct command, 3> q = Queue<struct command, 3>();


bool read_command(){

        String code = Serial.readString();
  //      Serial.println(code);
        com = parse(code);

  //        Serial.println(com.modifiers[0].c);
        // Serial.println("Printing");
        // Serial.print((char)com.type[0]);
        // Serial.print((byte)com.type[1]);
        // Serial.print("");
        // for (int x = 0; x < MODIFIERLENGTH; x++){
        //   Serial.print(com.modifiers[x].c);
        //   Serial.print(com.modifiers[x].f);
        //   Serial.print(" ");
        // }
        // Serial.print("\n");
        // Serial.println("End");


}

void parser_setup(){

  Serial.begin(115200);

}



// void setup(){
//   parser_setup();
// }
// void loop(){
//
//     if (Serial.available()){
//         read_command();
//     }
// }
