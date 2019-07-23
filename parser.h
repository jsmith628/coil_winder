# include "gcodes.h"

#define MODIFIERLENGTH 6

// Gx A[f] B[f] S[f]

struct modifier {
  char c;
  float f;
};

struct command {
  char type;
  unsigned int number;
  struct modifier modifiers[MODIFIERLENGTH];

};



void display_warning(String type){
  Serial.print("Warning: ");
  Serial.println(type);
}

void display_warning(String type, String details){
  Serial.print("Warning: ");
  Serial.println(type);
  Serial.println(details);
}

void display_warning(String type, unsigned int specificType, String details){
  Serial.print("Warning: ");
  Serial.print(type);
  Serial.print(specificType);
  Serial.println(details);
}


void interpret_gcode(struct command c){
  //A, B, S, F, D/P, R
  if(c.type == 'G'){
    switch (c.number) {
      default:
        display_warning("G",c.number," is not a valid G command.");
        break;
      case 0:
        g0(c.modifiers[0].f, c.modifiers[1].f, c.modifiers[2].f, c.modifiers[3].f);
        break;
      case 1:
        g1(c.modifiers[0].f, c.modifiers[1].f, c.modifiers[2].f, c.modifiers[3].f);
        break;
      case 4:
        g4(c.modifiers[6].f, c.modifiers[2].f);
        break;
      case 20:
        g20();
        break;
      case 21:
        g21();
        break;
      case 28:
        g28(c.modifiers[0].f, c.modifiers[1].f, c.modifiers[2].f);
        break;
      case 31:
        g31((c.modifiers[0].f!=0),(c.modifiers[1].f!=0),c.modifiers[2].f);
        break;
      case 32:
        g32(c.modifiers[0].f, c.modifiers[1].f, c.modifiers[2].f, c.modifiers[3].f);
        break;
      case 50:
        g50(c.modifiers[2].f);
        break;
      case 52:
        g52(c.modifiers[0].f, c.modifiers[1].f);
        break;
      case 76:
        g76(c.modifiers[0].f,c.modifiers[1].f,c.modifiers[2].f,c.modifiers[3].f,(int)c.modifiers[0].f,(c.modifiers[5].f!=0));
        break;
      case 90:
        g90();
        break;
      case 91:
        g91();
        break;
      case 94:
        g94();
        break;
      case 95:
        g95();
        break;
    }


  }else if(c.type == 'M'){
    switch (c.number) {
      default:
        display_warning("M",c.number," is not a valid G command.");
        break;
      case 0:
        m0();
        break;
      case 3:
        m3(c.modifiers[2].f);
        break;
      case 4:
        m4(c.modifiers[2].f);
        break;
      case 5:
        m5();
        break;
      case 17:
        m17();
        break;
      case 18:
        m18();
        break;
      case 30:
        m30();
        break;
      case 98:
        m98();
        break;
      case 99:
        m99();
        break;
      case 112:
        m112();
        break;
      case 114:
          if((c.modifiers[0].c != 0)||(c.modifiers[1].c != 0)){
            m114((c.modifiers[0].f != 0), (c.modifiers[0].f != 0));
          }else{
            m114();
          }
        break;
      case 125:
        m125(c.modifiers[0].f, c.modifiers[1].f);
        break;
      case 120:
        m120();
        break;
      case 121:
        m121();
        break;
      case 203:
        m203(c.modifiers[3].f);
        break;
      case 500:
        m500();
        break;
      case 501:
        m501();
        break;
      case 503:
        m503();
        break;
    }


  }else{
    display_warning("Invalid GCODE", "Commands must start with G or M.");
  }

}


struct command parse(String g){

  struct command c;
  for (int x = 0; x < MODIFIERLENGTH; x++){
      c.modifiers[x] = {0,NAN};
  }
  g.trim();

  // Evaluate command

  if(g.charAt(0) == 'G' || g.charAt(0) == 'M'){
    c.type = g.charAt(0);
    int x = 1;
    int y = 0;
    while (x<g.length()&&isDigit(g.charAt(x))){
      x++;
    }
    c.number = (byte)g.substring(1, x).toInt();

    // Evaluate parameters

    //A, B, S, F, D/P, R

    while (x < g.length()){
      if (isAlpha(g.charAt(x))){
        if(g.charAt(x) == 'G' || g.charAt(x) == 'M'){
          interpret_gcode(parse(g.substring(x, g.length())));
          break;
        }
          struct modifier m;
          m.c = (char)g.charAt(x);
          y = ++x;
          while ((isDigit(g.charAt(y))||g.charAt(y)=='.'||g.charAt(y)=='-')&& y<g.length()){
            y++;
          }
          m.f = g.substring(x, y).toFloat();
          switch (m.c) {
            case 'A':
              c.modifiers[0] = m;
              break;
            case 'B':
              c.modifiers[1] = m;
              break;
            case 'S':
              c.modifiers[2] = m;
              break;
            case 'F':
              c.modifiers[3] = m;
              break;
            case 'D':
              c.modifiers[4] = m;
              break;
            case 'R':
              c.modifiers[5] = m;
              break;
            case 'P':
              c.modifiers[4] = m;
              break;

          }
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
      c.type = 0;
      c.number = 0;
      return c;
  }
}

struct command com;


bool read_command(){

        String input = Serial.readString();
  //      Serial.println(code);
        com = parse(input);
        interpret_gcode(com);

  //        Serial.println(com.modifiers[0].c);
        // Serial.println("Printing");
        // Serial.print((char)com.type);
        // Serial.print((byte)com.number);
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
