#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

bool comments = false;
bool finishingMove = false;
bool waitOnStart = false;
bool imperial = false;
int turns = -1;
int finishingTurns = -1;
float wireGauge = -1;
float shaftLength = -1;
float turnsPerDir = -1;
float feedrate = -1;
float startOffset = -1;
char *name = NULL;
char *path = "gcode_gen.conf";

int parameters;
float passes;


void init(){

  char buffer[65];

  FILE *fp;
  fp = fopen (path,"r");
  if (fp!=NULL){
     while(fgets(&buffer[0], 65, fp) != NULL){
        if ((&buffer[0] != "#")&&(&buffer[0] != " ")){
          if((strncmp(&buffer[0], "wireGauge", 9) == 0) && (wireGauge == -1)){
            wireGauge =  atof(&buffer[9]);
          }else if((strncmp(&buffer[0], "shaftLength", 11) == 0) && (shaftLength == -1)){
            shaftLength = atof(&buffer[11]);
          }else if((strncmp(&buffer[0], "turns", 5) == 0) && (turns == -1)){
            turns = atof(&buffer[5]);
          }else if((strncmp(&buffer[0], "feedrate", 8) == 0) && (feedrate == -1)){
            feedrate = atof(&buffer[8]);
          }else if((strncmp(&buffer[0], "startOffset", 11) == 0) && (startOffset == -1)){
            startOffset = atoi(&buffer[11]);
          }else if((strncmp(&buffer[0], "comments", 8) == 0)&&(comments == false)){
            comments = (atoi(&buffer[8])>0);
          }else if((strncmp(&buffer[0], "finishingMove", 13) == 0)&&(finishingMove == false)){
            finishingMove = (atof(&buffer[13])>0);
          }else if((strncmp(&buffer[0], "finishingTurns", 14) == 0)&&(finishingTurns == false)){
            finishingTurns = (atoi(&buffer[14]));
          }else if((strncmp(&buffer[0], "imperial", 8) == 0)&&(imperial == false)){
            imperial = (atoi(&buffer[8])>0);
          }else if((strncmp(&buffer[0], "waitOnStart", 11) == 0)&&(waitOnStart == false)){
            waitOnStart = (atoi(&buffer[11])>0);
          }
        }
       }
    fclose (fp);
  }


    turnsPerDir = (wireGauge != 0) ? shaftLength/wireGauge : 0;
    passes = (turnsPerDir != 0) ? turns/turnsPerDir : 0;
}

void gen(FILE * out){
  int x = 0;
  bool dir = true;
  fprintf(out,("%s G91 ;\nM17 ;\nG0 A%.2f F%.2f ;\n%s"),(imperial) ? "G20" : "G21" ,startOffset, feedrate, (waitOnStart) ? "M0 ;\n" : "");

  if((int)passes > 0){
    fprintf(out, ((comments) ? "G0 %s%.2f W%.4f F%.2f ; Begin coiling cycles\n" : "G0 %s%.2f W%.4f F%.2f ;\n"), (dir) ? "A" : "A-", shaftLength, turnsPerDir, feedrate);
    dir = false;
    x++;
  }
  while (x < (int)passes){
      fprintf(out, "G0 %s%.2f W%.4f ;\n", (dir) ? "A" : "A-", shaftLength, turnsPerDir);
       dir = (dir) ? false : true;
       x++;
  }
  if (turns - (((int)passes)*turnsPerDir) > 0){
    if ((int)passes == 0){
      fprintf(out, ((comments) ? "G0 %s%.2f W%.4f ; Begin coiling cycle\n" : "G0 %s%.2f W%.4f ;\n"), (dir) ? "A" : "A-", (passes-(int)passes)*shaftLength, (passes-(int)passes)*turnsPerDir);
    } else {
      fprintf(out, "G0 %s%.2f W%.4f ;\n", (dir) ? "A" : "A-", (passes-(int)passes)*shaftLength, (passes-(int)passes)*turnsPerDir);
  }
  }
  if (finishingMove) fprintf(out, "M0 ;\nG0 W%d ;\n", finishingTurns);
  fprintf(out, "M30 ;");


}
int main(int argc, char *argv[]) {

  while ((parameters = getopt(argc, argv, "o:t:f:g:l:p:q:cmiw")) != -1){
    switch (parameters) {
      case 'o':
        name = optarg;
        break;
      case 'c':
        comments = true;
        break;
      case 't':
        turns = atoi(optarg);
        break;
      case 'f':
        feedrate = atof(optarg);
        break;
      case 'g':
        wireGauge = atof(optarg);
        break;
      case 'l':
        shaftLength = atof(optarg);
        break;
      case 'p':
        startOffset = atof(optarg);
        break;
      case 'm':
        finishingMove = true;
        if (optarg != NULL){
          finishingTurns = atoi(optarg);
        }
        break;
      case 'i':
        imperial = true;
        break;
      case 'w':
        waitOnStart = true;
        break;
      case 'q':
        path = optarg;
        break;
    }
  }

  init();

  FILE * f = fopen( (name != NULL) ? name : "output.gcode", "w" );
  if (f != NULL){
    gen(f);
    fclose(f);
  }

  return 0;
}
