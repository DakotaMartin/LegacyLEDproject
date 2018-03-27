
#include <Adafruit_NeoPixel.h>

const int dataPin = 4;
const int pixelCount = 101;

int outputR[101];
int outputG[101];
int outputB[101];

int leftRowR[5] , leftRowG[5] , leftRowB[5];
int rightRowR[5] , rightRowG[5] , rightRowB[5];

Adafruit_NeoPixel ledStrip = Adafruit_NeoPixel(pixelCount, dataPin);

int index;

int textBuffer[1023];
int curLetter;
int curCol;
int curRow;

int delayAmount = 150;
int nextShift;
int x , y , var;

void setup() {
  ledStrip.begin();
  Serial.begin(9600);
  var = 65;
}

int var45;

void loop() {
  /*
  outputR[20] = random(50);
  outputG[20] = random(50);
  outputB[20] = random(50);
  
  outputR[40] = random(50);
  outputG[40] = random(50);
  outputB[40] = random(50);

  outputR[60] = random(50);
  outputG[60] = random(50);
  outputB[60] = random(50);

  outputR[80] = random(50);
  outputG[80] = random(50);
  outputB[80] = random(50);

  outputR[100] = random(50);
  outputG[100] = random(50);
  outputB[100] = random(50);
  
  shiftLeft();
  */
  if (var45 < 4) {
    shiftLeft();
    var45++;
  } else {
    drawLetter(var , random(100) , random(100) , random(100));
    var++;
    var45 = 0;
  }
  if (var == 91) {
    var = 65;
  }
   /*
  if(Serial.available() > 0){
    var = Serial.read();
  } else {
    var = 0;
  }
  if (var45 < 4) {
    shiftLeft();
    var45++;
  } else {
    drawLetter(var , 0 , 0 , 20);
    var45 = 0;
  }
  */
  drawOutput();
  delay(100);
}

boolean a1 , a2 , a3 , a4 , a5, b1 , b2 , b3 , b4 , b5 , c1 , c2 , c3 , c4 , c5;

void drawLetter(int letter , int inputR , int inputG , int inputB) {
  a1 = false;
  a2 = false;
  a3 = false;
  a4 = false;
  a5 = false;
  b1 = false;
  b2 = false;
  b3 = false;
  b4 = false;
  b5 = false;
  c1 = false;
  c2 = false;
  c3 = false;
  c4 = false;
  c5 = false;
  if (letter == 'A') {
    b1 = true;
    a2 = true;
    c2 = true;
    a3 = true;
    c3 = true;
    a4 = true;
    c4 = true;
    a5 = true;
    c5 = true;
    b3 = true;
  } else if (letter == 'B') {
    a1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    a5 = true;
    b1 = true;
    b3 = true;
    b5 = true;
    c2 = true;
    c4 = true;
  } else if (letter == 'C') {
    b1 = true;
    c1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    b5 = true;
    c5 = true;
  } else if (letter == 'D') {
    a1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    a5 = true;
    b1 = true;
    b5 = true;
    c2 = true;
    c3 = true;
    c4 = true;
  } else if (letter == 'E') {
    a1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    a5 = true;
    b1 = true;
    c1 = true;
    b3 = true;
    c3 = true;
    b5 = true;
    c5 = true;
  } else if (letter == 'F') {
    a1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    a5 = true;
    b1 = true;
    c1 = true;
    b3 = true;
  } else if (letter == 'G') {
    a1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    a5 = true;
    b1 = true;
    c1 = true;
    b5 = true;
    c5 = true;
    c4 = true;
  } else if (letter == 'H') {
    a1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    a5 = true;
    c1 = true;
    c2 = true;
    c3 = true;
    c4 = true;
    c5 = true;
    b3 = true;
  } else if (letter == 'I') {
    a1 = true;
    a5 = true;
    b1 = true;
    b2 = true;
    b3 = true;
    b4 = true;
    b5 = true;
    c1 = true;
    c5 = true;
  } else if (letter == 'J') {
    a1 = true;
    b1 = true;
    c1 = true;
    b2 = true;
    b3 = true;
    b4 = true;
    a5 = true;
  } else if (letter == 'K') {
    a1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    a5 = true;
    b3 = true;
    c1 = true;
    c2 = true;
    c4 = true;
    c5 = true;
  } else if (letter == 'L') {
    a1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    a5 = true;
    b5 = true;
    c5 = true;
  } else if (letter == 'M') {
    a1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    a5 = true;
    c1 = true;
    c2 = true;
    c3 = true;
    c4 = true;
    c5 = true;
    b2 = true;
    b3 = true;
  } else if (letter == 'N') {
    a1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    a5 = true;
    c1 = true;
    c2 = true;
    c3 = true;
    c4 = true;
    c5 = true;
    b2 = true;
    b3 = true;
    b4 = true;
  } else if (letter == 'O') {
    b1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    b5 = true;
    c2 = true;
    c3 = true;
    c4 = true;
  } else if (letter == 'P') {
    a1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    a5 = true;
    b1 = true;
    b3 = true;
    c2 = true;
  } else if (letter == 'Q') {
    b1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    c2 = true;
    c3 = true;
    c4 = true;
    b5 = true;
    c5 = true;
  } else if (letter == 'R') {
    a1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    a5 = true;
    b1 = true;
    b3 = true;
    c2 = true;
    c4 = true;
    c5 = true;
  } else if (letter == 'S') {
    b1 = true;
    c1 = true;
    a2 = true;
    b3 = true;
    c4 = true;
    a5 = true;
    b5 = true;
  } else if (letter == 'T') {
    a1 = true;
    b1 = true;
    c1 = true;
    b2 = true;
    b3 = true;
    b4 = true;
    b5 = true;
  } else if (letter == 'U') {
    a1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    c1 = true;
    c2 = true;
    c3 = true;
    c4 = true;
    b5 = true;
    a5 = true;
    c5 = true;
  } else if (letter == 'V') {
    a1 = true;
    a2 = true;
    a3 = true;
    c1 = true;
    c2 = true;
    c3 = true;
    b5 = true;
    a4 = true;
    c4 = true;
  } else if (letter == 'W') {
    a1 = true;
    a2 = true;
    a3 = true;
    a4 = true;
    a5 = true;
    b4 = true;
    c1 = true;
    c2 = true;
    c3 = true;
    c4 = true;
    c5 = true;
    b3 = true;
  } else if (letter == 'X') {
    a1 = true;
    a2 = true;
    a4 = true;
    a5 = true;
    c1 = true;
    c2 = true;
    c4 = true;
    c5 = true;
    b3 = true;
  } else if (letter == 'Y') {
    a1 = true;
    a2 = true;
    c1 = true;
    c2 = true;
    b3 = true;
    b4 = true;
    b5 = true;
  } else if (letter == 'Z') {
    a1 = true;
    b1 = true;
    c1 = true;
    c2 = true;
    b3 = true;
    a4 = true;
    a5 = true;
    b5 = true;
    c5 = true;
  }
  if (a1) {
    outputR[18] = inputR;
    outputG[18] = inputG;
    outputB[18] = inputB;
  }
  if (a2) {
    outputR[38] = inputR;
    outputG[38] = inputG;
    outputB[38] = inputB;
  }
  if (a3) {
    outputR[58] = inputR;
    outputG[58] = inputG;
    outputB[58] = inputB;
  }
  if (a4) {
    outputR[78] = inputR;
    outputG[78] = inputG;
    outputB[78] = inputB;
  }
  if (a5) {
    outputR[98] = inputR;
    outputG[98] = inputG;
    outputB[98] = inputB;
  }
  if (b1) {
    outputR[19] = inputR;
    outputG[19] = inputG;
    outputB[19] = inputB;
  }
  if (b2) {
    outputR[39] = inputR;
    outputG[39] = inputG;
    outputB[39] = inputB;
  }
  if (b3) {
    outputR[59] = inputR;
    outputG[59] = inputG;
    outputB[59] = inputB;
  }
  if (b4) {
    outputR[79] = inputR;
    outputG[79] = inputG;
    outputB[79] = inputB;
  }
  if (b5) {
    outputR[99] = inputR;
    outputG[99] = inputG;
    outputB[99] = inputB;
  }
  if (c1) {
    outputR[20] = inputR;
    outputG[20] = inputG;
    outputB[20] = inputB;
  }
  if (c2) {
    outputR[40] = inputR;
    outputG[40] = inputG;
    outputB[40] = inputB;
  }
  if (c3) {
    outputR[60] = inputR;
    outputG[60] = inputG;
    outputB[60] = inputB;
  }
  if (c4) {
    outputR[80] = inputR;
    outputG[80] = inputG;
    outputB[80] = inputB;
  }
  if (c5) {
    outputR[100] = inputR;
    outputG[100] = inputG;
    outputB[100] = inputB;
  }
}

int ani2var;

void ani3() {
  index = 0;
  ani2var = 0;
  while (index < 20) {
    index++;
    ani2var++;
    outputR[index] = ani2var * 10 + 50;
  }
  ani2var = 0;
  while (index < 40) {
    index++;
    ani2var++;
    outputG[index] = ani2var * 10 + 50;
  }
  ani2var = 0;
  while (index < 60) {
    index++;
    ani2var++;
    outputB[index] = ani2var * 10 + 50;
  }
  ani2var = 0;
  while (index < 80) {
    index++;
    ani2var++;
    outputR[index] = ani2var * 10 + 50;
    outputB[index] = ani2var * 10 + 50;
  }
  ani2var = 0;
  while (index < 100) {
    index++;
    ani2var++;
    outputB[index] = ani2var * 10 + 50;
    outputG[index] = ani2var * 10 + 50;
  }
}

void ani2() {
  if (ani2var == 0 || ani2var > 100) {//If we're just starting out, or if we've just finnished: go back to the first pixel.
    ani2var = 1;
  }
  outputR[ani2var] = round(random(30));//make the pixel we're on a random color.
  outputG[ani2var] = round(random(30));
  outputB[ani2var] = round(random(30));
  ani2var++;//go to the next pixel.
}
void shiftLeft() {
  index = 1;
  while (index < 100) {
    outputR[index] = outputR[index + 1];
    outputG[index] = outputG[index + 1];
    outputB[index] = outputB[index + 1];
    index++;
  }
  outputR[20] = rightRowR[1];
  outputR[40] = rightRowR[2];
  outputR[60] = rightRowR[3];
  outputR[80] = rightRowR[4];
  outputR[100] = rightRowR[5];
  outputG[20] = rightRowG[1];
  outputG[40] = rightRowG[2];
  outputG[60] = rightRowG[3];
  outputG[80] = rightRowG[4];
  outputG[100] = rightRowG[5];
  outputB[20] = rightRowB[1];
  outputB[40] = rightRowB[2];
  outputB[60] = rightRowB[3];
  outputB[80] = rightRowB[4];
  outputB[100] = rightRowB[5];
}

void shiftRight() {
  outputR[20] = outputR[19];
  outputR[19] = outputR[18];
  outputR[18] = outputR[17];
  outputR[17] = outputR[16];
  outputR[16] = outputR[15];
  outputR[15] = outputR[14];
  outputR[14] = outputR[13];
  outputR[13] = outputR[12];
  outputR[12] = outputR[11];
  outputR[11] = outputR[10];
  outputR[10] = outputR[9];
  outputR[9] = outputR[8];
  outputR[8] = outputR[7];
  outputR[7] = outputR[6];
  outputR[6] = outputR[5];
  outputR[5] = outputR[4];
  outputR[4] = outputR[3];
  outputR[3] = outputR[2];
  outputR[2] = outputR[1];
  outputR[1] = leftRowR[1];
  outputG[20] = outputG[19];
  outputG[19] = outputG[18];
  outputG[18] = outputG[17];
  outputG[17] = outputG[16];
  outputG[16] = outputG[15];
  outputG[15] = outputG[14];
  outputG[14] = outputG[13];
  outputG[13] = outputG[12];
  outputG[12] = outputG[11];
  outputG[11] = outputG[10];
  outputG[10] = outputG[9];
  outputG[9] = outputG[8];
  outputG[8] = outputG[7];
  outputG[7] = outputG[6];
  outputG[6] = outputG[5];
  outputG[5] = outputG[4];
  outputG[4] = outputG[3];
  outputG[3] = outputG[2];
  outputG[2] = outputG[1];
  outputG[1] = leftRowG[1];
  outputB[20] = outputB[19];
  outputB[19] = outputB[18];
  outputB[18] = outputB[17];
  outputB[17] = outputB[16];
  outputB[16] = outputB[15];
  outputB[15] = outputB[14];
  outputB[14] = outputB[13];
  outputB[13] = outputB[12];
  outputB[12] = outputB[11];
  outputB[11] = outputB[10];
  outputB[10] = outputB[9];
  outputB[9] = outputB[8];
  outputB[8] = outputB[7];
  outputB[7] = outputB[6];
  outputB[6] = outputB[5];
  outputB[5] = outputB[4];
  outputB[4] = outputB[3];
  outputB[3] = outputB[2];
  outputB[2] = outputB[1];
  outputB[1] = leftRowB[1];
  //##############################################################3
  outputR[40] = outputR[39];
  outputR[39] = outputR[38];
  outputR[38] = outputR[37];
  outputR[37] = outputR[36];
  outputR[36] = outputR[35];
  outputR[35] = outputR[34];
  outputR[34] = outputR[33];
  outputR[33] = outputR[32];
  outputR[32] = outputR[31];
  outputR[31] = outputR[30];
  outputR[30] = outputR[29];
  outputR[29] = outputR[28];
  outputR[28] = outputR[27];
  outputR[27] = outputR[26];
  outputR[26] = outputR[25];
  outputR[25] = outputR[24];
  outputR[24] = outputR[23];
  outputR[23] = outputR[22];
  outputR[22] = outputR[21];
  outputR[21] = leftRowR[2];
  outputG[40] = outputG[39];
  outputG[39] = outputG[38];
  outputG[38] = outputG[37];
  outputG[37] = outputG[36];
  outputG[36] = outputG[35];
  outputG[35] = outputG[34];
  outputG[34] = outputG[33];
  outputG[33] = outputG[32];
  outputG[32] = outputG[31];
  outputG[31] = outputG[30];
  outputG[30] = outputG[29];
  outputG[29] = outputG[28];
  outputG[28] = outputG[27];
  outputG[27] = outputG[26];
  outputG[26] = outputG[25];
  outputG[25] = outputG[24];
  outputG[24] = outputG[23];
  outputG[23] = outputG[22];
  outputG[22] = outputG[21];
  outputG[21] = leftRowG[2];
  outputB[40] = outputB[39];
  outputB[39] = outputB[38];
  outputB[38] = outputB[37];
  outputB[37] = outputB[36];
  outputB[36] = outputB[35];
  outputB[35] = outputB[34];
  outputB[34] = outputB[33];
  outputB[33] = outputB[32];
  outputB[32] = outputB[31];
  outputB[31] = outputB[30];
  outputB[30] = outputB[29];
  outputB[29] = outputB[28];
  outputB[28] = outputB[27];
  outputB[27] = outputB[26];
  outputB[26] = outputB[25];
  outputB[25] = outputB[24];
  outputB[24] = outputB[23];
  outputB[23] = outputB[22];
  outputB[22] = outputB[21];
  outputB[21] = leftRowB[2];
  //############################################
  outputR[60] = outputR[59];
  outputR[59] = outputR[58];
  outputR[58] = outputR[57];
  outputR[57] = outputR[56];
  outputR[56] = outputR[55];
  outputR[55] = outputR[54];
  outputR[54] = outputR[53];
  outputR[53] = outputR[52];
  outputR[52] = outputR[51];
  outputR[51] = outputR[50];
  outputR[50] = outputR[49];
  outputR[49] = outputR[48];
  outputR[48] = outputR[47];
  outputR[47] = outputR[46];
  outputR[46] = outputR[45];
  outputR[45] = outputR[44];
  outputR[44] = outputR[43];
  outputR[43] = outputR[42];
  outputR[42] = outputR[41];
  outputR[41] = leftRowR[3];
  outputG[60] = outputG[59];
  outputG[59] = outputG[58];
  outputG[58] = outputG[57];
  outputG[57] = outputG[56];
  outputG[56] = outputG[55];
  outputG[55] = outputG[54];
  outputG[54] = outputG[53];
  outputG[53] = outputG[52];
  outputG[52] = outputG[51];
  outputG[51] = outputG[50];
  outputG[50] = outputG[49];
  outputG[49] = outputG[48];
  outputG[48] = outputG[47];
  outputG[47] = outputG[46];
  outputG[46] = outputG[45];
  outputG[45] = outputG[44];
  outputG[44] = outputG[43];
  outputG[43] = outputG[42];
  outputG[42] = outputG[41];
  outputG[41] = leftRowG[3];
  outputB[60] = outputB[59];
  outputB[59] = outputB[58];
  outputB[58] = outputB[57];
  outputB[57] = outputB[56];
  outputB[56] = outputB[55];
  outputB[55] = outputB[54];
  outputB[54] = outputB[53];
  outputB[53] = outputB[52];
  outputB[52] = outputB[51];
  outputB[51] = outputB[50];
  outputB[50] = outputB[49];
  outputB[49] = outputB[48];
  outputB[48] = outputB[47];
  outputB[47] = outputB[46];
  outputB[46] = outputB[45];
  outputB[45] = outputB[44];
  outputB[44] = outputB[43];
  outputB[43] = outputB[42];
  outputB[42] = outputB[41];
  outputB[41] = leftRowB[3];
  //########################################
  outputR[80] = outputR[79];
  outputR[79] = outputR[78];
  outputR[78] = outputR[77];
  outputR[77] = outputR[76];
  outputR[76] = outputR[75];
  outputR[75] = outputR[74];
  outputR[74] = outputR[73];
  outputR[73] = outputR[72];
  outputR[72] = outputR[71];
  outputR[71] = outputR[70];
  outputR[70] = outputR[69];
  outputR[69] = outputR[68];
  outputR[68] = outputR[67];
  outputR[67] = outputR[66];
  outputR[66] = outputR[65];
  outputR[65] = outputR[64];
  outputR[64] = outputR[63];
  outputR[63] = outputR[62];
  outputR[62] = outputR[61];
  outputR[61] = leftRowR[4];
  outputG[80] = outputG[79];
  outputG[79] = outputG[78];
  outputG[78] = outputG[77];
  outputG[77] = outputG[76];
  outputG[76] = outputG[75];
  outputG[75] = outputG[74];
  outputG[74] = outputG[73];
  outputG[73] = outputG[72];
  outputG[72] = outputG[71];
  outputG[71] = outputG[70];
  outputG[70] = outputG[69];
  outputG[69] = outputG[68];
  outputG[68] = outputG[67];
  outputG[67] = outputG[66];
  outputG[66] = outputG[65];
  outputG[65] = outputG[64];
  outputG[64] = outputG[63];
  outputG[63] = outputG[62];
  outputG[62] = outputG[61];
  outputG[61] = leftRowG[4];
  outputB[80] = outputB[79];
  outputB[79] = outputB[78];
  outputB[78] = outputB[77];
  outputB[77] = outputB[76];
  outputB[76] = outputB[75];
  outputB[75] = outputB[74];
  outputB[74] = outputB[73];
  outputB[73] = outputB[72];
  outputB[72] = outputB[71];
  outputB[71] = outputB[70];
  outputB[70] = outputB[69];
  outputB[69] = outputB[68];
  outputB[68] = outputB[67];
  outputB[67] = outputB[66];
  outputB[66] = outputB[65];
  outputB[65] = outputB[64];
  outputB[64] = outputB[63];
  outputB[63] = outputB[62];
  outputB[62] = outputB[61];
  outputB[61] = leftRowB[4];
  //########################################
  outputR[100] = outputR[99];
  outputR[99] = outputR[98];
  outputR[98] = outputR[97];
  outputR[97] = outputR[96];
  outputR[96] = outputR[95];
  outputR[95] = outputR[94];
  outputR[94] = outputR[93];
  outputR[93] = outputR[92];
  outputR[92] = outputR[91];
  outputR[91] = outputR[90];
  outputR[90] = outputR[89];
  outputR[89] = outputR[88];
  outputR[88] = outputR[87];
  outputR[87] = outputR[86];
  outputR[86] = outputR[85];
  outputR[85] = outputR[84];
  outputR[84] = outputR[83];
  outputR[83] = outputR[82];
  outputR[82] = outputR[81];
  outputR[81] = leftRowR[5];
  outputG[100] = outputG[99];
  outputG[99] = outputG[98];
  outputG[98] = outputG[97];
  outputG[97] = outputG[96];
  outputG[96] = outputG[95];
  outputG[95] = outputG[94];
  outputG[94] = outputG[93];
  outputG[93] = outputG[92];
  outputG[92] = outputG[91];
  outputG[91] = outputG[90];
  outputG[90] = outputG[89];
  outputG[89] = outputG[88];
  outputG[88] = outputG[87];
  outputG[87] = outputG[86];
  outputG[86] = outputG[85];
  outputG[85] = outputG[84];
  outputG[84] = outputG[83];
  outputG[83] = outputG[82];
  outputG[82] = outputG[81];
  outputG[81] = leftRowG[5];
  outputB[100] = outputB[99];
  outputB[99] = outputB[98];
  outputB[98] = outputB[97];
  outputB[97] = outputB[96];
  outputB[96] = outputB[95];
  outputB[95] = outputB[94];
  outputB[94] = outputB[93];
  outputB[93] = outputB[92];
  outputB[92] = outputB[91];
  outputB[91] = outputB[90];
  outputB[90] = outputB[89];
  outputB[89] = outputB[88];
  outputB[88] = outputB[87];
  outputB[87] = outputB[86];
  outputB[86] = outputB[85];
  outputB[85] = outputB[84];
  outputB[84] = outputB[83];
  outputB[83] = outputB[82];
  outputB[82] = outputB[81];
  outputB[81] = leftRowB[5];
}

void drawOutput() {
  ledStrip.setPixelColor(0 , outputG[1] , outputR[1] , outputB[1]);
  ledStrip.setPixelColor(1 , outputG[2] , outputR[2] , outputB[2]);
  ledStrip.setPixelColor(2 , outputG[3] , outputR[3] , outputB[3]);
  ledStrip.setPixelColor(3 , outputG[4] , outputR[4] , outputB[4]);
  ledStrip.setPixelColor(4 , outputG[5] , outputR[5] , outputB[5]);
  ledStrip.setPixelColor(5 , outputG[6] , outputR[6] , outputB[6]);
  ledStrip.setPixelColor(6 , outputG[7] , outputR[7] , outputB[7]);
  ledStrip.setPixelColor(7 , outputG[8] , outputR[8] , outputB[8]);
  ledStrip.setPixelColor(8 , outputG[9] , outputR[9] , outputB[9]);
  ledStrip.setPixelColor(9 , outputG[10] , outputR[10] , outputB[10]);
  ledStrip.setPixelColor(10 , outputG[11] , outputR[11] , outputB[11]);
  ledStrip.setPixelColor(11 , outputG[12] , outputR[12] , outputB[12]);
  ledStrip.setPixelColor(12 , outputG[13] , outputR[13] , outputB[13]);
  ledStrip.setPixelColor(13 , outputG[14] , outputR[14] , outputB[14]);
  ledStrip.setPixelColor(14 , outputG[15] , outputR[15] , outputB[15]);
  ledStrip.setPixelColor(15 , outputG[16] , outputR[16] , outputB[16]);
  ledStrip.setPixelColor(16 , outputG[17] , outputR[17] , outputB[17]);
  ledStrip.setPixelColor(17 , outputG[18] , outputR[18] , outputB[18]);
  ledStrip.setPixelColor(18 , outputG[19] , outputR[19] , outputB[19]);
  ledStrip.setPixelColor(19 , outputG[20] , outputR[20] , outputB[20]);
  ledStrip.setPixelColor(39 , outputG[21] , outputR[21] , outputB[21]);
  ledStrip.setPixelColor(38 , outputG[22] , outputR[22] , outputB[22]);
  ledStrip.setPixelColor(37 , outputG[23] , outputR[23] , outputB[23]);
  ledStrip.setPixelColor(36 , outputG[24] , outputR[24] , outputB[24]);
  ledStrip.setPixelColor(35 , outputG[25] , outputR[25] , outputB[25]);
  ledStrip.setPixelColor(34 , outputG[26] , outputR[26] , outputB[26]);
  ledStrip.setPixelColor(33 , outputG[27] , outputR[27] , outputB[27]);
  ledStrip.setPixelColor(32 , outputG[28] , outputR[28] , outputB[28]);
  ledStrip.setPixelColor(31 , outputG[29] , outputR[29] , outputB[29]);
  ledStrip.setPixelColor(30 , outputG[30] , outputR[30] , outputB[30]);
  ledStrip.setPixelColor(29 , outputG[31] , outputR[31] , outputB[31]);
  ledStrip.setPixelColor(28 , outputG[32] , outputR[32] , outputB[32]);
  ledStrip.setPixelColor(27 , outputG[33] , outputR[33] , outputB[33]);
  ledStrip.setPixelColor(26 , outputG[34] , outputR[34] , outputB[34]);
  ledStrip.setPixelColor(25 , outputG[35] , outputR[35] , outputB[35]);
  ledStrip.setPixelColor(24 , outputG[36] , outputR[36] , outputB[36]);
  ledStrip.setPixelColor(23 , outputG[37] , outputR[37] , outputB[37]);
  ledStrip.setPixelColor(22 , outputG[38] , outputR[38] , outputB[38]);
  ledStrip.setPixelColor(21 , outputG[39] , outputR[39] , outputB[39]);
  ledStrip.setPixelColor(20 , outputG[40] , outputR[40] , outputB[40]);
  ledStrip.setPixelColor(40 , outputG[41] , outputR[41] , outputB[41]);
  ledStrip.setPixelColor(41 , outputG[42] , outputR[42] , outputB[42]);
  ledStrip.setPixelColor(42 , outputG[43] , outputR[43] , outputB[43]);
  ledStrip.setPixelColor(43 , outputG[44] , outputR[44] , outputB[44]);
  ledStrip.setPixelColor(44 , outputG[45] , outputR[45] , outputB[45]);
  ledStrip.setPixelColor(45 , outputG[46] , outputR[46] , outputB[46]);
  ledStrip.setPixelColor(46 , outputG[47] , outputR[47] , outputB[47]);
  ledStrip.setPixelColor(47 , outputG[48] , outputR[48] , outputB[48]);
  ledStrip.setPixelColor(48 , outputG[49] , outputR[49] , outputB[49]);
  ledStrip.setPixelColor(49 , outputG[50] , outputR[50] , outputB[50]);
  ledStrip.setPixelColor(50 , outputG[51] , outputR[51] , outputB[51]);
  ledStrip.setPixelColor(51 , outputG[52] , outputR[52] , outputB[52]);
  ledStrip.setPixelColor(52 , outputG[53] , outputR[53] , outputB[53]);
  ledStrip.setPixelColor(53 , outputG[54] , outputR[54] , outputB[54]);
  ledStrip.setPixelColor(54 , outputG[55] , outputR[55] , outputB[55]);
  ledStrip.setPixelColor(55 , outputG[56] , outputR[56] , outputB[56]);
  ledStrip.setPixelColor(56 , outputG[57] , outputR[57] , outputB[57]);
  ledStrip.setPixelColor(57 , outputG[58] , outputR[58] , outputB[58]);
  ledStrip.setPixelColor(58 , outputG[59] , outputR[59] , outputB[59]);
  ledStrip.setPixelColor(59 , outputG[60] , outputR[60] , outputB[60]);
  ledStrip.setPixelColor(79 , outputG[61] , outputR[61] , outputB[61]);
  ledStrip.setPixelColor(78 , outputG[62] , outputR[62] , outputB[62]);
  ledStrip.setPixelColor(77 , outputG[63] , outputR[63] , outputB[63]);
  ledStrip.setPixelColor(76 , outputG[64] , outputR[64] , outputB[64]);
  ledStrip.setPixelColor(75 , outputG[65] , outputR[65] , outputB[65]);
  ledStrip.setPixelColor(74 , outputG[66] , outputR[66] , outputB[66]);
  ledStrip.setPixelColor(73 , outputG[67] , outputR[67] , outputB[67]);
  ledStrip.setPixelColor(72 , outputG[68] , outputR[68] , outputB[68]);
  ledStrip.setPixelColor(71 , outputG[69] , outputR[69] , outputB[69]);
  ledStrip.setPixelColor(70 , outputG[70] , outputR[70] , outputB[70]);
  ledStrip.setPixelColor(69 , outputG[71] , outputR[71] , outputB[71]);
  ledStrip.setPixelColor(68 , outputG[72] , outputR[72] , outputB[72]);
  ledStrip.setPixelColor(67 , outputG[73] , outputR[73] , outputB[73]);
  ledStrip.setPixelColor(66 , outputG[74] , outputR[74] , outputB[74]);
  ledStrip.setPixelColor(65 , outputG[75] , outputR[75] , outputB[75]);
  ledStrip.setPixelColor(64 , outputG[76] , outputR[76] , outputB[76]);
  ledStrip.setPixelColor(63 , outputG[77] , outputR[77] , outputB[77]);
  ledStrip.setPixelColor(62 , outputG[78] , outputR[78] , outputB[78]);
  ledStrip.setPixelColor(61 , outputG[79] , outputR[79] , outputB[79]);
  ledStrip.setPixelColor(60 , outputG[80] , outputR[80] , outputB[80]);
  ledStrip.setPixelColor(80 , outputG[81] , outputR[81] , outputB[81]);
  ledStrip.setPixelColor(81 , outputG[82] , outputR[82] , outputB[82]);
  ledStrip.setPixelColor(82 , outputG[83] , outputR[83] , outputB[83]);
  ledStrip.setPixelColor(83 , outputG[84] , outputR[84] , outputB[84]);
  ledStrip.setPixelColor(84 , outputG[85] , outputR[85] , outputB[85]);
  ledStrip.setPixelColor(85 , outputG[86] , outputR[86] , outputB[86]);
  ledStrip.setPixelColor(86 , outputG[87] , outputR[87] , outputB[87]);
  ledStrip.setPixelColor(87 , outputG[88] , outputR[88] , outputB[88]);
  ledStrip.setPixelColor(88 , outputG[89] , outputR[89] , outputB[89]);
  ledStrip.setPixelColor(89 , outputG[90] , outputR[90] , outputB[90]);
  ledStrip.setPixelColor(90 , outputG[91] , outputR[91] , outputB[91]);
  ledStrip.setPixelColor(91 , outputG[92] , outputR[92] , outputB[92]);
  ledStrip.setPixelColor(92 , outputG[93] , outputR[93] , outputB[93]);
  ledStrip.setPixelColor(93 , outputG[94] , outputR[94] , outputB[94]);
  ledStrip.setPixelColor(94 , outputG[95] , outputR[95] , outputB[95]);
  ledStrip.setPixelColor(95 , outputG[96] , outputR[96] , outputB[96]);
  ledStrip.setPixelColor(96 , outputG[97] , outputR[97] , outputB[97]);
  ledStrip.setPixelColor(97 , outputG[98] , outputR[98] , outputB[98]);
  ledStrip.setPixelColor(98 , outputG[99] , outputR[99] , outputB[99]);
  ledStrip.setPixelColor(99 , outputG[100] , outputR[100] , outputB[100]);
  ledStrip.show();
}

boolean letter(int input , int col , int row) {
  boolean output;
  output = false;
  col = col + 10;
  if (col == 11) {
    col = 3;
  } else if (col == 13) {
    col = 1;
  } else {
    col = 2;
  }

  if (input == 'A') {
    if (col == 1 || col == 3) {
      output = true;
    } else if (col == 2) {
      if (row == 1 || row == 3) {
        output = true;
      }
    }
  }
  if (input == 'B') {
    if (col == 1) {
      output = true;
    } else if (col == 2) {
      if (row == 1 || row == 3 || row == 5) {
        output = true;
      }
    } else {
      if (row == 2 || row == 4) {
        output = true;
      }
    }
  }
  if (input == 'C') {
    if (col == 1) {
      output = true;
    } else {
      if (row == 1 || row == 5) {
        output = true;
      }
    }
  }
  if (input == 'D') {
    if (col == 1) {
      output = true;
    } else if (col == 2) {
      if (row == 1 || row == 5) {
        output = true;
      }
    } else if (col == 3) {
      if (row == 2 || row == 3 || row == 4) {
        output = true;
      }
    }
  }
  if (letter == 'E') {
    if (col == 1) {
      output = true;
    } else if (col == 2) {
      if (row == 1 || row == 3 || row == 5) {
        output = true;
      } else {
        if (row == 1 || row == 5) {
          output = true;
        }
      }
    }
  }
  if (letter == 'F') {
    if (col == 1) {
      output = true;
    } else if (col == 2) {
      if (row == 1 || row == 3) {
        output = true;
      }
    } else {
      if (row == 1) {
        output = true;
      }
    }
  }
  if (letter == 'G') {
    if (col == 1) {
      if (row == 2 || row == 3 || row == 4 || row == 5) {
        output = true;
      }
    } else if (col == 2) {
      if (row == 1 || row == 5) {
        output = true;
      }
    } else {
      if (row == 4) {
        output = true;
      }
    }
  }
  if (letter == 'H') {
    if (col == 1 || col == 3) {
      output = true;
    } else {
      if (row == 3) {
        output = true;
      }
    }
  }
  if (letter == 'I') {
    if (col == 1 || col == 3) {
      if (row == 1 || row == 5) {
        output = true;
      }
    } else {
      output = true;
    }
  }
  if (letter == 'J') {
    if (col == 1) {
      if (row == 4) {
        output = true;
      }
    } else if (col == 2) {
      if (row == 5) {
        output = true;
      }
    } else {
      if (row == 1 || row == 3 || row == 4) {
        output = true;
      }
    }
  }
  if (letter == 'K') {
    if (col == 1) {
      output = true;
    } else if (col == 2) {
      if (row == 3) {
        output = true;
      }
    } else {
      if (row == 1 || row == 2 || row == 4 || row == 5) {
        output = true;
      }
    }
  }
  if (letter == 'L') {
    if (col == 1) {
      output = true;
    } else {
      if (row == 5) {
        output = true;
      }
    }
  }
  if (letter == 'M') {
    if (col == 1 || col == 3) {
      output = true;
    } else {
      if (row == 2 || row == 3) {
        output = true;
      }
    }
  }
  if (letter == 'N') {
    if (col == 1 || col == 3) {
      output = true;
    } else {
      if (row == 3 || row == 4) {
        output = true;
      }
    }
  }
  if (letter == 'O') {
    if (col == 1 || col == 3) {
      output = true;
    } else {
      if (row == 1 || row == 5) {
        output = true;
      }
    }
  }
  if (letter == 'P') {
    if (col == 1) {
      output = true;
    } else if (row == 2) {
      if (row == 1 || row == 3) {
        output = true;
      }
    } else {
      if (row == 1 || row == 2 || row == 3) {
        output = true;
      }
    }
  }
  if (letter == 'Q') {
    if (col == 1) {
      if (row == 2 || row == 3 || row == 4) {
        output = true;
      }
    } else if (col == 2) {
      if (row == 1 || row == 5) {
        output = true;
      }
    } else {
      if (row == 2 || row == 3 || row == 4 || row == 5) {
        output = true;
      }
    }
  }
  if (letter == 'R') {
    if (col == 1) {
      output = true;
    } else if (col == 2) {
      if (row == 1 || row == 3) {
        output = true;
      }
    } else {
      if (row == 2 || row == 4 || row == 5) {
        output = true;
      }
    }
  }
  if (letter == 'S') {
    if (col == 1) {
      if (row == 2 || row == 3 || row == 5) {
        output = true;
      }
    } else if (col == 2) {
      if (row == 1 || row == 3 || row == 5) {
        output = true;
      }
    } else {
      if (row == 1 || row == 3 || row == 4) {
        output = true;
      }
    }
  }
  if (letter == 'T') {
    if (col == 1 || col == 3) {
      if (row == 1) {
        output = true;
      }
    } else {
      output = true;
    }
  }
  if (letter == 'U') {
    if (col == 1 || col == 3) {
      output = true;
    } else {
      if (row == 5) {
        output = true;
      }
    }
  }
  if (letter == 'V') {
    if (col == 1) {
      if (row == 1 || row == 2) {
        output = true;
      }
    } else if (col == 2) {
      if (row == 3 || row == 4) {
        output = true;
      }
    } else {
      output = true;
    }
  }
  if (letter == 'W') {
    if (col == 1 || col == 3) {
      output = true;
    } else {
      if (row == 3 || row == 4) {
        output = true;
      }
    }
  }
  if (letter == 'X') {
    if (col == 1 || col == 3) {
      if (row == 1 || row == 2 || row == 4 || row == 5) {
        output = true;
      }
    } else {
      if (row == 3) {
        output = true;
      }
    }
  }
  if (letter == 'Y') {
    if (col == 1 || col == 3) {
      if (row == 1 || row == 2) {
        output = true;
      }
    } else {
      if (row == 3 || row == 4 || row == 5) {
        output = true;
      }
    }
  }
  if (letter == 'Z') {
    if (col == 1) {
      if (row == 1 || row == 4 || row == 5) {
        output = true;
      }
    } else if (col == 2) {
      if (row == 1 || row == 3 || row == 5) {
        output = true;
      }
    } else {
      if (row == 1 || row == 2 || row == 5) {
        output = true;
      }
    }
  }
  return (output);
}

