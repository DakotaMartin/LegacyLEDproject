
int[] outputR = new int[101];
int[] outputG = new int[101];
int[] outputB = new int[101];

// XpixelCount * YpixelCount < 101    <-- Things will break is that is ever false!!!
int XpixelCount = 20;
int YpixelCount = 5;

void setup() {
  size(800, 200);
  surface.setResizable(true);
  frameRate(30);
}

int index;

/*
So this is a simulation of the display so that you can write code for it and test if it will work/see how it will look
 without actually having the box. Everything is done by messing with the values in the 3 arreys outputR, outputG, and outputB.
 
 The top left LED is stored in index 1 (not 0 like how most programmers do it). Run the program and you'll see I've labled every pixel.
 Write your code in the ani functions and uncomment it to run it. I did the first 2 as examples.
 */

void draw() {
  //ani1();
  //ani2();
  ani3();
  //ani4();
  //ani5();
  renderPixels();
}

void ani1() {
  index = round(random(100));//Pick a random pixel
  outputR[index] = round(random(255));//make it a random color.
  outputG[index] = round(random(255));
  outputB[index] = round(random(255));
}

int ani2var;

void ani2() {
  if (ani2var == 0 || ani2var > 100) {//If we're just starting out, or if we've just finnished: go back to the first pixel.
    ani2var = 1;
  }
  outputR[ani2var] = round(random(255));//make the pixel we're on a random color.
  outputG[ani2var] = round(random(255));
  outputB[ani2var] = round(random(255));
  ani2var++;//go to the next pixel.
}

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

void ani4() {
}

void ani5() {
}

int rendercol, renderrow;

void renderPixels() {
  stroke(50);
  index = 1;
  rendercol = 0;
  renderrow = 0;
  while (rendercol < YpixelCount) {
    while (renderrow < XpixelCount) {
      fill(outputR[index], outputG[index], outputB[index]);
      rect((width / XpixelCount) * renderrow, (height / YpixelCount) * rendercol, (width / XpixelCount) * (renderrow + 1), (height / YpixelCount) * (rendercol + 1));
      fill(255);
      text(index, ((width / XpixelCount) * renderrow) + 5, ((height / YpixelCount) * rendercol) + 15);
      renderrow++;
      index++;
    }
    renderrow = 0;
    rendercol++;
  }
}