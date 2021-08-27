/////////////////// DISPLAY /////////////////////////
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1327.h>
#include "Font5x7FixedMono.h"

#define OLED_CLK 13
#define OLED_MOSI 11
#define OLED_CS 10
#define OLED_DC 8
#define OLED_RESET -1

Adafruit_SSD1327 display(128, 128,
                         &SPI, OLED_DC,
                         OLED_RESET, OLED_CS);

void display_setup(void) {
  Serial.println("-> connecting display");
  if ( ! display.begin(0x3D) ) {
    Serial.println("Unable to initialize OLED");
    while (1) yield();
  }

  display.setTextSize(1);
  display.setTextWrap(false);
  display.setTextColor(SSD1327_WHITE);
  display.setFont(&Font5x7FixedMono);
  display.setRotation(1);
  display.clearDisplay();
  display.display();
}

#define ROW_SIMPLE   "+-------------------+"
#define ROW_WAVY     "<~~~~~~~~~~~~~~~~~~~>"
#define ROW_COMMENT  "/*******************/"
#define ROW_X 2
#define ROW_TOP_Y 6
#define ROW_BOT_Y 128
#define COL_STEP 8
#define COL_NUM 15
#define COL_START_Y 15
#define COL_LEFT_X 2
#define COL_RIGHT_X 122

void draw_border(void) {
  display.setCursor(ROW_X, ROW_TOP_Y);
  display.print(ROW_SIMPLE);
  display.setCursor(ROW_X, ROW_BOT_Y);
  display.print(ROW_SIMPLE);

  for (int i = 0; i < COL_NUM - 1; i++) {
    int y = COL_START_Y + (i * COL_STEP);
    display.setCursor(COL_LEFT_X, y);
    display.print("|");
    display.setCursor(COL_RIGHT_X, y);
    display.print("|");
  }
}

void draw_border_comment(void) {
  display.setCursor(ROW_X, ROW_TOP_Y);
  display.print(ROW_COMMENT);
  display.setCursor(ROW_X, ROW_BOT_Y);
  display.print(ROW_COMMENT);

  for (int i = 0; i < COL_NUM - 1; i++) {
    int y = COL_START_Y + (i * COL_STEP);
    display.setCursor(COL_LEFT_X, y);
    display.print("*");
    display.setCursor(COL_RIGHT_X, y);
    display.print("*");
  }
}

void draw_border_wavy(void) {
  display.setCursor(ROW_X, ROW_TOP_Y);
  display.print(ROW_WAVY);
  display.setCursor(ROW_X, ROW_BOT_Y);
  display.print(ROW_WAVY);
  bool even = false;
  for (int i = 0; i < COL_NUM - 1; i++) {
    even = !even;
    int y = COL_START_Y + (i * COL_STEP);
    if (even) {
      display.setCursor(COL_LEFT_X, y);
      display.print("\\");
      display.setCursor(COL_RIGHT_X, y);
      display.print("/");
    } else {
      display.setCursor(COL_LEFT_X, y);
      display.print("/");
      display.setCursor(COL_RIGHT_X, y);
      display.print("\\");
    }
  }
}

/////////////////// INPUT ///////////////////////////
#include <Bounce.h>
#define BOUNCE_MS 10

#define NUM_BUTTONS 3
#define B0 0
#define B1 1
#define B2 2

void buttons_setup(void) {
  Serial.println("-> connecting buttons");
  pinMode(B0, INPUT_PULLUP);
  pinMode(B1, INPUT_PULLUP);
  pinMode(B2, INPUT_PULLUP);
}

Bounce buttons[NUM_BUTTONS] = {
  Bounce(B0, BOUNCE_MS),
  Bounce(B1, BOUNCE_MS),
  Bounce(B2, BOUNCE_MS)
};

bool held[NUM_BUTTONS] = { false, false, false };

bool buttons_update(void) {
  bool result0 = buttons[0].update();
  bool result1 = buttons[1].update();
  bool result2 = buttons[2].update();
  return (result0 && result1 && result2);
}

/*
   REMINDER:
    - myButton.fallingEdge() == ON
    - myButton.risingEdge() == OFF
    ...because our buttons are wired active LOW
*/


void handle_inputs(void) {
  buttons_update();
  if (buttons[0].fallingEdge()) {
    held[0] = true;
  } else if (buttons[0].risingEdge()) {
    held[0] = false;
  }

  if (buttons[1].fallingEdge()) {
    held[1] = true;
  } else if (buttons[1].risingEdge()) {
    held[1] = false;
  }

  if (buttons[2].fallingEdge()) {
    held[2] = true;
  } else if (buttons[2].risingEdge()) {
    held[2] = false;
  }
}

/////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting...");

  // CONNECT BUTTONS
  buttons_setup();

  // CONNECT DISPLAY
  display_setup();

  // LOCAL
}

enum BORDERS {NONE, NORMAL, WAVY, COMMENT, NUM_BORDERS};
int current_border = NONE;
void next_border(void) {
  current_border += 1;
  if (current_border == NUM_BORDERS) current_border = NONE;
}

enum TRIANGLE_POINTS {X1, Y1, X2, Y2, X3, Y3};
int  tri_points[6] = {
  54, 20,
  58, 46,
  84, 34
};

void tri_draw(int x_offset = 0, int y_offset = 0) {
  display.fillTriangle(
    tri_points[X1] + x_offset, tri_points[Y1] + y_offset,
    tri_points[X2] + x_offset, tri_points[Y2] + y_offset,
    tri_points[X3] + x_offset, tri_points[Y3] + y_offset,
    SSD1327_BLACK
  );
  display.drawTriangle(
    tri_points[X1] + x_offset, tri_points[Y1] + y_offset,
    tri_points[X2] + x_offset, tri_points[Y2] + y_offset,
    tri_points[X3] + x_offset, tri_points[Y3] + y_offset,
    SSD1327_WHITE
  );
}

void tri_fill(void) {
  display.fillTriangle(
    tri_points[X1], tri_points[Y1],
    tri_points[X2], tri_points[Y2],
    tri_points[X3], tri_points[Y3],
    SSD1327_WHITE
  );
}

int tri_center_x(int offset_x = 0) {
  return ((3 * offset_x) + tri_points[X1] + tri_points[X2] + tri_points[X3]) / 3;
}

int tri_center_y(int offset_y = 0) {
  return ((3 * offset_y) + tri_points[Y1] + tri_points[Y2] + tri_points[Y3]) / 3;
}

#define CIRCLE_RADIUS 10
// array of center coordinates
int grid[4][4][2] {
  {{22, 20}, {48, 20}, {74, 20}, {100, 20}},
  {{22, 46}, {48, 46}, {74, 46}, {100, 46}},
  {{22, 72}, {48, 72}, {74, 72}, {100, 72}},
  {{22, 98}, {48, 98}, {74, 98}, {100, 98}}
};



#define cols 4
#define rows cols

double theta = 0;
double polar_xs[rows] = { 0, 0, 0 };
double polar_ys[cols] = { 1, 1, 1 };

#define DELTA_MAX 0.2
#define DELTA_MIN 0.02
#define DELTA_STEP 0.01
float delta = 0.083;

void inc_delta(void) {
  if (delta + DELTA_STEP < DELTA_MAX) delta += DELTA_STEP;
}

void dec_delta(void) {
  if (delta - DELTA_STEP > DELTA_MIN) delta -= DELTA_STEP;
}

enum SCALES { MULT, PYTHAG, JUST, NUM_SCALES };
double scales[NUM_SCALES][cols][2] = {
  // MULT
  { {1, 1}, {2, 2}, {3, 3}, {4, 4} },
  // PYTHAG
  { {1, 1}, {9, 8}, {4, 3}, {3, 2} },
  // JUST
  { {1, 1}, {9, 8}, {6, 5}, {5, 4} },
};

int current_scale = MULT;
void next_scale(void) {
  current_scale++;
  if (current_scale == NUM_SCALES) current_scale = MULT;
  clear_pixels();
}

uint8_t tick = 0;

#define PIXEL_NUM 96

uint8_t pixels[PIXEL_NUM][cols][rows][3];
void clear_pixels(void) {
  for (int pn = 0; pn < PIXEL_NUM; pn++) {
    for (int j = 0; j < cols; j++) {
      for (int k = 0; k < rows; k++) {
        pixels[pn][j][k][0] = 0;
        pixels[pn][j][k][1] = 0;
        pixels[pn][j][k][2] = 0;
      }
    }
  }
}

int pixel_choke = 1;
void inc_choke(void) {
  if (pixel_choke + 1 <= PIXEL_NUM) pixel_choke++;
}

void dec_choke(void) {
  if (pixel_choke - 1 > 1) pixel_choke--;
}

int current_pixel = 0;
int oldest_pixel = PIXEL_NUM - 1;
void add_pixel(int col, int row, uint8_t x, uint8_t y, uint8_t z) {
  uint8_t pixel[3] = { x, y, z };
  pixels[oldest_pixel][col][row][0] = pixel[0];
  pixels[oldest_pixel][col][row][1] = pixel[1];
  pixels[oldest_pixel][col][row][2] = pixel[2];
}

void advance_pixel(void) {
  oldest_pixel--;
  if (oldest_pixel < 0) oldest_pixel = PIXEL_NUM - 1;
  current_pixel--;
  if (current_pixel < 0) current_pixel = PIXEL_NUM - 1;
}

#define pixel_A pixels[current_pixel][0][0][0], pixels[current_pixel][0][0][1]
#define pixel_B pixels[current_pixel][0][1][0], pixels[current_pixel][0][1][1]
#define pixel_C pixels[current_pixel][0][2][0], pixels[current_pixel][0][2][1]
#define pixel_D pixels[current_pixel][0][3][0], pixels[current_pixel][0][3][1]
#define pixel_E pixels[current_pixel][1][0][0], pixels[current_pixel][1][0][1]
#define pixel_F pixels[current_pixel][1][1][0], pixels[current_pixel][1][1][1]
#define pixel_G pixels[current_pixel][1][2][0], pixels[current_pixel][1][2][1]
#define pixel_H pixels[current_pixel][1][3][0], pixels[current_pixel][1][3][1]
#define pixel_I pixels[current_pixel][2][0][0], pixels[current_pixel][2][0][1]
#define pixel_J pixels[current_pixel][2][1][0], pixels[current_pixel][2][1][1]
#define pixel_K pixels[current_pixel][2][2][0], pixels[current_pixel][2][2][1]
#define pixel_L pixels[current_pixel][2][3][0], pixels[current_pixel][2][3][1]
#define pixel_M pixels[current_pixel][3][0][0], pixels[current_pixel][3][0][1]
#define pixel_N pixels[current_pixel][3][1][0], pixels[current_pixel][3][1][1]
#define pixel_O pixels[current_pixel][3][2][0], pixels[current_pixel][3][2][1]
#define pixel_P pixels[current_pixel][3][3][0], pixels[current_pixel][3][3][1]

void loop() {
  tick++;
  //bool every_other = tick % 2 == 0;
  //bool every_third = tick % 3 == 0;
  bool every_fourth  = tick % 4  == 0;

  handle_inputs();

  if (buttons[2].risingEdge() && (!held[0] && !held[1])) next_scale();
  if (held[2]) {
    if (buttons[0].fallingEdge() || (held[0])) dec_choke();
    if (buttons[1].fallingEdge() || (held[1])) inc_choke();
  } else {
    if (buttons[0].fallingEdge() || (every_fourth && held[0])) dec_delta();
    if (buttons[1].fallingEdge() || (every_fourth && held[1])) inc_delta();
  }

  theta += delta;

  // REDRAW
  display.clearDisplay();
  display.setTextColor(int(12 * cos(theta / 4)));

  for (int i = 0; i < cols; i++) {
    //    display.drawCircle(
    //      grid[0][i+1][0],
    //      grid[0][i+1][1],
    //      CIRCLE_RADIUS - 1,4
    //    );
    //
    //    display.drawCircle(
    //      grid[i+1][0][0],
    //      grid[i+1][0][1],
    //      CIRCLE_RADIUS - 1,4
    //    );

    polar_xs[i] = sin(theta * (scales[current_scale][i][0])) * CIRCLE_RADIUS;
    polar_ys[i] = sin(theta * (scales[current_scale][i][1])) * CIRCLE_RADIUS;



    //    display.drawLine(
    //      grid[0][i][0] + int(polar_xs[i]),
    //      grid[0][i][0] + int(polar_ys[i]) - 72,
    //      grid[0][i][0] + int(polar_xs[i]),
    //      grid[0][i][0] + int(polar_ys[i]) + 72,
    //      int(5 * polar_xs[i] / CIRCLE_RADIUS)
    //    );
    //    display.drawLine(
    //      grid[0][i][0] + int(polar_xs[i]) - 72,
    //      grid[0][i][0] + int(polar_ys[i]),
    //      grid[0][i][0] + int(polar_xs[i]) + 72,
    //      grid[0][i][0] + int(polar_ys[i]),
    //      int(5 * polar_ys[i] / CIRCLE_RADIUS)
    //    );
  }

  for (int j = 0; j < cols; j++) {
    for (int k = 0; k < rows; k++) {
      add_pixel(
        j, k,
        grid[j][k][0] + int(polar_xs[j]),
        grid[j][k][1] + int(polar_ys[k]),
        60
      );

      display.drawCircle(
        grid[j][k][0] + int(polar_xs[j]),
        grid[j][k][1] + int(polar_ys[k]),
        2, SSD1327_WHITE
      );

      // DRAW EVERYTHING
      for (int p = 0; p < pixel_choke; p++) {
        int pixel = p + current_pixel;
        if (pixel > PIXEL_NUM - 1) pixel = pixel % PIXEL_NUM - 1;
        display.drawPixel(
          pixels[pixel][j][k][0],
          pixels[pixel][j][k][1],
          pixels[pixel][j][k][2] / 4
        );
        pixels[pixel][j][k][2]--;
      }

      // DRAW LINES
      // using named macros for each current pixel
      /*
       *    A   B   C   D
       *    
       *    E   F   G   H
       *    
       *    I   J   K   L
       *    
       *    M   N   O   P
       */
      
      // row ABCD
      display.drawLine(pixel_A, pixel_B, 8);
      display.drawLine(pixel_B, pixel_C, 8);
      display.drawLine(pixel_C, pixel_D, 8);

      // row EFGH
      display.drawLine(pixel_E, pixel_F, 8);
      display.drawLine(pixel_F, pixel_G, 8);
      display.drawLine(pixel_G, pixel_H, 8);

      // row IJKL
      display.drawLine(pixel_I, pixel_J, 8);
      display.drawLine(pixel_J, pixel_K, 8);
      display.drawLine(pixel_K, pixel_L, 8);

      // row MNOP
      display.drawLine(pixel_M, pixel_N, 8);
      display.drawLine(pixel_N, pixel_O, 8);
      display.drawLine(pixel_O, pixel_P, 8);

      // col AEIM
      display.drawLine(pixel_A, pixel_E, 8);
      display.drawLine(pixel_E, pixel_I, 8);
      display.drawLine(pixel_I, pixel_M, 8);
      
      // col BFJN
      display.drawLine(pixel_B, pixel_F, 8);
      display.drawLine(pixel_F, pixel_J, 8);
      display.drawLine(pixel_J, pixel_N, 8);
      
      // col CGKO
      display.drawLine(pixel_C, pixel_G, 8);
      display.drawLine(pixel_G, pixel_K, 8);
      display.drawLine(pixel_K, pixel_O, 8);
      
      // col DHLP
      display.drawLine(pixel_D, pixel_H, 8);
      display.drawLine(pixel_H, pixel_L, 8);
      display.drawLine(pixel_L, pixel_P, 8);
    }
  }

  advance_pixel();

  //  switch (current_border) {
  //    case NORMAL:
  //      draw_border();
  //      break;
  //    case WAVY:
  //      draw_border_wavy();
  //      break;
  //    case COMMENT:
  //      draw_border_comment();
  //      break;
  //    case NONE:
  //      break;
  //  }

  //display.setCursor(12,123);
  //display.printf("sin(%3.1f) %1.2f", theta, sin(theta));
  
  // TODO better timing model
  delay(40);
  display.display();

}
